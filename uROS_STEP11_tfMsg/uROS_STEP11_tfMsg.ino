// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// clang-format off
// メッセージヘッダーファイルを見つけるため、micro_ros_arduino.hを先にインクルードすること
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <stdio.h>
#include <tf2_msgs/msg/tf_message.h>
// clang-format on

geometry_msgs__msg__Twist msg;
tf2_msgs__msg__TFMessage * tf_message;
sensor_msgs__msg__JointState jstate;
rosidl_runtime_c__String joint_name[2];
double positions[2];

rcl_subscription_t subscriber;
rcl_publisher_t publisher, publisher2;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define LED0 1
#define LED1 2
#define LED2 42
#define LED3 41

#define MOTOR_EN 9
#define CW_R 14
#define CW_L 21
#define PWM_R 13
#define PWM_L 45

#define MIN_HZ 80
#define TIRE_DIAMETER (48.00)
#define PULSE (TIRE_DIAMETER * PI / 400.0)
#define MIN_SPEED (MIN_HZ * PULSE)
#define TREAD_WIDTH (65.0)

hw_timer_t * timer0 = NULL;
hw_timer_t * timer2 = NULL;
hw_timer_t * timer3 = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned short R_STEP_HZ = MIN_HZ;
unsigned short L_STEP_HZ = MIN_HZ;

volatile unsigned int step_r, step_l;

volatile double max_speed = MIN_SPEED;
volatile double min_speed = MIN_SPEED;
volatile double r_accel = 0.0;
volatile double speed = 0.0;

double max_omega;
double min_omega;
double r_acc_omega = 0.0;
double omega = 0;
double position_r, position_l;

double odom_x, odom_y, odom_theta;

volatile bool motor_move = 0;

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }

void error_loop()
{
  while (1) {
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

const void euler_to_quat(float x, float y, float z, double * q)
{
  float c1 = cos(y / 2);
  float c2 = cos(z / 2);
  float c3 = cos(x / 2);

  float s1 = sin(y / 2);
  float s2 = sin(z / 2);
  float s3 = sin(x / 2);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

//割り込み
//目標値の更新周期1kHz
void IRAM_ATTR onTimer0(void)
{
  portENTER_CRITICAL_ISR(&timerMux);  //割り込み禁止
  control_interrupt();
  portEXIT_CRITICAL_ISR(&timerMux);  //割り込み許可
}

//Rモータの周期数割り込み
void IRAM_ATTR isr_r(void)
{
  portENTER_CRITICAL_ISR(&timerMux);  //割り込み禁止
  if (motor_move) {
    if (R_STEP_HZ < 30) R_STEP_HZ = 30;
    timerAlarmWrite(timer2, 2000000 / R_STEP_HZ, true);
    digitalWrite(PWM_R, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    }
    digitalWrite(PWM_R, LOW);
    step_r++;
  }
  portEXIT_CRITICAL_ISR(&timerMux);  //割り込み許可
}

//Lモータの周期数割り込み
void IRAM_ATTR isr_l(void)
{
  portENTER_CRITICAL_ISR(&timerMux);  //割り込み禁止
  if (motor_move) {
    if (L_STEP_HZ < 30) L_STEP_HZ = 30;
    timerAlarmWrite(timer3, 2000000 / L_STEP_HZ, true);
    digitalWrite(PWM_L, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    };
    digitalWrite(PWM_L, LOW);
    step_l++;
  }
  portEXIT_CRITICAL_ISR(&timerMux);  //割り込み許可
}

//twist message cb
void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  //linearは[m/s]の単位で入力されるのでPi:Coのシステムに合わせて[mm/s]にする
  speed = msg->linear.x * 1000.0;
  omega = msg->angular.z;
}

void setup()
{
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  digitalWrite(LED1, HIGH);
  set_microros_wifi_transports("使用するWiFiのAP名", "Wi-Fiのパスワード", "PCのIPアドレス", 8888);

  digitalWrite(LED2, HIGH);

  //motor disable
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(CW_R, OUTPUT);
  pinMode(CW_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  digitalWrite(MOTOR_EN, LOW);
  digitalWrite(CW_R, LOW);
  digitalWrite(CW_L, LOW);
  digitalWrite(PWM_R, LOW);
  digitalWrite(PWM_L, LOW);

  delay(2000);

  timer0 = timerBegin(0, 80, true);  //1us
  timerAttachInterrupt(timer0, &onTimer0, true);
  timerAlarmWrite(timer0, 1000, true);  //1kHz
  timerAlarmEnable(timer0);

  timer2 = timerBegin(2, 40, true);  //0.5us
  timerAttachInterrupt(timer2, &isr_r, true);
  timerAlarmWrite(timer2, 13333, true);  //150Hz
  timerAlarmEnable(timer2);

  timer3 = timerBegin(3, 40, true);  //0.5us
  timerAttachInterrupt(timer3, &isr_l, true);
  timerAlarmWrite(timer3, 13333, true);  //150Hz
  timerAlarmEnable(timer3);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_pico_node", "", &support));

  //publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf"));

  RCCHECK(rclc_publisher_init_default(
    &publisher2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

  //Subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  tf_message = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 1);

  tf_message->transforms.data[0].header.frame_id.data = "/odom";
  tf_message->transforms.data[0].header.frame_id.size =
    strlen(tf_message->transforms.data[0].header.frame_id.data);
  tf_message->transforms.data[0].header.frame_id.capacity = 100;

  tf_message->transforms.data[0].child_frame_id.data = "/base_footprint";
  tf_message->transforms.data[0].child_frame_id.size =
    strlen(tf_message->transforms.data[0].child_frame_id.data);
  tf_message->transforms.data[0].child_frame_id.capacity = 100;

  joint_name[0].data = "left_wheel_joint";
  joint_name[0].size = strlen(joint_name[0].data);
  joint_name[0].capacity = joint_name[0].size + 1;

  joint_name[1].data = "right_wheel_joint";
  joint_name[1].size = strlen(joint_name[1].data);
  joint_name[1].capacity = joint_name[1].size + 1;

  jstate.name.data = joint_name;
  jstate.name.size = 2;
  jstate.name.capacity = 2;
  jstate.position.data = positions;
  jstate.position.size = 2;
  jstate.position.capacity = 2;

  digitalWrite(MOTOR_EN, HIGH);
}

void loop()
{
  double q[4];
  uint32_t current = micros();

  jstate.header.stamp.sec = current / 1000000;
  jstate.header.stamp.nanosec = current - jstate.header.stamp.sec * 1000000;

  euler_to_quat(0, 0, odom_theta, q);
  tf_message->transforms.data[0].transform.translation.x = odom_x;
  tf_message->transforms.data[0].transform.translation.y = odom_y;
  tf_message->transforms.data[0].transform.translation.z = 0.0;

  tf_message->transforms.data[0].transform.rotation.x = (double)q[1];
  tf_message->transforms.data[0].transform.rotation.y = (double)q[2];
  tf_message->transforms.data[0].transform.rotation.z = (double)q[3];
  tf_message->transforms.data[0].transform.rotation.w = (double)q[0];
  tf_message->transforms.data[0].header.stamp.nanosec = jstate.header.stamp.nanosec;
  tf_message->transforms.data[0].header.stamp.sec = jstate.header.stamp.sec;

  jstate.position.data[0] = position_l;
  jstate.position.data[1] = position_r;

  RCSOFTCHECK(rcl_publish(&publisher, tf_message, NULL));
  RCSOFTCHECK(rcl_publish(&publisher2, &jstate, NULL));

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  //for subscription

  delay(10);
}
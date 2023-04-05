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


#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
geometry_msgs__msg__Twist msg;

rcl_subscription_t subscriber;
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

hw_timer_t* timer0 = NULL;
hw_timer_t* timer2 = NULL;
hw_timer_t* timer3 = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned short RStepHz = MIN_HZ;
unsigned short LStepHz = MIN_HZ;

volatile unsigned int StepR, StepL;

volatile double max_speed = MIN_SPEED;
volatile double min_speed = MIN_SPEED;
volatile double r_accel = 0.0;
volatile double speed = 0.0;

double max_omega;
double min_omega;
double r_acc_omega = 0.0;
double omega = 0;

volatile bool motor_move = 0;

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

//割り込み
//目標値の更新周期1kHz
void IRAM_ATTR OnTimer0(void) {
  portENTER_CRITICAL_ISR(&timerMux);  //割り込み禁止
  control_interrupt();
  portEXIT_CRITICAL_ISR(&timerMux);  //割り込み許可
}

//Rモータの周期数割り込み
void IRAM_ATTR IsrR(void) {
  portENTER_CRITICAL_ISR(&timerMux);  //割り込み禁止
  if (motor_move) {
    if (RStepHz < 30) RStepHz = 30;
    timerAlarmWrite(timer2, 2000000 / RStepHz, true);
    digitalWrite(PWM_R, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    }
    digitalWrite(PWM_R, LOW);
    StepR++;
  }
  portEXIT_CRITICAL_ISR(&timerMux);  //割り込み許可
}

//Lモータの周期数割り込み
void IRAM_ATTR IsrL(void) {
  portENTER_CRITICAL_ISR(&timerMux);  //割り込み禁止
  if (motor_move) {
    if (LStepHz < 30) LStepHz = 30;
    timerAlarmWrite(timer3, 2000000 / LStepHz, true);
    digitalWrite(PWM_L, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    };
    digitalWrite(PWM_L, LOW);
    StepL++;
  }
  portEXIT_CRITICAL_ISR(&timerMux);  //割り込み許可
}


//twist message cb
void subscription_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;

  speed = msg->linear.x * 1000.0;
  omega = msg->angular.z;

}

void setup() {
  //  set_microros_transports();
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
  timerAttachInterrupt(timer0, &OnTimer0, true);
  timerAlarmWrite(timer0, 1000, true);  //1kHz
  timerAlarmEnable(timer0);

  timer2 = timerBegin(2, 40, true);  //0.5us
  timerAttachInterrupt(timer2, &IsrR, true);
  timerAlarmWrite(timer2, 13333, true);  //150Hz
  timerAlarmEnable(timer2);

  timer3 = timerBegin(3, 40, true);  //0.5us
  timerAttachInterrupt(timer3, &IsrL, true);
  timerAlarmWrite(timer3, 13333, true);  //150Hz
  timerAlarmEnable(timer3);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_pico_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  digitalWrite(MOTOR_EN, HIGH);
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

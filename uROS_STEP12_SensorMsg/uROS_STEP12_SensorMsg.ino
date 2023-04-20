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
#include <pico_msgs/msg/light_sensor.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include <std_msgs/msg/int16.h>
// clang-format off

pico_msgs__msg__LightSensor g_sensor_msg;
std_msgs__msg__Int16 g_bat_msg;

rcl_publisher_t g_publisher_sensor, g_publisher_battery;
rclc_support_t g_support;
rcl_allocator_t g_allocator;
rcl_node_t g_node;

#define LED0 1
#define LED1 2
#define LED2 42
#define LED3 41

#define SLED_FR 16
#define SLED_FL 15
#define SLED_R 18
#define SLED_L 17

#define AD4 7
#define AD3 6
#define AD2 5
#define AD1 4
#define AD0 8

hw_timer_t * g_timer1 = NULL;

portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

volatile short g_sensor_value_fr;
volatile short g_sensor_value_fl;
volatile short g_sensor_value_r;
volatile short g_sensor_value_l;
volatile short g_battery_value;

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      errorLoop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }

void errorLoop()
{
  while (1) {
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

void IRAM_ATTR onTimer1(void)
{
  static char cnt = 0;
  portENTER_CRITICAL_ISR(&g_timer_mux);
  switch (cnt) {
    case 0:
      digitalWrite(SLED_FR, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      g_sensor_value_fr = analogRead(AD1);
      digitalWrite(SLED_FR, LOW);  //LED消灯
      break;
    case 1:
      digitalWrite(SLED_FL, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      g_sensor_value_fl = analogRead(AD2);
      digitalWrite(SLED_FL, LOW);  //LED消灯
      break;
    case 2:
      digitalWrite(SLED_R, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      g_sensor_value_r = analogRead(AD3);
      digitalWrite(SLED_R, LOW);  //LED消灯
      break;
    case 3:
      digitalWrite(SLED_L, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      g_sensor_value_l = analogRead(AD4);
      digitalWrite(SLED_L, LOW);  //LED消灯
      g_battery_value = (double)analogReadMilliVolts(AD0) / 10.0 * (10.0 + 51.0);
      break;
    default:
      Serial.printf("error\n\r");
      break;
  }
  cnt++;
  if (cnt == 4) cnt = 0;
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void publisherTask(void * pvParameters)
{
  while (1) {
    g_sensor_msg.forward_r = g_sensor_value_fr;
    g_sensor_msg.forward_l = g_sensor_value_fl;
    g_sensor_msg.right = g_sensor_value_r;
    g_sensor_msg.left = g_sensor_value_l;
    g_bat_msg.data = g_battery_value;
    RCSOFTCHECK(rcl_publish(&g_publisher_sensor, &g_sensor_msg, NULL));
    RCSOFTCHECK(rcl_publish(&g_publisher_battery, &g_bat_msg, NULL));
    delay(10);
  }
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

  //Sensor 発光off
  pinMode(SLED_FR, OUTPUT);
  pinMode(SLED_FL, OUTPUT);
  pinMode(SLED_R, OUTPUT);
  pinMode(SLED_L, OUTPUT);
  digitalWrite(SLED_FR, LOW);
  digitalWrite(SLED_FL, LOW);
  digitalWrite(SLED_R, LOW);
  digitalWrite(SLED_L, LOW);

  delay(2000);

  g_timer1 = timerBegin(1, 80, true);  //1us
  timerAttachInterrupt(g_timer1, &onTimer1, true);
  timerAlarmWrite(g_timer1, 250, true);  //4kHz
  timerAlarmEnable(g_timer1);

  g_allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&g_support, 0, NULL, &g_allocator));

  // create g_node
  RCCHECK(rclc_node_init_default(&g_node, "micro_ros_pico_node", "", &g_support));

  // create g_publisher
  RCCHECK(rclc_publisher_init_default(
    &g_publisher_sensor, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(pico_msgs, msg, LightSensor),
    "pico_sensor"));

  RCCHECK(rclc_publisher_init_default(
    &g_publisher_battery, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "pico_battery"));

  xTaskCreateUniversal(
    //  xTaskCreatePinnedToCore(
    publisherTask, "publisherTask", 4096, NULL, 1, NULL,
    //    PRO_CPU_NUM
    //    APP_CPU_NUM
    CONFIG_ARDUINO_RUNNING_CORE);
}

void loop() { delay(10); }

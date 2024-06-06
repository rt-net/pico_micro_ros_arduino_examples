// Copyright 2024 RT Corporation
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

//microROSを使用しないときはコメントアウトする
#define USE_MICRO_ROS

//マイクロマウスで使用するヘッダ
#include "FS.h"
#include "SPIFFS.h"
#include "device.h"
#include "map_manager.h"
#include "mytypedef.h"
#include "parameter.h"

//micro-ROSで使用するヘッダ
// clang-format off
// メッセージヘッダーファイルを見つけるため、micro_ros_arduino.hを先にインクルードすること
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <pico_msgs/msg/light_sensor.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int16.h>
#include <stdio.h>
#include <tf2_msgs/msg/tf_message.h>
#include <visualization_msgs/msg/marker.h>
// clang-format off

//micro-ROSで使用する変数
volatile double g_position_r, g_position_l;
volatile double g_odom_x, g_odom_y, g_odom_theta;
short g_publish_x, g_publish_y;

volatile bool g_fast_task;              //最短経路のマーカーを表示するときに使用
volatile int g_start_x, g_start_y;        //最短経路のマーカーの初期値で使用
volatile t_direction_glob g_start_dir;  //最短経路のマーカーの初期値で使用

//マイクロマウスで使用する変数
signed char g_mode;
short g_battery_value;
t_sensor g_sen_r, g_sen_l, g_sen_fr, g_sen_fl;
t_control g_con_wall;
volatile double g_accel;
volatile double g_max_speed, g_min_speed;
volatile double g_speed = 0.0;
volatile bool g_motor_move;
MapManager g_map_control;

volatile bool g_theta_adj;

void setup()
{
  // put your setup code here, to run once:
  initDevice();

  g_sen_r.ref = REF_SEN_R;
  g_sen_l.ref = REF_SEN_L;
  g_sen_r.th_wall = TH_SEN_R;
  g_sen_l.th_wall = TH_SEN_L;

  g_sen_fr.th_wall = TH_SEN_FR;
  g_sen_fl.th_wall = TH_SEN_FL;

  g_sen_r.th_control = CONTROL_TH_SEN_R;
  g_sen_l.th_control = CONTROL_TH_SEN_L;

  g_con_wall.kp = CON_WALL_KP;

  g_map_control.setGoalX(GOAL_X);
  g_map_control.setGoalY(GOAL_Y);

  g_motor_move = false;
  g_fast_task = false;
  g_theta_adj = false;
  setRStepHz(MIN_SPEED);
  setLStepHz(MIN_SPEED);

#if defined(USE_MICRO_ROS)
  initMicroROS();
#endif
  g_mode = 1;
}

void loop()
{
  // put your main code here, to run repeatedly:
  setLED(g_mode);
  switch (getSW()) {
    case SW_LM:
      g_mode = decButton(g_mode, 1, 15);
      break;
    case SW_RM:
      g_mode = incButton(g_mode, 15, 1);
      break;
    case SW_CM:
      okButton();
      execByMode(g_mode);
      break;
    default:
      break;
  }
  delay(1);
}

void execByMode(int mode)
{
  enableMotor();
  delay(1000);

  controlInterruptStop();
  g_odom_x = 0;
  g_odom_y = -0.0;
  g_odom_theta = 0.0;
  controlInterruptStart();

  switch (g_mode) {
    case 1:
      searchLefthand();
      break;
    case 2:  //足立法
      g_map_control.positionInit();
      searchAdachi(g_map_control.getGoalX(), g_map_control.getGoalY());
      rotate(right, 2);
      g_map_control.nextDir(right);
      g_map_control.nextDir(right);
      goalAppeal();
      searchAdachi(0, 0);
      rotate(right, 2);
      g_map_control.nextDir(right);
      g_map_control.nextDir(right);
      mapWrite();
      break;
    case 3:  //最短走行
      copyMap();
      g_map_control.positionInit();
      fastRun(g_map_control.getGoalX(), g_map_control.getGoalY());
      rotate(right, 2);
      g_map_control.nextDir(right);
      g_map_control.nextDir(right);
      goalAppeal();
      fastRun(0, 0);
      rotate(right, 2);
      g_map_control.nextDir(right);
      g_map_control.nextDir(right);
      break;
    case 4:
      break;
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
    case 10:
      break;
    case 11:
      break;
    case 12:
      break;
    case 13:
      break;
    case 14:
      break;
    case 15:
      disableMotor();
      adjustMenu();  //調整メニューに行く
      break;
    default:
      Serial.printf("SelectMode input Error\n\r");
      break;
  }
  disableMotor();
}

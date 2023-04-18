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
volatile double position_r, position_l;
volatile double odom_x, odom_y, odom_theta;
short publish_x, publish_y;

volatile bool fast_task;              //最短経路のマーカーを表示するときに使用
volatile int start_x, start_y;        //最短経路のマーカーの初期値で使用
volatile t_direction_glob start_dir;  //最短経路のマーカーの初期値で使用

//マイクロマウスで使用する変数
signed char __mode;
short battery_value;
t_sensor sen_r, sen_l, sen_fr, sen_fl;
t_control con_wall;
volatile double r_accel;
volatile double max_speed, min_speed;
volatile double speed = 0.0;
volatile bool motor_move;
map_manager map_control;

volatile bool theta_adj;

void setup()
{
  // put your setup code here, to run once:
  InitDevice();

  sen_r.ref = REF_SEN_R;
  sen_l.ref = REF_SEN_L;
  sen_r.th_wall = TH_SEN_R;
  sen_l.th_wall = TH_SEN_L;

  sen_fr.th_wall = TH_SEN_FR;
  sen_fl.th_wall = TH_SEN_FL;

  sen_r.th_control = CONTROL_TH_SEN_R;
  sen_l.th_control = CONTROL_TH_SEN_L;

  con_wall.kp = CON_WALL_KP;

  map_control.set_goal_x(GOAL_X);
  map_control.set_goal_y(GOAL_Y);

  motor_move = false;
  fast_task = false;
  theta_adj = false;
  SetRStepHz(MIN_SPEED);
  SetLStepHz(MIN_SPEED);

#if defined(USE_MICRO_ROS)
  init_microROS();
#endif
  __mode = 1;
}

void loop()
{
  // put your main code here, to run repeatedly:
  SetLED(__mode);
  switch (GetSW()) {
    case SW_LM:
      __mode = dec_button(__mode, 1, 15);
      break;
    case SW_RM:
      __mode = inc_button(__mode, 15, 1);
      break;
    case SW_CM:
      ok_button();
      exec_by_mode(__mode);
      break;
  }
  delay(1);
}

void exec_by_mode(int _mode)
{
  EnableMotor();
  delay(1000);

  ControlInterruptStop();
  odom_x = 0;
  odom_y = -0.0;
  odom_theta = 0.0;
  ControlInterruptStart();

  switch (__mode) {
    case 1:
      search_lefthand();
      break;
    case 2:  //足立法
      map_control.position_init();
      search_adachi(map_control.get_goal_x(), map_control.get_goal_y());
      rotate(right, 2);
      map_control.next_dir(right);
      map_control.next_dir(right);
      goal_appeal();
      search_adachi(0, 0);
      rotate(right, 2);
      map_control.next_dir(right);
      map_control.next_dir(right);
      map_write();
      break;
    case 3:  //最短走行
      copy_map();
      map_control.position_init();
      fast_run(map_control.get_goal_x(), map_control.get_goal_y());
      rotate(right, 2);
      map_control.next_dir(right);
      map_control.next_dir(right);
      goal_appeal();
      fast_run(0, 0);
      rotate(right, 2);
      map_control.next_dir(right);
      map_control.next_dir(right);
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
      DisableMotor();
      adjust_menu();  //調整メニューに行く
      break;
  }
  DisableMotor();
}

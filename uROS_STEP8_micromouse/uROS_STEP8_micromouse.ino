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

#include "FS.h"
#include "SPIFFS.h"
#include "device.h"
#include "map_manager.h"
#include "mytypedef.h"
#include "parameter.h"

signed char __mode;
short battery_value;
t_sensor sen_r, sen_l, sen_fr, sen_fl;
t_control con_wall;

volatile double r_accel;
double max_speed, min_speed;
volatile double speed;
volatile bool motor_move;
map_manager map_control;

void setup()
{
  // put your setup code here, to run once:

  InitAll();

  DisableBuzzer();
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

  switch (_mode) {
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

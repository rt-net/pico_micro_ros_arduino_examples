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
/*
volatile unsigned int timer_cnt=0;

unsigned int Get_timer_counter(void){
  return timer_cnt;
}
*/

void control_interrupt(void)
{
  double spd_r, spd_l, omega;
  static char temp_cnt = 0;
  short temp_xy;

  speed += r_accel;

  if (speed > max_speed) {
    speed = max_speed;
  }

  if (motor_move == 0) {
    speed = 0.0;
  } else if (speed < min_speed) {
    speed = min_speed;
  }

  if (con_wall.enable == true) {
    con_wall.p_error = con_wall.error;
    if ((sen_r.is_control == true) && (sen_l.is_control == true)) {
      con_wall.error = sen_r.error - sen_l.error;
    } else {
      con_wall.error = 2.0 * (sen_r.error - sen_l.error);
    }
    con_wall.diff = con_wall.error - con_wall.p_error;
    con_wall.sum += con_wall.error;
    if (con_wall.sum > con_wall.sum_max) {
      con_wall.sum = con_wall.sum_max;
    } else if (con_wall.sum < (-con_wall.sum_max)) {
      con_wall.sum = -con_wall.sum_max;
    }
    con_wall.control = 0.001 * speed * con_wall.kp * con_wall.error;
    spd_r = speed + con_wall.control;
    spd_l = speed - con_wall.control;
  } else {
    spd_r = speed;
    spd_l = speed;
  }

  //odom
  //xは方向
  double speed2 = (spd_r * RMotorSinged() + spd_l * LMotorSinged()) / 2.0;
  omega = (spd_r * RMotorSinged() - spd_l * LMotorSinged()) / TREAD_WIDTH * 1.00;
  odom_x += speed2 * 0.001 * cos(odom_theta) * 0.001;
  odom_y += speed2 * 0.001 * sin(odom_theta) * 0.001;
  odom_theta += omega * 0.001;

  //角度、位置のズレを修正
  if (
    ((sen_r.value < (sen_r.ref + 20)) || (sen_l.value < (sen_l.ref + 20))) &&
    ((sen_r.value > (sen_r.ref - 20)) || (sen_l.value > (sen_l.ref - 20)))) {
    temp_cnt++;
    if (temp_cnt > 5) {
      switch (map_control.get_mypos_dir()) {
        case north:
          if (theta_adj == true) {
            odom_theta = 0.0;
          }
          break;
        case east:
          if (theta_adj == true) {
            odom_theta = -1.57;
          }
          break;
        case south:
          if (theta_adj == true) {
            odom_theta = 3.14;
          }
          break;
        case west:
          if (theta_adj == true) {
            odom_theta = 1.57;
          }
          break;
      }
      temp_cnt = 0;
    }
  } else {
    temp_cnt = 0;
  }

  position_r += spd_r * 0.001 * RMotorSinged() / (TIRE_DIAMETER * PI) * 2 * PI;
  position_l -= spd_l * 0.001 * LMotorSinged() / (TIRE_DIAMETER * PI) * 2 * PI;

  SetRStepHz((unsigned short)(spd_r / PULSE));
  SetLStepHz((unsigned short)(spd_l / PULSE));
}

void sensor_interrupt(void)
{
  static char cnt = 0;
  short tmp1, tmp2;
  static char bled_cnt = 0;

  switch (cnt) {
    case 0:
      sen_r.p2_value = sen_r.p_value;
      sen_l.p2_value = sen_l.p_value;
      sen_r.p_value = sen_r.value;
      sen_l.p_value = sen_l.value;
      sen_r.value = GetSensorR();
      sen_l.value = GetSensorL();

      if ((sen_r.value / 4 + sen_r.p_value / 2 + sen_r.p2_value / 4) > sen_r.th_wall) {
        sen_r.is_wall = true;
      } else {
        sen_r.is_wall = false;
      }
      if ((sen_l.ref / 4 + sen_l.p_value / 2 + sen_l.p2_value / 4) > sen_l.th_wall) {
        sen_l.is_wall = true;
      } else {
        sen_l.is_wall = false;
      }

      if (sen_r.value > sen_r.th_control) {
        sen_r.error = sen_r.value / 4 + sen_r.p_value / 2 + sen_r.p2_value / 4 - sen_r.ref;
        sen_r.is_control = true;
      } else {
        sen_r.error = 0;
        sen_r.is_control = false;
      }
      if (sen_l.value > sen_l.th_control) {
        sen_l.error = sen_l.value - (sen_l.ref / 4 + sen_l.p_value / 2 + sen_l.p2_value / 4);
        sen_l.is_control = true;
      } else {
        sen_l.error = 0;
        sen_l.is_control = false;
      }
      break;
    case 1:
      bled_cnt++;
      if (bled_cnt > 10) {
        bled_cnt = 0;
      }
      battery_value = GetBatteryVolt();
      if (((battery_value - BATT_MIN) * 10 / (BATT_MAX - BATT_MIN)) > bled_cnt) {
        SetBLED(1);
      } else {
        SetBLED(2);
      }
      sen_fr.p2_value = sen_fr.p_value;
      sen_fl.p2_value = sen_fl.p_value;
      sen_fr.p_value = sen_fr.value;
      sen_fl.p_value = sen_fl.value;
      sen_fr.value = GetSensorFR();
      sen_fl.value = GetSensorFL();
      if ((sen_fr.value / 4 + sen_fr.p_value / 2 + sen_fr.p2_value / 2) > sen_fr.th_wall) {
        sen_fr.is_wall = true;
      } else {
        sen_fr.is_wall = false;
      }
      if ((sen_fl.value / 4 + sen_fl.p_value / 2 + sen_fl.p2_value / 2) > sen_fl.th_wall) {
        sen_fl.is_wall = true;
      } else {
        sen_fl.is_wall = false;
      }
      break;
  }
  cnt++;
  if (cnt == 2) cnt = 0;
}
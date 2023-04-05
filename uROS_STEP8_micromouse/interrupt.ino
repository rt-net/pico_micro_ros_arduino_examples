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

void control_interrupt(void) {
  double spd_r, spd_l;

  speed += r_accel;

  if (speed > max_speed) {
    speed = max_speed;
  }
  if (speed < min_speed) {
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
  if (spd_r < MIN_SPEED) spd_r = MIN_SPEED;
  if (spd_l < MIN_SPEED) spd_l = MIN_SPEED;

  SetRStepHz((unsigned short)(spd_r / PULSE));
  SetLStepHz((unsigned short)(spd_l / PULSE));
}

void sensor_interrupt(void) {
  static char cnt = 0;
  static char bled_cnt = 0;


  switch (cnt) {
    case 0:
      sen_fr.p_value = sen_fr.value;
      sen_fr.value = GetSensorFR();
      if (sen_fr.value > sen_fr.th_wall) {
        sen_fr.is_wall = true;
      } else {
        sen_fr.is_wall = false;
      }
      break;
    case 1:
      sen_fl.p_value = sen_fl.value;
      sen_fl.value = GetSensorFL();
      if (sen_fl.value > sen_fl.th_wall) {
        sen_fl.is_wall = true;
      } else {
        sen_fl.is_wall = false;
      }
      break;
    case 2:
      sen_r.p_value = sen_r.value;
      sen_r.value = GetSensorR();
      if (sen_r.value > sen_r.th_wall) {
        sen_r.is_wall = true;
      } else {
        sen_r.is_wall = false;
      }
      if (sen_r.value > sen_r.th_control) {
        sen_r.error = sen_r.value - sen_r.ref;
        sen_r.is_control = true;
      } else {
        sen_r.error = 0;
        sen_r.is_control = false;
      }
      break;
    case 3:
      sen_l.p_value = sen_l.value;
      sen_l.value = GetSensorL();
      if (sen_l.value > sen_l.th_wall) {
        sen_l.is_wall = true;
      } else {
        sen_l.is_wall = false;
      }
      if (sen_l.value > sen_l.th_control) {
        sen_l.error = sen_l.value - sen_l.ref;
        sen_l.is_control = true;
      } else {
        sen_l.error = 0;
        sen_l.is_control = false;
      }
      bled_cnt++;
      if (bled_cnt > 10) {
        bled_cnt = 0;
      }
      battery_value = GetBatteryVolt();
      if ( ((battery_value - BATT_MIN) * 10 / (BATT_MAX - BATT_MIN)) > bled_cnt) {
        SetBLED(1);
      } else {
        SetBLED(2);
      }
      break;
  }
  cnt++;
  if (cnt == 4) cnt = 0;
}
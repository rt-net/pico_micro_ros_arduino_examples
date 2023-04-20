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

void controlInterrupt(void)
{
  double spd_r, spd_l;

  g_speed += g_accel;

  if (g_speed > g_max_speed) {
    g_speed = g_max_speed;
  }
  if (g_speed < g_min_speed) {
    g_speed = g_min_speed;
  }

  if (g_con_wall.enable == true) {
    g_con_wall.p_error = g_con_wall.error;
    if ((g_sen_r.is_control == true) && (g_sen_l.is_control == true)) {
      g_con_wall.error = g_sen_r.error - g_sen_l.error;
    } else {
      g_con_wall.error = 2.0 * (g_sen_r.error - g_sen_l.error);
    }
    g_con_wall.diff = g_con_wall.error - g_con_wall.p_error;
    g_con_wall.sum += g_con_wall.error;
    if (g_con_wall.sum > g_con_wall.sum_max) {
      g_con_wall.sum = g_con_wall.sum_max;
    } else if (g_con_wall.sum < (-g_con_wall.sum_max)) {
      g_con_wall.sum = -g_con_wall.sum_max;
    }
    g_con_wall.control = 0.001 * g_speed * g_con_wall.kp * g_con_wall.error;
    spd_r = g_speed + g_con_wall.control;
    spd_l = g_speed - g_con_wall.control;
  } else {
    spd_r = g_speed;
    spd_l = g_speed;
  }
  if (spd_r < MIN_SPEED) spd_r = MIN_SPEED;
  if (spd_l < MIN_SPEED) spd_l = MIN_SPEED;

  setRStepHz((unsigned short)(spd_r / PULSE));
  setLStepHz((unsigned short)(spd_l / PULSE));
}

void sensorInterrupt(void)
{
  static char cnt = 0;
  static char bled_cnt = 0;

  switch (cnt) {
    case 0:
      g_sen_fr.p_value = g_sen_fr.value;
      g_sen_fr.value = getSensorFR();
      if (g_sen_fr.value > g_sen_fr.th_wall) {
        g_sen_fr.is_wall = true;
      } else {
        g_sen_fr.is_wall = false;
      }
      break;
    case 1:
      g_sen_fl.p_value = g_sen_fl.value;
      g_sen_fl.value = getSensorFL();
      if (g_sen_fl.value > g_sen_fl.th_wall) {
        g_sen_fl.is_wall = true;
      } else {
        g_sen_fl.is_wall = false;
      }
      break;
    case 2:
      g_sen_r.p_value = g_sen_r.value;
      g_sen_r.value = getSensorR();
      if (g_sen_r.value > g_sen_r.th_wall) {
        g_sen_r.is_wall = true;
      } else {
        g_sen_r.is_wall = false;
      }
      if (g_sen_r.value > g_sen_r.th_control) {
        g_sen_r.error = g_sen_r.value - g_sen_r.ref;
        g_sen_r.is_control = true;
      } else {
        g_sen_r.error = 0;
        g_sen_r.is_control = false;
      }
      break;
    case 3:
      g_sen_l.p_value = g_sen_l.value;
      g_sen_l.value = getSensorL();
      if (g_sen_l.value > g_sen_l.th_wall) {
        g_sen_l.is_wall = true;
      } else {
        g_sen_l.is_wall = false;
      }
      if (g_sen_l.value > g_sen_l.th_control) {
        g_sen_l.error = g_sen_l.value - g_sen_l.ref;
        g_sen_l.is_control = true;
      } else {
        g_sen_l.error = 0;
        g_sen_l.is_control = false;
      }
      bled_cnt++;
      if (bled_cnt > 10) {
        bled_cnt = 0;
      }
      g_battery_value = getBatteryVolt();
      if (((g_battery_value - BATT_MIN) * 10 / (BATT_MAX - BATT_MIN)) > bled_cnt) {
        setBLED(1);
      } else {
        setBLED(2);
      }
      break;
    default:
      Serial.printf("sensor state error\n\r");
      break;
  }
  cnt++;
  if (cnt == 4) cnt = 0;
}
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

void controlInterrupt(void)
{
  double spd_r, spd_l, g_omega;
  static char temp_cnt = 0;

  g_speed += g_accel;

  if (g_speed > g_max_speed) {
    g_speed = g_max_speed;
  }

  if (g_motor_move == 0) {
    g_speed = 0.0;
  } else if (g_speed < g_min_speed) {
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

  //odom
  //xは方向
  double speed2 = (spd_r * motorSignedR() + spd_l * motorSignedL()) / 2.0;
  g_omega = (spd_r * motorSignedR() - spd_l * motorSignedL()) / TREAD_WIDTH * 1.00;
  g_odom_x += speed2 * 0.001 * cos(g_odom_theta) * 0.001;
  g_odom_y += speed2 * 0.001 * sin(g_odom_theta) * 0.001;
  g_odom_theta += g_omega * 0.001;

  //角度、位置のズレを修正
  if (
    ((g_sen_r.value < (g_sen_r.ref + 20)) || (g_sen_l.value < (g_sen_l.ref + 20))) &&
    ((g_sen_r.value > (g_sen_r.ref - 20)) || (g_sen_l.value > (g_sen_l.ref - 20)))) {
    temp_cnt++;
    if (temp_cnt > 5) {
      switch (g_map_control.getMyPosDir()) {
        case north:
          if (g_theta_adj == true) {
            g_odom_theta = 0.0;
          }
          break;
        case east:
          if (g_theta_adj == true) {
            g_odom_theta = -1.57;
          }
          break;
        case south:
          if (g_theta_adj == true) {
            g_odom_theta = 3.14;
          }
          break;
        case west:
          if (g_theta_adj == true) {
            g_odom_theta = 1.57;
          }
          break;
      }
      temp_cnt = 0;
    }
  } else {
    temp_cnt = 0;
  }

  g_position_r += spd_r * 0.001 * motorSignedR() / (TIRE_DIAMETER * PI) * 2 * PI;
  g_position_l -= spd_l * 0.001 * motorSignedL() / (TIRE_DIAMETER * PI) * 2 * PI;

  setRStepHz((unsigned short)(spd_r / PULSE));
  setLStepHz((unsigned short)(spd_l / PULSE));
}

void sensorInterrupt(void)
{
  static char cnt = 0;
  static char bled_cnt = 0;

  switch (cnt) {
    case 0:
      g_sen_r.p2_value = g_sen_r.p_value;
      g_sen_l.p2_value = g_sen_l.p_value;
      g_sen_r.p_value = g_sen_r.value;
      g_sen_l.p_value = g_sen_l.value;
      g_sen_r.value = getSensorR();
      g_sen_l.value = getSensorL();

      if ((g_sen_r.value / 4 + g_sen_r.p_value / 2 + g_sen_r.p2_value / 4) > g_sen_r.th_wall) {
        g_sen_r.is_wall = true;
      } else {
        g_sen_r.is_wall = false;
      }
      if ((g_sen_l.ref / 4 + g_sen_l.p_value / 2 + g_sen_l.p2_value / 4) > g_sen_l.th_wall) {
        g_sen_l.is_wall = true;
      } else {
        g_sen_l.is_wall = false;
      }

      if (g_sen_r.value > g_sen_r.th_control) {
        g_sen_r.error =
          g_sen_r.value / 4 + g_sen_r.p_value / 2 + g_sen_r.p2_value / 4 - g_sen_r.ref;
        g_sen_r.is_control = true;
      } else {
        g_sen_r.error = 0;
        g_sen_r.is_control = false;
      }
      if (g_sen_l.value > g_sen_l.th_control) {
        g_sen_l.error =
          g_sen_l.value - (g_sen_l.ref / 4 + g_sen_l.p_value / 2 + g_sen_l.p2_value / 4);
        g_sen_l.is_control = true;
      } else {
        g_sen_l.error = 0;
        g_sen_l.is_control = false;
      }
      break;
    case 1:
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
      g_sen_fr.p2_value = g_sen_fr.p_value;
      g_sen_fl.p2_value = g_sen_fl.p_value;
      g_sen_fr.p_value = g_sen_fr.value;
      g_sen_fl.p_value = g_sen_fl.value;
      g_sen_fr.value = getSensorFR();
      g_sen_fl.value = getSensorFL();
      if ((g_sen_fr.value / 4 + g_sen_fr.p_value / 2 + g_sen_fr.p2_value / 2) > g_sen_fr.th_wall) {
        g_sen_fr.is_wall = true;
      } else {
        g_sen_fr.is_wall = false;
      }
      if ((g_sen_fl.value / 4 + g_sen_fl.p_value / 2 + g_sen_fl.p2_value / 2) > g_sen_fl.th_wall) {
        g_sen_fl.is_wall = true;
      } else {
        g_sen_fl.is_wall = false;
      }
      break;
  }
  cnt++;
  if (cnt == 2) cnt = 0;
}
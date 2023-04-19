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

  speed += r_accel;

  if (speed > max_speed) {
    speed = max_speed;
  }
  if (speed < min_speed) {
    speed = min_speed;
  }

  if ((sen_r.is_control == true) && (sen_l.is_control == true)) {
    con_wall.error = sen_r.error - sen_l.error;
  } else {
    con_wall.error = 2.0 * (sen_r.error - sen_l.error);
  }

  con_wall.control = 0.001 * speed * con_wall.kp * con_wall.error;

  spd_r = speed + con_wall.control;
  spd_l = speed - con_wall.control;

  if (spd_r < min_speed) {
    spd_r = min_speed;
  }
  if (spd_l < min_speed) {
    spd_l = min_speed;
  }

  r_step_hz = (unsigned short)(spd_r / PULSE);
  l_step_hz = (unsigned short)(spd_l / PULSE);
}

void sensorInterrupt(void)
{
  static char cnt = 0;
  static char bled_cnt = 0;

  switch (cnt) {
    case 0:
      digitalWrite(SLED_FR, HIGH);
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sen_fr.value = analogRead(AD1);
      digitalWrite(SLED_FR, LOW);
      if (sen_fr.value > sen_fr.th_wall) {
        sen_fr.is_wall = true;
      } else {
        sen_fr.is_wall = false;
      }
      break;
    case 1:
      digitalWrite(SLED_FL, HIGH);
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sen_fl.value = analogRead(AD2);
      digitalWrite(SLED_FL, LOW);
      if (sen_fl.value > sen_fl.th_wall) {
        sen_fl.is_wall = true;
      } else {
        sen_fl.is_wall = false;
      }
      break;
    case 2:
      digitalWrite(SLED_R, HIGH);
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sen_r.value = analogRead(AD3);
      digitalWrite(SLED_R, LOW);
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
      digitalWrite(SLED_L, HIGH);
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sen_l.value = analogRead(AD4);
      digitalWrite(SLED_L, LOW);
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
      battery_value = (double)analogReadMilliVolts(AD0) / 10.0 * (10.0 + 51.0);
      break;
  }
  cnt++;
  if (cnt == 4) cnt = 0;
}
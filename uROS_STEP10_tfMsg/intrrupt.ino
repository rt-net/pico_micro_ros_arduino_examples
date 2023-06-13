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
  double speed_r, speed_l;

  if ((g_speed < 0.001) && (g_speed > -0.001) && (g_omega < 0.001) && (g_omega > -0.001)) {
    g_motor_move = 0;
  } else {
    g_motor_move = 1;
  }

  speed_r = g_speed + g_omega * TREAD_WIDTH / 2.0;
  speed_l = g_speed - g_omega * TREAD_WIDTH / 2.0;

  if ((speed_r > 0.001) && (speed_r < MIN_SPEED)) {
    speed_r = MIN_SPEED;
  } else if ((speed_r < -0.001) && (speed_r > (-1.0 * MIN_SPEED))) {
    speed_r = -1.0 * MIN_SPEED;
  }
  if ((speed_l > 0.001) && (speed_l < MIN_SPEED)) {
    speed_l = MIN_SPEED;
  } else if ((speed_l < -0.001) && (speed_l > (-1.0 * MIN_SPEED))) {
    speed_l = -1.0 * MIN_SPEED;
  }
  g_speed = (speed_r + speed_l) / 2.0;

  g_odom_x += g_speed * 0.001 * cos(g_odom_theta) * 0.001;
  g_odom_y += g_speed * 0.001 * sin(g_odom_theta) * 0.001;
  g_odom_theta += g_omega * 0.001;
  g_position_r += speed_r * 0.001 / (48 * PI) * 2 * PI;
  g_position_l -= speed_l * 0.001 / (48 * PI) * 2 * PI;

  if (speed_r > 0) {
    digitalWrite(CW_R, LOW);
  } else {
    digitalWrite(CW_R, HIGH);
  }
  g_step_hz_r = abs((signed short)(speed_r / PULSE));

  if (speed_l > 0) {
    digitalWrite(CW_L, LOW);
  } else {
    digitalWrite(CW_L, HIGH);
  }
  g_step_hz_l = abs((signed short)(speed_l / PULSE));
}

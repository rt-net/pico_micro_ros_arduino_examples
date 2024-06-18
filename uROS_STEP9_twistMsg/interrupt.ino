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

  if (speed_r > 0) {
    digitalWrite(CW_R, LOW);
  } else {
    digitalWrite(CW_R, HIGH);
    speed_r *= -1.0;
  }
  if (speed_r < 0.001) {
    speed_r = 0.0;
  } else if (speed_r < MIN_SPEED) {
    speed_r = MIN_SPEED;
  }
  g_step_hz_r = (signed short)(speed_r / PULSE);

  if (speed_l > 0) {
    digitalWrite(CW_L, LOW);
  } else {
    digitalWrite(CW_L, HIGH);
    speed_l *= -1.0;
  }
  if (speed_l < MIN_SPEED) speed_l = MIN_SPEED;
  g_step_hz_l = (signed short)(speed_l / PULSE);
}
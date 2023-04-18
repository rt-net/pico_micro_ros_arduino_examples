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

void control_interrupt(void)
{
  double speed_r, speed_l;

  if ((speed < 0.001) && (speed > -0.001) && (omega < 0.001) && (omega > -0.001)) {
    motor_move = 0;
  } else {
    motor_move = 1;
  }

  speed_r = speed + omega * TREAD_WIDTH / 2.0;
  speed_l = speed - omega * TREAD_WIDTH / 2.0;

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
  speed = (speed_r + speed_l) / 2.0;

  odom_x += speed * 0.001 * cos(odom_theta) * 0.001;
  odom_y += speed * 0.001 * sin(odom_theta) * 0.001;
  odom_theta += omega * 0.001;
  position_r += speed_r * 0.001 / (48 * PI) * 2 * PI;
  position_l -= speed_l * 0.001 / (48 * PI) * 2 * PI;

  if (speed_r > 0) {
    digitalWrite(CW_R, LOW);
  } else {
    digitalWrite(CW_R, HIGH);
  }
  R_STEP_HZ = abs((signed short)(speed_r / PULSE));

  if (speed_l > 0) {
    digitalWrite(CW_L, LOW);
  } else {
    digitalWrite(CW_L, HIGH);
  }
  L_STEP_HZ = abs((signed short)(speed_l / PULSE));
}

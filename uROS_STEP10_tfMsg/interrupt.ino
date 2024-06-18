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
  // 直進速度と回転速度から、左右のモータ速度を求める
  double speed_r = g_speed + g_omega * TREAD_WIDTH / 2.0;
  double speed_l = g_speed - g_omega * TREAD_WIDTH / 2.0;

  // 左右両方のモータ速度がMIN_SPEED以下の場合は走行を停止する
  if (fabs(speed_r) < MIN_SPEED && fabs(speed_l) < MIN_SPEED) {
    g_motor_move = 0;
    return;
  }

  g_motor_move = 1;
  // モータ速度をMIN_SPEED以上に制限する
  if (fabs(speed_r) < MIN_SPEED) {
    speed_r = (speed_r > 0.0) ? MIN_SPEED : -1.0 * MIN_SPEED;
  }
  if (fabs(speed_l) < MIN_SPEED) {
    speed_l = (speed_l > 0.0) ? MIN_SPEED : -1.0 * MIN_SPEED;
  }

  // 制限されたモータ速度から、直進速度と回転速度を求める
  const double forward_speed = (speed_r + speed_l) / 2.0;
  const double omega = (speed_r - speed_l) / TREAD_WIDTH;

  const double UPDATE_INTERVAL = 0.001;
  g_odom_x += forward_speed * UPDATE_INTERVAL * cos(g_odom_theta) * UPDATE_INTERVAL;
  g_odom_y += forward_speed * UPDATE_INTERVAL * sin(g_odom_theta) * UPDATE_INTERVAL;
  g_odom_theta += omega * UPDATE_INTERVAL;
  g_position_r += speed_r * UPDATE_INTERVAL / (TIRE_DIAMETER * PI) * 2 * PI;
  g_position_l -= speed_l * UPDATE_INTERVAL / (TIRE_DIAMETER * PI) * 2 * PI;

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

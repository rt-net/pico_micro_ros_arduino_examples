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

void accelerate(int len, int tar_speed)
{
  int obj_step;
  max_speed = tar_speed;
  r_accel = 1.5;
  step_r = step_l = 0;
  speed = min_speed = MIN_SPEED;
  r_step_hz = l_step_hz = (unsigned short)(speed / PULSE);

  obj_step = (int)((float)len * 2.0 / PULSE);
  digitalWrite(CW_R, LOW);
  digitalWrite(CW_L, LOW);
  motor_move = 1;

  while ((step_r + step_l) < obj_step)
    ;
}

void oneStep(int len, int tar_speed)
{
  int obj_step;
  max_speed = tar_speed;
  r_accel = 0.0;
  step_r = step_l = 0;
  speed = min_speed = tar_speed;
  r_step_hz = l_step_hz = (unsigned short)(speed / PULSE);
  obj_step = (int)((float)len * 2.0 / PULSE);
  digitalWrite(CW_R, LOW);
  digitalWrite(CW_L, LOW);

  while ((step_r + step_l) < obj_step)
    ;
}

void decelerate(int len, int tar_speed)
{
  int obj_step;
  max_speed = tar_speed;
  r_accel = 0.0;
  step_r = step_l = 0;
  speed = min_speed = tar_speed;
  r_step_hz = l_step_hz = (unsigned short)(speed / PULSE);
  obj_step = (int)((float)len * 2.0 / PULSE);
  digitalWrite(CW_R, LOW);
  digitalWrite(CW_L, LOW);

  while ((len - (step_r + step_l) / 2.0 * PULSE) >
         (((speed * speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * 1.5)))
    ;
  r_accel = -1.5;
  min_speed = MIN_SPEED;

  while ((step_r + step_l) < obj_step)
    ;

  motor_move = 0;
}
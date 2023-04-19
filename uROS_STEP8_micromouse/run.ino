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

void straight(int len, int init_speed, int max_sp, int finish_speed)
{
  int obj_step;

  controlInterruptStop();
  max_speed = max_sp;
  r_accel = SEARCH_ACCEL;

  if (init_speed < MIN_SPEED) {
    speed = MIN_SPEED;
    clearStepR();
    clearStepL();
  } else {
    speed = init_speed;
  }
  if (finish_speed < MIN_SPEED) {
    finish_speed = MIN_SPEED;
  }
  if (init_speed < finish_speed) {
    min_speed = MIN_SPEED;
  } else {
    min_speed = finish_speed;
  }
  setRStepHz((unsigned short)(speed / PULSE));
  setLStepHz((unsigned short)(speed / PULSE));

  con_wall.enable = true;
  obj_step = (int)((float)len * 2.0 / PULSE);
  moveDir(MOT_FORWARD, MOT_FORWARD);  //left,right
  controlInterruptStart();

  motor_move = 1;
  while ((len - (getStepR() + getStepL()) / 2.0 * PULSE) >
         (((speed * speed) - (finish_speed * finish_speed)) / (2.0 * 1000.0 * SEARCH_ACCEL))) {
    continue;
  }
  r_accel = -1.0 * SEARCH_ACCEL;

  while ((getStepR() + getStepL()) < obj_step) {
    continue;
  }

  if (finish_speed == SEARCH_SPEED) {
    controlInterruptStop();
    max_speed = min_speed = speed = finish_speed;
    r_accel = 0.0;
    clearStepR();
    clearStepL();
    controlInterruptStart();
  } else {
    motor_move = 0;
  }
}

void accelerate(int len, int finish_speed)
{
  int obj_step;

  controlInterruptStop();
  max_speed = finish_speed;
  r_accel = SEARCH_ACCEL;
  speed = min_speed = MIN_SPEED;
  setRStepHz((unsigned short)(speed / PULSE));
  setLStepHz((unsigned short)(speed / PULSE));
  clearStepR();
  clearStepL();
  con_wall.enable = true;
  obj_step = (int)((float)len * 2.0 / PULSE);
  moveDir(MOT_FORWARD, MOT_FORWARD);
  controlInterruptStart();

  motor_move = 1;

  while ((getStepR() + getStepL()) < obj_step) {
    continue;
  }

  controlInterruptStop();
  max_speed = min_speed = speed = finish_speed;
  r_accel = 0.0;
  clearStepR();
  clearStepL();
  controlInterruptStart();
}

void oneStep(int len, int tar_speed)
{
  int obj_step;
  controlInterruptStop();
  speed = min_speed = max_speed = tar_speed;
  r_accel = 0.0;
  setRStepHz((unsigned short)(speed / PULSE));
  setLStepHz((unsigned short)(speed / PULSE));
  con_wall.enable = true;
  obj_step = (int)((float)len * 2.0 / PULSE);
  moveDir(MOT_FORWARD, MOT_FORWARD);
  controlInterruptStart();

  while ((getStepR() + getStepL()) < obj_step) {
    continue;
  }
  controlInterruptStop();
  max_speed = min_speed = speed = tar_speed;
  r_accel = 0.0;
  clearStepR();
  clearStepL();
  controlInterruptStart();
}

void decelerate(int len, int tar_speed)
{
  int obj_step;
  controlInterruptStop();
  max_speed = tar_speed;
  r_accel = 0.0;
  speed = min_speed = tar_speed;

  setRStepHz((unsigned short)(speed / PULSE));
  setLStepHz((unsigned short)(speed / PULSE));
  con_wall.enable = true;
  obj_step = (int)((float)len * 2.0 / PULSE);
  moveDir(MOT_FORWARD, MOT_FORWARD);
  controlInterruptStart();

  while ((len - (getStepR() + getStepL()) / 2.0 * PULSE) >
         (((speed * speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * SEARCH_ACCEL))) {
    continue;
  }
  r_accel = -1.0 * SEARCH_ACCEL;
  min_speed = MIN_SPEED;

  while ((getStepR() + getStepL()) < obj_step) {
    continue;
  }

  motor_move = 0;

  delay(300);
}

void rotate(t_direction dir, int times)
{
  int obj_step;
  controlInterruptStop();
  max_speed = SEARCH_SPEED;
  r_accel = TURN_ACCEL;
  speed = min_speed = MIN_SPEED;
  setRStepHz((unsigned short)(speed / PULSE));
  setLStepHz((unsigned short)(speed / PULSE));
  clearStepR();
  clearStepL();
  con_wall.enable = false;
  obj_step = (int)(TREAD_WIDTH * PI / 4.0 * (float)times * 2.0 / PULSE);

  switch (dir) {
    case right:
      moveDir(MOT_FORWARD, MOT_BACK);
      motor_move = 1;
      break;
    case left:
      moveDir(MOT_BACK, MOT_FORWARD);
      motor_move = 1;
      break;
    default:
      motor_move = 0;
      break;
  }
  controlInterruptStart();

  while (((obj_step - (getStepR() + getStepL())) / 2.0 * PULSE) >
         (((speed * speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * TURN_ACCEL))) {
    continue;
  }
  r_accel = -1.0 * TURN_ACCEL;

  while ((getStepR() + getStepL()) < obj_step) {
    continue;
  }

  motor_move = 0;
  delay(300);
}

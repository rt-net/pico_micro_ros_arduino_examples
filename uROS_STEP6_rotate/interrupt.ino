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

void control_interrupt(void){

  speed += r_accel;

  if(speed > max_speed){
    speed = max_speed;
  }
  if(speed < min_speed){
    speed = min_speed;
  }

  LStepHz = RStepHz =   (unsigned short)(speed/PULSE);

}
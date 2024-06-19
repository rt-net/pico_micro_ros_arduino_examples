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
  g_speed += g_accel;

  if (g_speed > g_max_speed) {
    g_speed = g_max_speed;
  }
  if (g_speed < g_min_speed) {
    g_speed = g_min_speed;
  }

  g_step_hz_l = g_step_hz_r = (unsigned short)(g_speed / PULSE);
}
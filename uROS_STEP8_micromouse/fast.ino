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

void fastRun(short gx, short gy)
{
  t_direction_glob glob_nextdir;
  int straight_count = 0;

  t_direction temp_next_dir = g_map_control.getNextDir2(gx, gy, &glob_nextdir);
  switch (temp_next_dir) {
    case right:
      rotate(right, 1);  //右に曲がって
      break;
    case left:
      rotate(left, 1);  //左に曲がって
      break;
    case rear:
      rotate(right, 2);  //180度に旋回して
      break;
    default:
      break;
  }

  accelerate(HALF_SECTION, SEARCH_SPEED);
  straight_count = 0;
  g_map_control.setMyPosDir(glob_nextdir);
  g_map_control.axisUpdate();

  while ((g_map_control.getMyPosX() != gx) || (g_map_control.getMyPosY() != gy)) {
    switch (g_map_control.getNextDir2(gx, gy, &glob_nextdir)) {
      case front:
        straight_count++;
        break;
      case right:
        straight(straight_count * SECTION, SEARCH_SPEED, MAX_SPEED, SEARCH_SPEED);
        straight_count = 0;
        decelerate(HALF_SECTION, SEARCH_SPEED);
        rotate(right, 1);
        accelerate(HALF_SECTION, SEARCH_SPEED);
        break;
      case left:
        straight(straight_count * SECTION, SEARCH_SPEED, MAX_SPEED, SEARCH_SPEED);
        straight_count = 0;
        decelerate(HALF_SECTION, SEARCH_SPEED);
        rotate(left, 1);
        accelerate(HALF_SECTION, SEARCH_SPEED);
        break;
      default:
        break;
    }
    g_map_control.setMyPosDir(glob_nextdir);
    g_map_control.axisUpdate();
  }
  if (straight_count > 0) {
    straight(straight_count * SECTION, SEARCH_SPEED, MAX_SPEED, SEARCH_SPEED);
  }
  decelerate(HALF_SECTION, SEARCH_SPEED);
}

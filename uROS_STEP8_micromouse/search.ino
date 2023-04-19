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

void searchLefthand(void)
{
  accelerate(HALF_SECTION, SEARCH_SPEED);

  while (1) {
    if (g_sen_l.is_wall == false) {
      decelerate(HALF_SECTION, SEARCH_SPEED);
      rotate(left, 1);
      accelerate(HALF_SECTION, SEARCH_SPEED);
    } else if ((g_sen_fl.is_wall == false) && (g_sen_fr.is_wall == false)) {
      straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_SPEED);
    } else if (g_sen_r.is_wall == false) {
      decelerate(HALF_SECTION, SEARCH_SPEED);
      rotate(right, 1);
      accelerate(HALF_SECTION, SEARCH_SPEED);
    } else {
      decelerate(HALF_SECTION, SEARCH_SPEED);
      rotate(right, 2);
      accelerate(HALF_SECTION, SEARCH_SPEED);
    }
  }
}

void searchAdachi(char gx, char gy)
{
  t_direction_glob glob_nextdir;
  t_direction temp_next_dir;

  temp_next_dir = g_map_control.getNextDir(gx, gy, &glob_nextdir);

  switch (temp_next_dir) {
    case front:
      break;
    case right:
      rotate(right, 1);
      break;
    case left:
      rotate(left, 1);
      break;
    case rear:
      rotate(right, 2);
      break;
  }

  accelerate(HALF_SECTION, SEARCH_SPEED);

  g_map_control.setMyPosDir(glob_nextdir);
  g_map_control.axisUpdate();

  while ((g_map_control.getMyPosX() != gx) || (g_map_control.getMyPosY() != gy)) {
    g_map_control.setWall(g_sen_fr.is_wall, g_sen_r.is_wall, g_sen_l.is_wall);

    switch (g_map_control.getNextDir(gx, gy, &glob_nextdir)) {
      case front:
        oneStep(SECTION, SEARCH_SPEED);
        break;
      case right:
        decelerate(HALF_SECTION, SEARCH_SPEED);
        rotate(right, 1);
        accelerate(HALF_SECTION, SEARCH_SPEED);
        break;
      case left:
        decelerate(HALF_SECTION, SEARCH_SPEED);
        rotate(left, 1);
        accelerate(HALF_SECTION, SEARCH_SPEED);
        break;
      case rear:
        decelerate(HALF_SECTION, SEARCH_SPEED);
        rotate(right, 2);
        accelerate(HALF_SECTION, SEARCH_SPEED);
        break;
    }

    g_map_control.setMyPosDir(glob_nextdir);  //方向を更新
    g_map_control.axisUpdate();
  }

  g_map_control.setWall(g_sen_fr.is_wall, g_sen_r.is_wall, g_sen_l.is_wall);
  decelerate(HALF_SECTION, SEARCH_SPEED);
}
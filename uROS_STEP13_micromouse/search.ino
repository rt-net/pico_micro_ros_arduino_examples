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
    if (sen_l.is_wall == false) {
      decelerate(HALF_SECTION, SEARCH_SPEED);
      rotate(left, 1);
      accelerate(HALF_SECTION, SEARCH_SPEED);
    } else if ((sen_fl.is_wall == false) && (sen_fr.is_wall == false)) {
      straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_SPEED);
    } else if (sen_r.is_wall == false) {
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
  int straight_count = 0;
  t_direction temp_next_dir;

  temp_next_dir = map_control.getNextDir(gx, gy, &glob_nextdir);

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

  map_control.setMyPosDir(glob_nextdir);
  map_control.axisUpdate();

  while ((map_control.getMyPosX() != gx) || (map_control.getMyPosY() != gy)) {
    map_control.setWall(sen_fl.is_wall, sen_r.is_wall, sen_l.is_wall);
    publish_x = map_control.getMyPosX();
    publish_y = map_control.getMyPosY();

    switch (map_control.getNextDir(gx, gy, &glob_nextdir)) {
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

    map_control.setMyPosDir(glob_nextdir);  //方向を更新
    map_control.axisUpdate();
  }

  map_control.setWall(sen_fl.is_wall, sen_r.is_wall, sen_l.is_wall);
  publish_x = map_control.getMyPosX();
  publish_y = map_control.getMyPosY();
  decelerate(HALF_SECTION, SEARCH_SPEED);
}

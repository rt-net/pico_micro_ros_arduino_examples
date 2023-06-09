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

unsigned char g_second_run[256];

void fastRun(short gx, short gy)
{
  t_direction_glob glob_nextdir;
  int straight_count = 0;
  int i = 0;

  //rvizに表示するマーカーの座標の初期化
  g_start_x = g_map_control.getMyPosX();
  g_start_y = g_map_control.getMyPosY();
  g_start_dir = g_map_control.getMyPosDir();

  //マーカー用のデータを作成
  t_direction temp_next_dir = g_map_control.getNextDir2(gx, gy, &glob_nextdir);
  g_map_control.setMyPosDir(glob_nextdir);
  g_map_control.axisUpdate();
  while ((g_map_control.getMyPosX() != gx) || (g_map_control.getMyPosY() != gy)) {
    switch (g_map_control.getNextDir2(gx, gy, &glob_nextdir)) {
      case front:
        straight_count++;
        break;
      case right:
        g_second_run[i++] = straight_count;
        g_second_run[i++] = R90;
        straight_count = 0;
        break;
      case left:
        g_second_run[i++] = straight_count;
        g_second_run[i++] = L90;
        straight_count = 0;
        break;
      default:
        break;
    }
    g_map_control.setMyPosDir(glob_nextdir);
    g_map_control.axisUpdate();
  }

  g_second_run[i++] = straight_count;
  g_second_run[i++] = 127;

  g_map_control.setMyPosDir(g_start_dir);

  //second_runにあるデータに沿って走行する。
  switch (temp_next_dir) {
    case right:
      rotate(right, 1);  //右に曲がって
      g_map_control.nextDir(right);
      break;
    case left:
      rotate(left, 1);  //左に曲がって
      g_map_control.nextDir(left);
      break;
    case rear:
      rotate(right, 2);  //180度に旋回して
      g_map_control.nextDir(right);
      g_map_control.nextDir(right);
      break;
    default:
      break;
  }

  g_fast_task = true;  //rvizに最短経路のマーカーを表示
  delay(10);

  accelerate(HALF_SECTION, SEARCH_SPEED);
  i = 0;
  while (1) {
    if (g_second_run[i] > 0) {
      straight(g_second_run[i] * SECTION, SEARCH_SPEED, MAX_SPEED, SEARCH_SPEED);
    }
    i++;
    if (g_second_run[i] == 127) {
      break;
    } else if (g_second_run[i] == R90) {
      decelerate(HALF_SECTION, SEARCH_SPEED);
      g_map_control.nextDir(right);
      rotate(right, 1);
      accelerate(HALF_SECTION, SEARCH_SPEED);
    } else if (g_second_run[i] == L90) {
      decelerate(HALF_SECTION, SEARCH_SPEED);
      g_map_control.nextDir(left);
      rotate(left, 1);
      accelerate(HALF_SECTION, SEARCH_SPEED);
    }
    i++;
  }
  decelerate(HALF_SECTION, SEARCH_SPEED);
}

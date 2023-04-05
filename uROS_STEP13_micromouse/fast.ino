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

unsigned char         second_run[256];

void fast_run(short gx, short gy) {
  t_direction_glob glob_nextdir;
  int straight_count = 0;
  int i = 0;

//rvizに表示するマーカーの座標の初期化
  start_x=map_control.get_mypos_x();
  start_y=map_control.get_mypos_y();
  start_dir=map_control.get_mypos_dir();

//マーカー用のデータを作成
  t_direction temp_next_dir = map_control.get_nextdir2(gx, gy, &glob_nextdir);
  map_control.set_mypos_dir(glob_nextdir);
  map_control.axis_update();
  while ((map_control.get_mypos_x() != gx) || (map_control.get_mypos_y() != gy)) {
    switch (map_control.get_nextdir2(gx, gy, &glob_nextdir)) {    
      case front:
        straight_count++;
        break;
      case right:
        second_run[i++] = straight_count;
        second_run[i++] = R90;
        straight_count = 0;
        break;
      case left:
        second_run[i++] = straight_count;
        second_run[i++] = L90;
        straight_count = 0; 
        break;
    }
    map_control.set_mypos_dir(glob_nextdir);
    map_control.axis_update();
  }

  second_run[i++] = straight_count;
  second_run[i++] = 127;  


  map_control.set_mypos_dir(start_dir);

//second_runにあるデータに沿って走行する。
  switch (temp_next_dir) {
    break;
    case right:
      rotate(right, 1);  //右に曲がって
      map_control.next_dir(right);
      break;
    case left:
      rotate(left, 1);  //左に曲がって
      map_control.next_dir(left);
      break;
    case rear:
      rotate(right, 2);  //180度に旋回して
      map_control.next_dir(right);
      map_control.next_dir(right);            
      break;
  }

  fast_task=true;//rvizに最短経路のマーカーを表示
  delay(10);

  accelerate(HALF_SECTION, SEARCH_SPEED);
  i=0;
  while(1){
    if(second_run[i]>0){
      straight(second_run[i] * SECTION, SEARCH_SPEED, MAX_SPEED, SEARCH_SPEED);
    }
    i++;
    if(second_run[i]==127){
      break;
    }else if(second_run[i]==R90){
        decelerate(HALF_SECTION, SEARCH_SPEED);
        map_control.next_dir(right);
        rotate(right, 1);
        accelerate(HALF_SECTION, SEARCH_SPEED);
    }else if(second_run[i]==L90){
        decelerate(HALF_SECTION, SEARCH_SPEED);
        map_control.next_dir(left);
        rotate(left, 1);
        accelerate(HALF_SECTION, SEARCH_SPEED);
    }
    i++;
  }
  decelerate(HALF_SECTION,SEARCH_SPEED);
}

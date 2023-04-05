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


void map_view(void) {
  Serial.printf("\x1b[2j");
  Serial.printf("\x1b[0;0H");
  Serial.printf("\n\r+");
  for (int i = 0; i < MAZESIZE_X; i++) {

    switch (map_control.get_wall_data(i, MAZESIZE_Y - 1, north)) {  //黒色は"[30m"
      case NOWALL:
        Serial.printf("\x1b[37m  +");  //NOWALL
        break;
      case WALL:
        Serial.printf("\x1b[37m--+");  //WALL
        break;
      case _UNKNOWN:
        Serial.printf("\x1b[31m--+");  //UNNOWN
        break;
      default:
        Serial.printf("\x1b[33m--+");  //VWALL
        break;
    }
  }
  Serial.printf("\n\r");
  for (int j = (MAZESIZE_Y - 1); j > -1; j--) {
    switch (map_control.get_wall_data(0, j, west)) {
      case NOWALL:
        Serial.printf("\x1b[37m ");  //NOWALL
        break;
      case WALL:
        Serial.printf("\x1b[37m|");  //WALL
        break;
      case _UNKNOWN:
        Serial.printf("\x1b[31m|");  //UNNOWN
        break;
      default:
        Serial.printf("\x1b[33m|");  //VWALL
        break;
    }
    for (int i = 0; i < MAZESIZE_X; i++) {
      switch (map_control.get_wall_data(i, j, east)) {
        case NOWALL:
          Serial.printf("\x1b[37m   ");  //NOWALL
          break;
        case WALL:
          Serial.printf("\x1b[37m  |");  //WALL
          break;
        case _UNKNOWN:
          Serial.printf("\x1b[31m  |");  //UNNOWN
          break;
        default:
          Serial.printf("\x1b[33m  |");  //VWALL
          break;
      }
    }
    Serial.printf("\n\r+");
    for (int i = 0; i < MAZESIZE_X; i++) {
      switch (map_control.get_wall_data(i, j, south)) {
        case NOWALL:
          Serial.printf("\x1b[37m  +");  //NOWALL
          break;
        case WALL:
          Serial.printf("\x1b[37m--+");  //WALL
          break;
        case _UNKNOWN:
          Serial.printf("\x1b[31m--+");  //UNNOWN
          break;
        default:
          Serial.printf("\x1b[33m--+");  //VWALL
          break;
      }
    }
    Serial.printf("\n\r");
  }
}

void view_adc(void) {
  int i;
  DisableMotor();

  while (1) {
    Serial.printf("r_sen        is\t%d   \r\n", sen_r.value);
    Serial.printf("fr_sen       is\t%d   \r\n", sen_fr.value);
    Serial.printf("fl_sen       is\t%d  \r\n", sen_fl.value);
    Serial.printf("l_sen        is\t%d   \r\n", sen_l.value);
    Serial.printf("VDD          is\t%d mV\r\n", battery_value);
    Serial.printf("\n\r");  //改行
    delay(100);
    Serial.printf("\x1b[2j");
    Serial.printf("\x1b[0;0H");
  }
}

void straight_check(int section_check) {
  EnableMotor();
  delay(1000);
  accelerate(HALF_SECTION, SEARCH_SPEED);
  if (section_check > 1) {
    straight(SECTION * (section_check - 1), SEARCH_SPEED, MAX_SPEED, SEARCH_SPEED);
  }
  decelerate(HALF_SECTION, SEARCH_SPEED);

  DisableMotor();
}


void go_and_turn_right(void) {
  EnableMotor();
  delay(1000);
  for (int i = 0; i < 8; i++) {
    rotate(right, 1);
  }
  DisableMotor();
}


void adjust_menu(void) {
  unsigned char _mode = 1;
  char LED3_data;
  char sw;

  while (1) {
    SetLED(_mode);
    while (1) {
      sw = GetSW();
      if (sw != 0) break;
      delay(33);
      LED3_data ^= 1;
      SetLED((_mode & 0x7) + ((LED3_data << 3) & 0x08));
    }
    LED3_data = 0;
    switch (sw) {
      case SW_RM:
        _mode = inc_button(_mode, 7, 1);
        break;
      case SW_LM:
        _mode = dec_button(_mode, 1, 7);
        break;
      case SW_CM:
        ok_button();
        if (exec_by_mode_adjust(_mode) == 1) {
          return;
        }
        break;
    }
  }
}

unsigned char exec_by_mode_adjust(unsigned char _mode) {
  DisableMotor();
  switch (_mode) {
    case 1:
      view_adc();
      break;
    case 2:
      straight_check(7);
      break;

    case 3:
      go_and_turn_right();
      break;
    case 4:
      copy_map();
      map_view();
      break;

    case 5:
      break;

    case 6:
      break;

    case 7:
      return 1;
      break;
  }

  return 0;
}

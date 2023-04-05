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


short inc_button(short _data, short limit, short limit_data) {
  _data++;
  if (_data > limit) {
    _data = limit_data;
  }
  EnableBuzzer(INC_FREQ);
  delay(30);
  DisableBuzzer();
  return _data;
}
short dec_button(short _data, short limit, short limit_data) {
  _data--;
  if (_data < limit) {
    _data = limit_data;
  }
  EnableBuzzer(DEC_FREQ);
  delay(30);
  DisableBuzzer();
  return _data;
}

void ok_button(void) {
  EnableBuzzer(DEC_FREQ);
  delay(80);
  EnableBuzzer(INC_FREQ);
  delay(80);
  DisableBuzzer();
}

void goal_appeal(void)  //ゴールしたことをアピールする
{
  unsigned char led_data;

  int wtime = 100;

  DisableMotor();
  map_write();

  for (int j = 0; j < 10; j++) {
    led_data = 1;
    for (int i = 0; i < 4; i++) {
      SetLED(led_data);
      led_data <<= 1;
      delay(wtime);
    }
    led_data = 8;
    for (int i = 0; i < 4; i++) {
      SetLED(led_data);
      led_data >>= 1;
      delay(wtime);
    }
    wtime -= 5;
  }

  delay(500);
  EnableMotor();
}

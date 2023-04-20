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

short incButton(short data, short limit, short limit_data)
{
  data++;
  if (data > limit) {
    data = limit_data;
  }
  enableBuzzer(INC_FREQ);
  delay(30);
  disableBuzzer();
  return data;
}
short decButton(short data, short limit, short limit_data)
{
  data--;
  if (data < limit) {
    data = limit_data;
  }
  enableBuzzer(DEC_FREQ);
  delay(30);
  disableBuzzer();
  return data;
}

void okButton(void)
{
  enableBuzzer(DEC_FREQ);
  delay(80);
  enableBuzzer(INC_FREQ);
  delay(80);
  disableBuzzer();
}

void goalAppeal(void)  //ゴールしたことをアピールする
{
  unsigned char led_data;

  int wtime = 100;

  disableMotor();
  mapWrite();

  for (int j = 0; j < 10; j++) {
    led_data = 1;
    for (int i = 0; i < 4; i++) {
      setLED(led_data);
      led_data <<= 1;
      delay(wtime);
    }
    led_data = 8;
    for (int i = 0; i < 4; i++) {
      setLED(led_data);
      led_data >>= 1;
      delay(wtime);
    }
    wtime -= 5;
  }

  delay(500);
  enableMotor();
}

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

#define SLED_FR 16
#define SLED_FL 15
#define SLED_R 18
#define SLED_L 17

#define AD4 7
#define AD3 6
#define AD2 5
#define AD1 4
#define AD0 8

volatile short sensor_fr_value;
volatile short sensor_fl_value;
volatile short sensor_r_value;
volatile short sensor_l_value;
volatile short battery_value;

hw_timer_t* timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR OnTimer1(void) {
  static char cnt = 0;
  portENTER_CRITICAL_ISR(&timerMux);
  switch (cnt) {
    case 0:
      digitalWrite(SLED_FR, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sensor_fr_value = analogRead(AD1);
      digitalWrite(SLED_FR, LOW);  //LED消灯
      break;
    case 1:
      digitalWrite(SLED_FL, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sensor_fl_value = analogRead(AD2);
      digitalWrite(SLED_FL, LOW);  //LED消灯
      break;
    case 2:
      digitalWrite(SLED_R, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sensor_r_value = analogRead(AD3);
      digitalWrite(SLED_R, LOW);  //LED消灯
      break;
    case 3:
      digitalWrite(SLED_L, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      sensor_l_value = analogRead(AD4);
      digitalWrite(SLED_L, LOW);  //LED消灯
      battery_value = (double)analogReadMilliVolts(AD0) / 10.0 * (10.0 + 51.0);
      break;
  }
  cnt++;
  if (cnt == 4) cnt = 0;
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup() {
  // put your setup code here, to run once:
  //Sensor 発光off
  pinMode(SLED_FR, OUTPUT);
  pinMode(SLED_FL, OUTPUT);
  pinMode(SLED_R, OUTPUT);
  pinMode(SLED_L, OUTPUT);
  digitalWrite(SLED_FR, LOW);
  digitalWrite(SLED_FL, LOW);
  digitalWrite(SLED_R, LOW);
  digitalWrite(SLED_L, LOW);

  Serial.begin(115200);

  timer1 = timerBegin(1, 80, true);  //1us
  timerAttachInterrupt(timer1, &OnTimer1, true);
  timerAlarmWrite(timer1, 250, true);  //4kHz
  timerAlarmEnable(timer1);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.printf("r_sen  is %d\n\r", sensor_r_value);
  Serial.printf("fr_sen is %d\n\r", sensor_fr_value);
  Serial.printf("fl_sen is %d\n\r", sensor_fl_value);
  Serial.printf("l_sen  is %d\n\r", sensor_l_value);
  Serial.printf("VDD    is %d\n\r", battery_value);
  delay(100);
}

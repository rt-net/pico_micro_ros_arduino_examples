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

volatile short g_sensor_value_fr;
volatile short g_sensor_value_fl;
volatile short g_sensor_value_r;
volatile short g_sensor_value_l;
volatile short g_battery_value;

hw_timer_t * g_timer1 = NULL;
portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer1(void)
{
  static char cnt = 0;
  portENTER_CRITICAL_ISR(&g_timer_mux);
  switch (cnt) {
    case 0:
      digitalWrite(SLED_FR, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      g_sensor_value_fr = analogRead(AD1);
      digitalWrite(SLED_FR, LOW);  //LED消灯
      break;
    case 1:
      digitalWrite(SLED_FL, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      g_sensor_value_fl = analogRead(AD2);
      digitalWrite(SLED_FL, LOW);  //LED消灯
      break;
    case 2:
      digitalWrite(SLED_R, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      g_sensor_value_r = analogRead(AD3);
      digitalWrite(SLED_R, LOW);  //LED消灯
      break;
    case 3:
      digitalWrite(SLED_L, HIGH);  //LED点灯
      for (int i = 0; i < 300; i++) {
        asm("nop \n");
      }
      g_sensor_value_l = analogRead(AD4);
      digitalWrite(SLED_L, LOW);  //LED消灯
      g_battery_value = (double)analogReadMilliVolts(AD0) / 10.0 * (10.0 + 51.0);
      break;
    default:
      Serial.printf("error¥n¥r");
      break;
  }
  cnt++;
  if (cnt == 4) cnt = 0;
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void setup()
{
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

  g_timer1 = timerBegin(1000000);  //1MHz(1us)
  timerAttachInterrupt(g_timer1, &onTimer1);
  timerAlarm(g_timer1, 250, true, 0);  //250 * 1us =250us(4kHz)
  timerStart(g_timer1);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.printf("r_sen  is %d\n\r", g_sensor_value_r);
  Serial.printf("fr_sen is %d\n\r", g_sensor_value_fr);
  Serial.printf("fl_sen is %d\n\r", g_sensor_value_fl);
  Serial.printf("l_sen  is %d\n\r", g_sensor_value_l);
  Serial.printf("VDD    is %d\n\r", g_battery_value);
  delay(100);
}

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

#include "driver/adc.h"

hw_timer_t * g_timer0 = NULL;
hw_timer_t * g_timer1 = NULL;
hw_timer_t * g_timer2 = NULL;
hw_timer_t * g_timer3 = NULL;

volatile double motor_signed_r, motor_signed_l;
volatile unsigned short g_step_hz_r, g_step_hz_l;
volatile unsigned int g_step_r, g_step_l;

portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void setRStepHz(short data) { g_step_hz_r = data; }

void setLStepHz(short data) { g_step_hz_l = data; }

void clearStepR(void) { g_step_r = 0; }

void clearStepL(void) { g_step_l = 0; }

unsigned int getStepR(void) { return g_step_r; }

unsigned int getStepL(void) { return g_step_l; }

double motorSignedR(void) { return motor_signed_r; }

double motorSignedL(void) { return motor_signed_l; }

void IRAM_ATTR onTimer0(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  controlInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void IRAM_ATTR onTimer1(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  sensorInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void IRAM_ATTR isrR(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  if (g_motor_move) {
    if (g_step_hz_r < 30) g_step_hz_r = 30;
    timerAlarmWrite(g_timer2, 2000000 / g_step_hz_r, true);
    digitalWrite(PWM_R, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    }
    digitalWrite(PWM_R, LOW);
    g_step_r++;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void IRAM_ATTR isrL(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  if (g_motor_move) {
    if (g_step_hz_l < 30) g_step_hz_l = 30;
    timerAlarmWrite(g_timer3, 2000000 / g_step_hz_l, true);
    digitalWrite(PWM_L, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    };
    digitalWrite(PWM_L, LOW);
    g_step_l++;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void controlInterruptStart(void) { timerAlarmEnable(g_timer0); }
void controlInterruptStop(void) { timerAlarmDisable(g_timer0); }

void sensorInterruptStart(void) { timerAlarmEnable(g_timer1); }
void sensorInterruptStop(void) { timerAlarmDisable(g_timer1); }

void PWMInterruptStart(void)
{
  timerAlarmEnable(g_timer2);
  timerAlarmEnable(g_timer3);
}
void PWMInterruptStop(void)
{
  timerAlarmDisable(g_timer2);
  timerAlarmDisable(g_timer3);
}

void initDevice(void)
{
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(BLED0, OUTPUT);
  pinMode(BLED1, OUTPUT);

  pinMode(SW_L, INPUT);
  pinMode(SW_C, INPUT);
  pinMode(SW_R, INPUT);

  ledcSetup(0, 440, 10);
  ledcAttachPin(BUZZER, 0);
  ledcWrite(0, 1024);

  pinMode(SLED_FR, OUTPUT);
  pinMode(SLED_FL, OUTPUT);
  pinMode(SLED_R, OUTPUT);
  pinMode(SLED_L, OUTPUT);
  digitalWrite(SLED_FR, LOW);
  digitalWrite(SLED_FL, LOW);
  digitalWrite(SLED_R, LOW);
  digitalWrite(SLED_L, LOW);

  pinMode(MOTOR_EN, OUTPUT);
  pinMode(CW_R, OUTPUT);
  pinMode(CW_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);
  digitalWrite(CW_R, LOW);
  digitalWrite(CW_L, LOW);
  digitalWrite(PWM_R, LOW);
  digitalWrite(PWM_L, LOW);

  if (!SPIFFS.begin(true)) {
    while (1) {
      Serial.println("SPIFFS Mount Failed");
      delay(100);
    }
  }

  g_timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(g_timer0, &onTimer0, false);
  timerAlarmWrite(g_timer0, 1000, true);
  timerAlarmEnable(g_timer0);

  g_timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(g_timer1, &onTimer1, true);
  timerAlarmWrite(g_timer1, 500, true);
  timerAlarmEnable(g_timer1);

  g_timer2 = timerBegin(2, 40, true);
  timerAttachInterrupt(g_timer2, &isrR, false);
  timerAlarmWrite(g_timer2, 13333, true);
  timerAlarmEnable(g_timer2);

  g_timer3 = timerBegin(3, 40, true);
  timerAttachInterrupt(g_timer3, &isrL, false);
  timerAlarmWrite(g_timer3, 13333, true);
  timerAlarmEnable(g_timer3);

  Serial.begin(115200);

  enableBuzzer(INC_FREQ);
  delay(80);
  disableBuzzer();
}

//LED
void setLED(unsigned char data)
{
  digitalWrite(LED0, data & 0x01);
  digitalWrite(LED1, (data >> 1) & 0x01);
  digitalWrite(LED2, (data >> 2) & 0x01);
  digitalWrite(LED3, (data >> 3) & 0x01);
}
void setBLED(char data)
{
  if (data & 0x01) {
    digitalWrite(BLED0, HIGH);
  } else {
    digitalWrite(BLED0, LOW);
  }
  if (data & 0x02) {
    digitalWrite(BLED1, HIGH);
  } else {
    digitalWrite(BLED1, LOW);
  }
}

//Buzzer
void enableBuzzer(short f) { ledcWriteTone(BUZZER_CH, f); }
void disableBuzzer(void)
{
  ledcWrite(BUZZER_CH, 1024);  //duty 100% Buzzer OFF
}

//motor
void enableMotor(void)
{
  digitalWrite(MOTOR_EN, HIGH);  //Power ON
}
void disableMotor(void)
{
  digitalWrite(MOTOR_EN, LOW);  //Power OFF
}

void moveDir(t_CW_CCW left_CW, t_CW_CCW right_CW)
{  //左右のモータの回転方向を指示する
  if (right_CW == MOT_FORWARD) {
    digitalWrite(CW_R, LOW);
    motor_signed_r = 1.0;
  } else {
    digitalWrite(CW_R, HIGH);
    motor_signed_r = -1.0;
  }

  if (left_CW == MOT_FORWARD) {
    digitalWrite(CW_L, LOW);
    motor_signed_l = 1.0;
  } else {
    digitalWrite(CW_L, HIGH);
    motor_signed_l = -1.0;
  }
}

//SWITCH
unsigned char getSW(void)
{
  unsigned char ret = 0;
  if (digitalRead(SW_R) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW_R) == LOW);
    ret |= SW_RM;
  }
  if (digitalRead(SW_C) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW_C) == LOW);
    ret |= SW_CM;
  }
  if (digitalRead(SW_L) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW_L) == LOW);
    ret |= SW_LM;
  }
  return ret;
}

//sensor
unsigned short getSensorR(void)
{
  digitalWrite(SLED_R, HIGH);
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  //  unsigned short tmp = analogRead(AD3);
  unsigned short tmp = adc1_get_raw(ADC1_CHANNEL_5);
  digitalWrite(SLED_R, LOW);
  return tmp;
}
unsigned short getSensorL(void)
{
  digitalWrite(SLED_L, HIGH);
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  //  unsigned short tmp = analogRead(AD4);
  unsigned short tmp = adc1_get_raw(ADC1_CHANNEL_6);
  digitalWrite(SLED_L, LOW);
  return tmp;
}
unsigned short getSensorFL(void)
{
  digitalWrite(SLED_FL, HIGH);  //LED点灯
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  //  unsigned short tmp = analogRead(AD2);
  unsigned short tmp = adc1_get_raw(ADC1_CHANNEL_4);
  digitalWrite(SLED_FL, LOW);  //LED消灯
  return tmp;
}
unsigned short getSensorFR(void)
{
  digitalWrite(SLED_FR, HIGH);
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  //  unsigned short tmp = analogRead(AD1);
  unsigned short tmp = adc1_get_raw(ADC1_CHANNEL_3);
  digitalWrite(SLED_FR, LOW);
  return tmp;
}
short getBatteryVolt(void) { return (double)analogReadMilliVolts(AD0) / 10.0 * (10.0 + 51.0); }

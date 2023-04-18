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

hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;
hw_timer_t * timer3 = NULL;

volatile double MotorSingedR, MotorSingedL;
volatile unsigned short RStepHz, LStepHz;
volatile unsigned int StepR, StepL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void SetRStepHz(short data) { RStepHz = data; }

void SetLStepHz(short data) { LStepHz = data; }

void ClearStepR(void) { StepR = 0; }

void ClearStepL(void) { StepL = 0; }

unsigned int GetStepR(void) { return StepR; }

unsigned int GetStepL(void) { return StepL; }

double RMotorSinged(void) { return MotorSingedR; }

double LMotorSinged(void) { return MotorSingedL; }

void IRAM_ATTR OnTimer0(void)
{
  portENTER_CRITICAL_ISR(&timerMux);
  control_interrupt();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR OnTimer1(void)
{
  portENTER_CRITICAL_ISR(&timerMux);
  sensor_interrupt();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR IsrR(void)
{
  portENTER_CRITICAL_ISR(&timerMux);
  if (motor_move) {
    if (RStepHz < 30) RStepHz = 30;
    timerAlarmWrite(timer2, 2000000 / RStepHz, true);
    digitalWrite(PWM_R, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    }
    digitalWrite(PWM_R, LOW);
    StepR++;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR IsrL(void)
{
  portENTER_CRITICAL_ISR(&timerMux);
  if (motor_move) {
    if (LStepHz < 30) LStepHz = 30;
    timerAlarmWrite(timer3, 2000000 / LStepHz, true);
    digitalWrite(PWM_L, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    };
    digitalWrite(PWM_L, LOW);
    StepL++;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void ControlInterruptStart(void) { timerAlarmEnable(timer0); }
void ControlInterruptStop(void) { timerAlarmDisable(timer0); }

void SensorInterruptStart(void) { timerAlarmEnable(timer1); }
void SensorInterruptStop(void) { timerAlarmDisable(timer1); }

void PWMInterruptStart(void)
{
  timerAlarmEnable(timer2);
  timerAlarmEnable(timer3);
}
void PWMInterruptStop(void)
{
  timerAlarmDisable(timer2);
  timerAlarmDisable(timer3);
}

void InitDevice(void)
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

  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &OnTimer0, false);
  timerAlarmWrite(timer0, 1000, true);
  timerAlarmEnable(timer0);

  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, &OnTimer1, true);
  timerAlarmWrite(timer1, 500, true);
  timerAlarmEnable(timer1);

  timer2 = timerBegin(2, 40, true);
  timerAttachInterrupt(timer2, &IsrR, false);
  timerAlarmWrite(timer2, 13333, true);
  timerAlarmEnable(timer2);

  timer3 = timerBegin(3, 40, true);
  timerAttachInterrupt(timer3, &IsrL, false);
  timerAlarmWrite(timer3, 13333, true);
  timerAlarmEnable(timer3);

  Serial.begin(115200);

  EnableBuzzer(INC_FREQ);
  delay(80);
  DisableBuzzer();
}

//LED
void SetLED(unsigned char _data)
{
  digitalWrite(LED0, _data & 0x01);
  digitalWrite(LED1, (_data >> 1) & 0x01);
  digitalWrite(LED2, (_data >> 2) & 0x01);
  digitalWrite(LED3, (_data >> 3) & 0x01);
}
void SetBLED(char _data)
{
  if (_data & 0x01) {
    digitalWrite(BLED0, HIGH);
  } else {
    digitalWrite(BLED0, LOW);
  }
  if (_data & 0x02) {
    digitalWrite(BLED1, HIGH);
  } else {
    digitalWrite(BLED1, LOW);
  }
}

//Buzzer
void EnableBuzzer(short f) { ledcWriteTone(BUZZER_CH, f); }
void DisableBuzzer(void)
{
  ledcWrite(BUZZER_CH, 1024);  //duty 100% Buzzer OFF
}

//motor
void EnableMotor(void)
{
  digitalWrite(MOTOR_EN, HIGH);  //Power ON
}
void DisableMotor(void)
{
  digitalWrite(MOTOR_EN, LOW);  //Power OFF
}

void MoveDir(t_CW_CCW left_CW, t_CW_CCW right_CW)
{  //左右のモータの回転方向を指示する
  if (right_CW == MOT_FORWARD) {
    digitalWrite(CW_R, LOW);
    MotorSingedR = 1.0;
  } else {
    digitalWrite(CW_R, HIGH);
    MotorSingedR = -1.0;
  }

  if (left_CW == MOT_FORWARD) {
    digitalWrite(CW_L, LOW);
    MotorSingedL = 1.0;
  } else {
    digitalWrite(CW_L, HIGH);
    MotorSingedL = -1.0;
  }
}

//SWITCH
unsigned char GetSW(void)
{
  int i;
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
unsigned short GetSensorR(void)
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
unsigned short GetSensorL(void)
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
unsigned short GetSensorFL(void)
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
unsigned short GetSensorFR(void)
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
short GetBatteryVolt(void) { return (double)analogReadMilliVolts(AD0) / 10.0 * (10.0 + 51.0); }

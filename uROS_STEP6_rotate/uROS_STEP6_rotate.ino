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

#define LED0 1
#define LED1 2
#define LED2 42
#define LED3 41

#define SW_L 10
#define SW_C 11
#define SW_R 12

#define MOTOR_EN 9
#define CW_R 14
#define CW_L 21
#define PWM_R 13
#define PWM_L 45

#define MIN_HZ 80
#define TIRE_DIAMETER (48.00)
#define PULSE (TIRE_DIAMETER * PI / 400.0)
#define MIN_SPEED (MIN_HZ * PULSE)
#define TREAD_WIDTH (65.00)

typedef enum {
  front,
  right,
  rear,
  left,
  unknown,
} t_local_dir;

hw_timer_t * timer0 = NULL;
hw_timer_t * timer2 = NULL;
hw_timer_t * timer3 = NULL;

portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED;

unsigned short r_step_hz = MIN_HZ;
unsigned short l_step_hz = MIN_HZ;

volatile unsigned int step_r, step_l;
double max_speed;
double min_speed;
double r_accel = 0.0;
volatile double speed = MIN_SPEED;

volatile bool motor_move = 0;

//割り込み
//目標値の更新周期1kHz
void IRAM_ATTR onTimer0(void)
{
  portENTER_CRITICAL_ISR(&timer_mux);  //割り込み禁止
  controlInterrupt();
  portEXIT_CRITICAL_ISR(&timer_mux);  //割り込み許可
}

//Rモータの周期数割り込み
void IRAM_ATTR isrR(void)
{
  portENTER_CRITICAL_ISR(&timer_mux);  //割り込み禁止
  if (motor_move) {
    timerAlarmWrite(timer2, 2000000 / r_step_hz, true);
    digitalWrite(PWM_R, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    }
    digitalWrite(PWM_R, LOW);
    step_r++;
  }
  portEXIT_CRITICAL_ISR(&timer_mux);  //割り込み許可
}

//Lモータの周期数割り込み
void IRAM_ATTR isrL(void)
{
  portENTER_CRITICAL_ISR(&timer_mux);  //割り込み禁止
  if (motor_move) {
    timerAlarmWrite(timer3, 2000000 / l_step_hz, true);
    digitalWrite(PWM_L, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    };
    digitalWrite(PWM_L, LOW);
    step_l++;
  }
  portEXIT_CRITICAL_ISR(&timer_mux);  //割り込み許可
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(SW_L, INPUT);
  pinMode(SW_C, INPUT);
  pinMode(SW_R, INPUT);

  //motor disable
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

  timer0 = timerBegin(0, 80, true);  //1us
  timerAttachInterrupt(timer0, &onTimer0, true);
  timerAlarmWrite(timer0, 1000, true);  //1kHz
  timerAlarmEnable(timer0);

  timer2 = timerBegin(2, 40, true);  //0.5us
  timerAttachInterrupt(timer2, &isrR, true);
  timerAlarmWrite(timer2, 13333, true);  //150Hz
  timerAlarmEnable(timer2);

  timer3 = timerBegin(3, 40, true);  //0.5us
  timerAttachInterrupt(timer3, &isrL, true);
  timerAlarmWrite(timer3, 13333, true);  //150Hz
  timerAlarmEnable(timer3);
}

void loop()
{
  // put your main code here, to run repeatedly:
  while (digitalRead(SW_L) & digitalRead(SW_C) & digitalRead(SW_R)) {
    continue;
  }
  digitalWrite(MOTOR_EN, HIGH);
  delay(1000);
  rotate(right, 1);
  delay(1000);
  rotate(left, 1);
  delay(1000);
  rotate(right, 2);
  delay(1000);
  rotate(left, 2);
  delay(1000);
  digitalWrite(MOTOR_EN, LOW);
}
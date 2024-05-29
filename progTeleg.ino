#include <Wire.h>
#include "BH1750FVI.h"
#include <Arduino.h>
#include "ConnectionList.h"
#include "Defines.h"
#include "Motor.h"


MotorConnectionParams mconnp1 =
{
    .IN =      MOTOR1_IN,
    .EN =      MOTOR1_EN,
    .ENC_PPR =  MOTORS_PPR,
    .i =        MOTORS_GEAR_RATIO,
    .ke =       MOTORS_KE,
    .ENC_PORT = MOTOR1_ENC_PORT,
    .ENC_MASK = MOTOR1_ENC_MASK,
    .ENC_SHIFT= MOTOR1_ENC_SHIFT,
    .ENC_DIR =  MOTOR1_ENC_DIR,
};
MotorConnectionParams mconnp2 =
{
    .IN =      MOTOR2_IN,
    .EN =      MOTOR2_EN,
    .ENC_PPR =  MOTORS_PPR,
    .i =        MOTORS_GEAR_RATIO,
    .ke =       MOTORS_KE,
    .ENC_PORT = MOTOR2_ENC_PORT,
    .ENC_MASK = MOTOR2_ENC_MASK,
    .ENC_SHIFT= MOTOR2_ENC_SHIFT,
    .ENC_DIR =  MOTOR2_ENC_DIR,
};
MotorControllerParams mctrlp = 
{
    .maxU = MOTORS_MAX_U,
    .moveU = MOTORS_MOVE_U,
    .maxSpeed = MOTORS_MAX_SPEED,
    .maxAccel = 9999,
    .Ts = Ts_s,
    .kp = MOTORS_PI_GAIN,
    .ki = MOTORS_PI_KI,
    .speedFilterT = 2*Ts_s,
    .maxUi = 9999
};
Motor motor1(&mconnp1, &mctrlp);
Motor motor2(&mconnp2, &mctrlp);

ISR(PCINT2_vect){motor1.interruptHandler();}    // Port D, PCINT16 - PCINT23
ISR(PCINT0_vect){motor2.interruptHandler();}    // Port B, PCINT0 - PCINT7

uint32_t tmr1 = 0; // таймер
uint32_t tmr2 = 0; // 2 таймер
int grey = 165; // усл значение серого
int v_f = 65; // скорость вперёд
float Kp = 1; // коэффициент пропорцианального регулятора
int prkt = 0; // счётчик перекрёстков 
bool is_black = false; // видит ли чёрный цвет датчик
bool is_blackOld = false; // видел ли датчик чёрный цвет в предыдущей итерации
bool tmr2_flag = false; // флаг 2 таймера
int M1 = 12; // левый по ходу мотор на шилде
int M2 = 13; // правый по ходу мотор на шилде
int E1 = 10; // enable левого мотора
int E2 = 11; // enable правого мотора
int perkt_sens;
int perkt_sensOld;

uint32_t dt = 0;
float S1;
float S2;
float err;
float up;
int u;
int v_L;
int v_R;

BH1750FVI s1(BH1750_SECOND_I2CADDR, BH1750_CONTINUOUS_LOW_RES_MODE, BH1750_SENSITIVITY_MIN, BH1750_ACCURACY_MIN);  // обьявление датчика S1
BH1750FVI s2(BH1750_DEFAULT_I2CADDR, BH1750_CONTINUOUS_LOW_RES_MODE, BH1750_SENSITIVITY_MIN, BH1750_ACCURACY_MIN); // обьявление датчика S2

void setup()
{
  Wire.begin(); // инициализация библиотеки Wire позволяющую использовать шину I2c
  Serial.begin(115200); // инициализация последавательного порта

  pinMode(M1, OUTPUT); // устанавливаем пин M1 на выход
  pinMode(M2, OUTPUT); // устанавливаем пин M2 на выход
  pinMode(A3, INPUT);
  pinMode(A0, OUTPUT);

  pinMode(S_LEFT, INPUT);
  pinMode(S_RIGHT, INPUT);

  s1.begin(); // инициализируем датчик S1
  s2.begin(); // инициализируем датчик S1

  s1.setCalibration(1.06);                           //call before "readLightLevel()", 1.06=white LED & artifical sun
  s1.setSensitivity(1.00);                           //call before "readLightLevel()", 1.00=no optical filter in front of the sensor
  s1.setResolution(BH1750_ONE_TIME_LOW_RES_MODE);

  s2.setCalibration(1.06);                           //call before "readLightLevel()", 1.06=white LED & artifical sun
  s2.setSensitivity(1.00);                           //call before "readLightLevel()", 1.00=no optical filter in front of the sensor
  s2.setResolution(BH1750_ONE_TIME_LOW_RES_MODE);

  // Det timer 2 divisor to  256 for PWM frequency of 122.070312500 Hz
  TCCR2B = TCCR2B & B11111000 | B00000110;

  // https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
  PCICR |= 0b00000001;    // turn on port b
  PCICR |= 0b00000100;    // turn on port d change interrupts

  PCMSK0 |= 0b00000011;    // turn on pins PB0 and PB1 (D8 and D9)
  PCMSK2 |= 0b11000000;    // turn on pins PD6 & PD7 (D6 and D7)
}

enum
{
  initialization,
  line,
  manip_on,
  sleep,
};

int state = initialization;


void drive(int v_L, int v_R)
{
  motor1.setSpeed(v_R);
  motor2.setSpeed(v_L);

  motor1.update();
  motor2.update();
}

void change_state_by_timer(uint32_t delay_time, int next_state)
{
  if (tmr2_flag == false)
  {
    tmr2_flag = true;
    tmr2 = millis();
  }
  if (millis() - tmr2 > delay_time)
  {
    tmr2_flag = false;
    state = next_state;
  }
}

void loop()
{
  // таймер
  while (micros() - tmr1 < Ts_us);
  dt = micros() - tmr1;
  tmr1 = micros();

  // чтение датчика
  // S1 = s1.readLightLevel(); // перекрёстки
  // S2 = s2.readLightLevel(); // линия
  perkt_sens = digitalRead(A3);

  switch (state) {
    case initialization:
      change_state_by_timer(2000, line);
      break;
    case line:
      // п+ регулятор
      err = S1 - S2 + 11;
      
      up = Kp * err;
      u = up;
      // микшер
      v_L = v_f - u;
      v_R = v_f + u;
      // поверка перекрёстков
      if (perkt_sens == 0 && perkt_sensOld == 1) {
        prkt = prkt + 1;
      }
      // проверка 3 перекрёстков (условие перехода)
      if (prkt >= 3) {
        v_L = 0;
        v_R = 0;
        is_black = 0;
        is_blackOld = 0;
        state = manip_on;
      }
      break;
    case manip_on:
      // подаём сигнал
      digitalWrite(A0, HIGH);
      // условие перехода
      change_state_by_timer(100, sleep);
      break;
    case sleep:
      digitalWrite(A0, LOW);
      v_L = 0;
      v_R = 0;
      break;
  }
  // выдача управляющих воздействий
  drive(4, 4);

  // Логи
  Serial.println(
    "dt: " + String(dt)
    // LOG("motor1 angle", motor1.getAngle())

    // " S1: " + String(S1) +
    // " state: " + String(state) +
    // " prkt: " + String(prkt) +
    // " perkt_sens: " + String(perkt_sens) +
    // " S2: " + String(S2) +
    // " err: " + String(err) +
    // " u: " + String(u) +
    // " v_L: " + String(v_L) +
    // " is_blackOld: " + String(is_blackOld) +
    // " is_black: " + String(is_black) +
    // " v_R: " + String(v_R)
    );
  is_blackOld = is_black;
  perkt_sensOld = perkt_sens;
}

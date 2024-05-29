#pragma once

#include "Arduino.h"


//------------------------MOTORS---------------------------

#define MOTOR1_IN           12
#define MOTOR1_EN           10
#define MOTOR1_ENC_PORT     &PIND
#define MOTOR1_ENC_MASK     0b11000000
#define MOTOR1_ENC_SHIFT    6
#define MOTOR1_ENC_DIR      1

#define MOTOR2_IN           13
#define MOTOR2_EN           11
#define MOTOR2_ENC_PORT     &PINB
#define MOTOR2_ENC_MASK     0b00000011
#define MOTOR2_ENC_SHIFT    0
#define MOTOR2_ENC_DIR      -1


//-----------------------SENSORS---------------------------

#define S_LEFT              A2
#define S_RIGHT             A3


//-------------------------I2C-----------------------------

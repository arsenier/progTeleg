#pragma once

/////////////// TIME ////////////////

#define Ts_us                                   10000 /*!< sample time [microseconds]*/
#define Ts_s                                    ((float)Ts_us * 0.000001)


////////////// MOTORS //////////////

#define MOTORS_ROBOT_MAX_SPEED                  1500 /*!< [mm/s] */
    

#define MOTORS_MAX_U                            12.0 /*!< [V] */
#define MOTORS_MOVE_U                           0.1  /*!< [V] */
#define MOTORS_MAX_SPEED                        30.0     //rad/second
#define MOTORS_KE                               0.67  /*!< [V/rad/s] 12V/100rpm*/

#define MOTORS_WHEEL_RAD_MM                     35
#define MOTORS_ROBOT_HALF_BASE_MM               200

#define MOTORS_GEAR_RATIO                       75
#define MOTORS_PPR                              48     //Encoder pulses per rotation
#define MOTORS_TIME_CONSTANT                    0.200
#define MOTORS_Tu                               Ts_s
#define MOTORS_ABS_OPTIMUM_SETTING              2.0
#define MOTORS_PI_KI                            (MOTORS_KE/(MOTORS_ABS_OPTIMUM_SETTING*MOTORS_Tu))
#define MOTORS_PI_GAIN                          (MOTORS_PI_KI*MOTORS_TIME_CONSTANT)


///////////// LINE //////////////////

#define BLACK_VAL                               800
#define WHITE_VAL                               100
#define GREY_VAL                                ((BLACK_VAL + WHITE_VAL) / 2)

//////////////////////////
#define LOG(name, val)  + String("\t") + String(name) + String(": ") + String(val)

#ifndef UAGV_CORE_CONFIG_H_
#define UAGV_CORE_CONFIG_H_

#include <Wire.h>
#include <avr/wdt.h>

#include <math.h>

#define CONTROL_MOTOR_SPEED_PERIOD       30   //hz
#define IMU_PUBLISH_PERIOD               200  //hz
#define SENSOR_STATE_PUBLISH_PERIOD      20   //hz
#define CMD_VEL_PUBLISH_PERIOD           20   //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD 20   //hz
#define DRIVE_TEST_PERIOD                20   //hz

#define WHEEL_RADIUS                    0.09           // meter
#define WHEEL_SEPARATION                0.46           // meter (BURGER => 0.16, WAFFLE => 0.287)
#define ROBOT_RADIUS                    0.294           // meter (BURGER => 0.078, WAFFLE => 0.294)
#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define LEFT                            0
#define RIGHT                           1

#define VELOCITY_CONSTANT_VALUE         1263.632956882  // V = r * w = r * RPM * 0.10472
                                                        //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                        // Goal RPM = V * 1263.632956882

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_STEP                   0.01   // m/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define TICK2RAD                        0.000628318  // 2 * 3.14159265359 / 10000 = 0.000628318f

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define TEST_DISTANCE                   0.300     // meter
#define TEST_RADIAN                     3.14      // 180 degree

#define WAIT_FOR_BUTTON_PRESS           0
#define WAIT_SECOND                     1
#define CHECK_BUTTON_RELEASED           2

void motorRun(void);
void i2cenc(void);
void byteconvert(int motorOutPut, int transmission);
long i2cenc2(byte SA);
long check(byte SA);

#endif // UAGV_CORE_CONFIG_H_

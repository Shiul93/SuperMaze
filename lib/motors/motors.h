#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"

#define RMOTOR  0
#define LMOTOR  1
#define RLMOTOR 2

extern void setupPWM();
extern void motorSpeed(int motor, boolean direction, int speed);
extern void motorSpeedUpdate(int Rspeed, int Lspeed);
extern void motorSpeedPercent(int motor, int speed);
extern void motorBrake(int motor);
extern void motorCoast(int motor);

#endif

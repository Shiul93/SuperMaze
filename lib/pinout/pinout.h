
#ifndef PINOUT_H
#define PINOUT_H

#include "Arduino.h"

//Motor driver TB6612
extern int driver_PWMA;
extern int driver_PWMB;
extern int driver_AIN1;
extern int driver_AIN2;
extern int driver_BIN1;
extern int driver_BIN2;
extern int driver_STBY;

//Buzzer
extern int buzzer_pin;

//Encoders
extern int encoderR1;
extern int encoderR2;
extern int encoderL1;
extern int encoderL2;

//TOF Sensors
extern int sensor_rst1;
extern int sensor_rst2;
extern int sensor_rst3;
extern int sensor_rst4;

//Buttons
extern int button1;
extern int button2;

//Neopixel

extern int neopixel;

extern void setupPins();

#endif

#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include "MPU9250.h"
#include <VL53L0X.h>

extern int distFL, distCL, distCR, distFR;

extern int meanindex;
extern int distCLarray[];
extern int distFLarray[];
extern int distCRarray[];
extern int distFRarray[];
extern float accelArray[];
extern float gyroArray[];

extern void startIMU();
extern void readAccelValues();
extern void readGyroValues();
extern void setupDistanceSensors();
extern void updateDistances();
#endif

#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include "MPU9250.h"




void startIMU();
void readAccelValues(float * coords);
void readGyroValues(float * coords);
#endif

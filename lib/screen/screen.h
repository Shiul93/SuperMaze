#ifndef SCREEN_H
#define SCREEN_H

#include "Arduino.h"
#include <U8g2lib.h>


void displayPID(double ep, double ei, double ed, double error);
void displayENC (int R, int L, int dist, int angle);
void displayAccel (float * coords);
void displayGyro (float * coords);
extern void displayInt(int data);
extern void setupScreen();
void displayString (char * s);
#endif

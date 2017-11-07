#ifndef BEHAVIORS_H
#define BEHAVIORS_H

#include "Arduino.h"
 
#define GYRO_KP 1.5
#define GYRO_KI 0.05
#define GYRO_KD 0.15

#define SPEEDL_KP 50
#define SPEEDL_KI 0
#define SPEEDL_KD 1
#define SPEEDR_KP 50
#define SPEEDR_KI 0
#define SPEEDR_KD 1

#define WALL_KP 0.50
#define WALL_KI 0.01
#define WALL_KD 0.02

#define ROT_KP 20
#define ROT_KI 0 
#define ROT_KD 3
#define ROT_ALMOST 5
#define ROT_SPEED_LIMIT_LOW 30
#define ROT_SPEED_LIMIT_HIGH 75

#define DIST_KP 0.3
#define DIST_KI 0.05 
#define DIST_KD 0.15
#define DIST_ALMOST 5
#define DIST_SPEED_LIMIT_LOW 30
#define DIST_SPEED_LIMIT_HIGH 100

#define MAPSIZEX 16
#define MAPSIZEY 16

#define NORTH   0b00000001
#define SOUTH   0b00000010
#define EAST    0b00000100
#define WEST    0b00001000

#define VISITED 0b00010000




extern double  g_errP, g_errI, g_errD, g_lastErr;

extern double  sl_errP, sl_errI, sl_errD, sl_lastErr;
extern double  sr_errP, sr_errI, sr_errD, sr_lastErr;

extern double  wf_errP, wf_errI, wf_errD, wf_lastErr;

extern double  rot_errP, rot_errI, rot_errD, rot_lastErr;
extern bool rot_almost, rot_completed;
extern int rot_almost_count;

extern double d_errP, d_errI, d_errD, d_lastErr;
extern int d_almost_count;
extern bool d_almost, d_completed;

extern bool working,finishBehavior;

void wallFollowBehavior();
void gyroBehavior();
void speedBehavior(double speed,bool rotate);
void rotateBehavior(int degrees);
void distanceBehavior(int mm);
void resetErrors();
void labBehavior();
void mazeBehavior();
void setupMap();


extern int nextLabState;
extern int posX;
extern int posY;
extern byte absoluteOrientation;
extern byte mazemap[18][18];

    
#endif
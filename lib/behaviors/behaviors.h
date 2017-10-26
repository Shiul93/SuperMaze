#ifndef BEHAVIORS_H
#define BEHAVIORS_H
 
#define GYRO_KP 1.5
#define GYRO_KI 0.05
#define GYRO_KD 0.15

extern double  g_errP, g_errI, g_errD, g_lastErr;


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
#define ROT_KD 2.5
#define ROT_ALMOST 20


extern double  sl_errP, sl_errI, sl_errD, sl_lastErr;
extern double  sr_errP, sr_errI, sr_errD, sr_lastErr;

extern double  wf_errP, wf_errI, wf_errD, wf_lastErr;

extern double  rot_errP, rot_errI, rot_errD, rot_lastErr;
extern bool rot_almost, rot_completed;
extern int rot_almost_count;

void wallFollowBehavior();
void gyroBehavior();
void speedBehavior(double speed,bool rotate);
void rotateBehavior(int degrees);
    
#endif
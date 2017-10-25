#include "behaviors.h"
#include "TB6612.h"
#include "motors.h"
#include "encoders.h"
#include "utils.h"
#include "sensors.h"

double  g_errP, g_errI, g_errD, g_lastErr = 0;

double lastSpeedSetL, lastSpeedSetR = 0;
double  sl_errP, sl_errI, sl_errD, sl_lastErr = 0;
double  sr_errP, sr_errI, sr_errD, sr_lastErr = 0;

double  wf_errP, wf_errI, wf_errD, wf_lastErr = 0;

/** 
 * @brief  Gyroscope control behavior
 * @note   Oposes to rotation
 * @retval None
 */
void gyroBehavior(){
    int error = gyroArray[2];
    g_errP = error;
    g_errI = g_errI+error;
    g_errI = ((error*g_errI)<0) ? 0 : g_errI;
  
    g_errD = error-g_lastErr;
  
  
    g_lastErr = error;
    double pidErr = GYRO_KP*g_errP +GYRO_KI*g_errI + GYRO_KD*g_errD;
    int newerror = pidErr;
  
    if (abs(newerror) > 5){
  
      motorSpeed(RMOTOR, (error>0), abs(newerror));
      motorSpeed(LMOTOR, (error<=0), abs(newerror));
    }else{
      motorBrake(RLMOTOR);
    }
  
  }

/** 
 * @brief  Speed set controller
 * @note   
 * @param  speed: Objective speed
 * @param  rotate: if true, right wheel will be reversed
 * @retval None
 */
void speedBehavior(double speed,bool rotate){
  //Error L = objective speed - actual speed
  double errorL = speed>0 ?speed - speedL : - (abs(speed) - abs(speedL)) ;
  sl_errP = errorL;
  sl_errI = sl_errI+errorL;
  sl_errI = ((errorL*sl_errI)<0) ? 0 : sl_errI;
  sl_errD = errorL-sl_lastErr;
  
  //Save for integral component
  sl_lastErr = errorL;
  //Calculate pid error correction
  double pidErrL = SPEEDL_KP*sl_errP + SPEEDL_KI*sl_errI + SPEEDL_KD*sl_errD;
  double newerrorL = lastSpeedSetL +  pidErrL;
  lastSpeedSetL = newerrorL;

  //Error R = Actual L speed - Actual R speed /// SPEED CONTROL
  double errorR = speedL > 0 ? speedL - speedR : - (abs(speedL) - abs(speedR)) ;
  //Error R = L Ticks- R Ticks /// POSITION CONTROL
  //double errorR = encoderL - encoderR ;
  errorR = rotate ? -errorR : errorR;
  sr_errP = errorR;
  sr_errI = sr_errI+errorR;
  sr_errI = ((errorR*sr_errI)<0) ? 0 : sr_errI;
  sr_errD = errorR-sr_lastErr;



  sr_lastErr = errorR;
  double pidErrR = SPEEDR_KP*sr_errP + SPEEDR_KI*sr_errI + SPEEDR_KD*sr_errD;
  //pid_err_print = pidErrR;
  double newerrorR = lastSpeedSetR +  pidErrR;
  lastSpeedSetR = newerrorR;



    Serial.print("errorR ");
    Serial.println(errorR);
    
    motorSpeed(RMOTOR, (lastSpeedSetR>=0), abs(newerrorR));
    motorSpeed(LMOTOR, (newerrorL>=0), abs(newerrorL));

}


/** 
 * @brief  Wall follower behavior
 * @note   
 * @param  kp: P Constant
 * @param  ki: I Constant
 * @param  kd: D Constant
 * @retval None
 */
void wallFollowBehavior(){
  int error = distCL-distCR;
  wf_errP = error;
  wf_errI = abs(wf_errI+error)>1200 ? sign(wf_errI+error)*1200 : (wf_errI+error);
  wf_errI = ((error*wf_errI)<0) ? 0 : wf_errI;

  wf_errD = error-wf_lastErr;
  wf_errD = abs(error-wf_lastErr)>1200 ? sign(error-wf_lastErr)*1200 : (error-wf_lastErr);

  wf_lastErr = error;
  double pidErr = WALL_KP*wf_errP + WALL_KI*wf_errI + WALL_KD*wf_errD;
  int newerror = pidErr;
  //Serial.printf("eP: %i eI: %i eD: %i PID_Err: %i \n",errP,errI,errD,newerror);

  if (abs(newerror) > 5){

    motorSpeed(RMOTOR, (error<0), abs(newerror));
    motorSpeed(LMOTOR, (error>=0), abs(newerror));
    //motorSpeed(RMOTOR, true, newerror);
    //motorSpeed(LMOTOR, true, newerror);
  }else{
    motorBrake(RLMOTOR);
  }
}
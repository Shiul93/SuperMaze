#include "behaviors.h"
#include "TB6612.h"
#include "motors.h"
#include "encoders.h"
#include "utils.h"
#include "sensors.h"
#include "screen.h"

double  g_errP, g_errI, g_errD, g_lastErr = 0;

double lastSpeedSetL, lastSpeedSetR = 0;
double  sl_errP, sl_errI, sl_errD, sl_lastErr = 0;
double  sr_errP, sr_errI, sr_errD, sr_lastErr = 0;

double  wf_errP, wf_errI, wf_errD, wf_lastErr = 0;

double  rot_errP, rot_errI, rot_errD, rot_lastErr = 0;
int rot_almost_count = 0;
bool rot_almost, rot_completed = false;


double  d_errP, d_errI, d_errD, d_lastErr = 0;
int d_almost_count = 0;
bool d_almost, d_completed = false;

bool finishBehavior = true; 
bool working = false;

int labState = 0;
int nextLabState = 0;

//Absolute orientation on the map
byte absoluteOrientation = NORTH;

int posX = 1;
int posY = 1;

bool rotation = false;
int rotDir = 0;


byte mazemap[18][18] = {
  {0,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,0},
  {4,9,1,1,1,1,1,1,1,1,1,1,1,1,1,1,5,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,8},
  {4,10,2,2,2,2,2,2,2,2,2,2,2,2,2,2,6,8},
  {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},  
  
  };

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
      finishBehavior = true;
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

/** 
 * @brief  Rotation controller
 * @note   
 * @param  degrees: Objective position 
 * @param  angle: Actual position
 * @param  kp: P Constant
 * @param  ki: I Constant
 * @param  kd: D Constant
 * @retval None
 */

void rotateBehavior(int degrees){
  rot_completed = false;
  rot_almost = false;
  working = true;

  int error = readAngle()-degrees;
  rot_errP = error;
  rot_errI = rot_errI+error;
  rot_errI = ((error*rot_errI)<0) ? 0 : rot_errI;

  rot_errD = error-rot_lastErr;


  rot_lastErr = error;
  double pidErr = ROT_KP*rot_errP + ROT_KI*rot_errI + ROT_KD*rot_errD;
  int newerror = (abs(pidErr)> ROT_SPEED_LIMIT_LOW)? sign(pidErr)*ROT_SPEED_LIMIT_LOW : pidErr;
  newerror = (abs(pidErr)>ROT_SPEED_LIMIT_HIGH)? ROT_SPEED_LIMIT_HIGH : pidErr;


  if (abs(newerror) > 1){

    motorSpeed(RMOTOR, (error>0), abs(newerror));
    motorSpeed(LMOTOR, (error<0), abs(newerror));
    }else{
      if ((rot_almost_count > ROT_ALMOST)||(error==0)){
        rot_completed = true;
        rot_almost_count = 0;
        motorBrake(RLMOTOR);
        finishBehavior = true;
        working = false;
        
        

      }else{
        rot_almost_count++;
        motorSpeed(RMOTOR, (error>0), abs(newerror));
        motorSpeed(LMOTOR, (error<0), abs(newerror));
      }
  }
}

/** 
 * @brief  Distance set controller
 * @note   
 * @param  mm: Objective
 * @param  distance: Actual distance
 * @param  kp: P Constant
 * @param  ki: I Constant
 * @param  kd: D Constant
 * @retval None
 */
void distanceBehavior(int mm){
  d_completed = false;
  d_almost = false;
  working = true;
  

  int error = mm - readDistance();
  d_errP = error;
  d_errI = d_errI+error;
  d_errI = ((error*d_errI)<0) ? 0 : d_errI;

  d_errD = error-d_lastErr;


  d_lastErr = error;
  double pidErr = DIST_KP*d_errP + DIST_KI*d_errI + DIST_KD*d_errD;
  int newerror = (abs(pidErr)< DIST_SPEED_LIMIT_LOW)? sign(pidErr)*DIST_SPEED_LIMIT_LOW : pidErr;
  newerror = (abs(pidErr)>DIST_SPEED_LIMIT_HIGH)? DIST_SPEED_LIMIT_HIGH : pidErr;


  if (abs(newerror) > 3){

    motorSpeed(RMOTOR, (error>0), abs(newerror));
    motorSpeed(LMOTOR, (error>0), abs(newerror));
    }else{
      if (d_almost_count > DIST_ALMOST){
        d_completed = true;
        d_almost_count = 0;
        motorBrake(RLMOTOR);
        working = false;
        finishBehavior = true;
        

      }else{
        d_almost_count++;
        motorSpeed(RMOTOR, (error>0), abs(newerror));
        motorSpeed(LMOTOR, (error>0), abs(newerror));
      }
  }

}

/** 
 * @brief  Resets PID error
 * @note   
 * @retval None
 */
void resetErrors(){
  g_errP = 0; 
  g_errI = 0;
  g_errD = 0;

  sl_errP = 0; 
  sl_errI = 0;
  sl_errD = 0;
  sr_errP = 0; 
  sr_errI = 0;
  sr_errD = 0;

  wf_errP = 0; 
  wf_errI = 0;
  wf_errD = 0;

  d_errP = 0; 
  d_errI = 0;
  d_errD = 0;

  rot_errP = 0; 
  rot_errI = 0;
  rot_errD = 0;

  
}

void labBehavior(){
  
  int multiplier = 1;
  

  if (finishBehavior){
    Serial1.println("FINISH BEHAVIOR");
    encoderReset();
    resetErrors();    
    finishBehavior = false;
    
    labState = nextLabState;
    if (distCR>200){
      nextLabState = 1;
    }else if (distCL>200){
      nextLabState = 2;
    }else{
      multiplier = (distFL/180);
      nextLabState = 0;
    }
    if ( distFL < 100){
      labState = 1;
    }
    
  }else{
      switch(labState){
        Serial1.println("State");
        case 0:
          distanceBehavior(180);
          break;
        case 1:
          rotateBehavior(90);//LEFT
          break;
        case 2:
          rotateBehavior(-90);//RIGHT
          break;
        case 3:
          rotateBehavior(180);
          break;
        /*case 3:
          distanceBehavior(180*multiplier);
          break;*/
        
        
        default:
          finishBehavior = false;
          labState = 0;
        break;
      }
    
      
  }
}


void updateWalls(){
  mazemap[posX][posY]=  mazemap[posX][posY]|VISITED;  
  int frontDist = (distFL>0)&(distFR>0)? (distFL+distFR)/2 : 0;
  int frontCas = frontDist / 180;

  //Serial1.println(frontCas);
  switch (absoluteOrientation){
    case NORTH:
      mazemap[posX][posY+frontCas] =frontDist > 0? mazemap[posX][posY+frontCas]|NORTH : mazemap[posX][posY+frontCas];
      mazemap[posX][posY+frontCas+1] =frontDist > 0? mazemap[posX][posY+frontCas+1]|NORTH : mazemap[posX][posY+frontCas+1];
      
      if ((distCL < 200)&&(distCL > 0)){
        mazemap[posX][posY+1] = mazemap[posX][posY+1]|WEST;
        //mazemap[posX+1][posY+1 ] = mazemap[posX+1][posY+1]|EAST;
        
      }
      if ((distCR < 200)&&(distCR > 0)){
        mazemap[posX][posY+1] = mazemap[posX][posY+1]|EAST;
         
        //mazemap[posX-1][posY+1] = mazemap[posX-1][posY+1]|WEST;
        
      }
    break;
    case SOUTH:
      mazemap[posX][posY-frontCas] = frontDist > 0?mazemap[posX][posY-frontCas]|SOUTH : mazemap[posX][posY-frontCas];
      mazemap[posX][posY-frontCas-1] = frontDist > 0?mazemap[posX][posY-frontCas-1]|SOUTH : mazemap[posX][posY-frontCas-1];
      
      if ((distCL < 200)&&(distCL > 0)){
        mazemap[posX][posY-1] = mazemap[posX][posY-1]|EAST;
      }
      if ((distCR < 200)&&(distCR > 0)){
        mazemap[posX][posY-1] = mazemap[posX][posY-1]|WEST; 
      }
    break;
    case EAST:
      mazemap[posX+frontCas][posY] =frontDist > 0? mazemap[posX+frontCas][posY]|EAST : mazemap[posX+frontCas][posY];
      mazemap[posX+frontCas+1][posY] =frontDist > 0? mazemap[posX+frontCas+1][posY]|EAST : mazemap[posX+frontCas+1][posY];
      
      if ((distCL < 200)&&(distCL > 0)){
        mazemap[posX+1][posY] = mazemap[posX+1][posY]|NORTH;
      }
      if ((distCR < 200)&&(distCR > 0)){
        mazemap[posX+1][posY] = mazemap[posX+1][posY]|SOUTH; 
      }
    break;
    case WEST:
      mazemap[posX-frontCas][posY] = frontDist > 0?mazemap[posX-frontCas][posY]|WEST : mazemap[posX-frontCas][posY];
      mazemap[posX-frontCas-1][posY] = frontDist > 0?mazemap[posX-frontCas-1][posY]|WEST : mazemap[posX-frontCas-1][posY];
      
      if ((distCL < 200)&&(distCL > 0)){
        mazemap[posX-1][posY] = mazemap[posX-1][posY]|SOUTH;
      }
      if ((distCR < 200)&&(distCR > 0)){
        mazemap[posX-1][posY] = mazemap[posX-1][posY]|NORTH; 
      }    
    break;
  }
}


boolean checkFront(){
  return ((mazemap[posX][posY]&absoluteOrientation)==absoluteOrientation);
}

boolean checkRight(){
 switch (absoluteOrientation){
   case NORTH:
     if((mazemap[posX][posY]&EAST)==EAST){
       return true;
     }
   break;
   case SOUTH:
     if((mazemap[posX][posY]&WEST)==WEST){
       return true;
     }      
   break;
   case EAST:
     if((mazemap[posX][posY]&SOUTH)==SOUTH){
       return true;
     }      
   break;
   case WEST:
     if((mazemap[posX][posY]&NORTH)==NORTH){
       return true;
     }          
   break;
 }
 return false;
}

boolean checkLeft(){
 switch (absoluteOrientation){
   case NORTH:
     if((mazemap[posX][posY]&WEST)==WEST){
       return true;
     }
   break;
   case SOUTH:
     if((mazemap[posX][posY]&EAST)==EAST){
       return true;
     }      
   break;
   case EAST:
     if((mazemap[posX][posY]&NORTH)==NORTH){
       return true;
     }      
   break;
   case WEST:
     if((mazemap[posX][posY]&SOUTH)==SOUTH){
       return true;
     }          
   break;
 }
 return false;
}

void updatePos(){
  switch (absoluteOrientation){
    case NORTH:
      posY=posY+1;
    break;
    case SOUTH:
      posY=posY-1;    
    break;
    case EAST:
      posX=posX+1;     
    break;
    case WEST:
      posX=posX-1;          
    break;
  }
}
void updateOrientation(int rot){
  switch (absoluteOrientation){
    case NORTH:
        if (rot == 180){
          absoluteOrientation = SOUTH;
      }else if (rot == 90){
          absoluteOrientation = WEST;
      }else if (rot == -90) {
          absoluteOrientation = EAST;
      }
      break;
    case SOUTH:
      if (rot == 180){
        absoluteOrientation = NORTH;
      }else if (rot == 90){
        absoluteOrientation = EAST;
      }else if (rot == -90) {
        absoluteOrientation = WEST;
      }    
      break;
    case EAST:
      if (rot == 180){
        absoluteOrientation = WEST;        
      }else if (rot == 90){
        absoluteOrientation = NORTH;
      }else if (rot == -90) {
        absoluteOrientation = SOUTH;       
      }    
      break;
    case WEST:
      if (rot == 180){
        absoluteOrientation = EAST;        
      }else if (rot == 90){
        absoluteOrientation = SOUTH;
      }else if (rot == -90) {
        absoluteOrientation = NORTH;        
      }    
      break;
  }
}

int decideMovement(){
  if (rotation){
    rotation = false;
    updatePos();    
    return 0;
  }
  if (!checkRight()){
    rotation = true;
    rotDir = 2;
    updateOrientation(-90);
    return rotDir;
  }
  if (checkFront()&&!checkLeft()){
    rotation = true;
    rotDir = 1;
    updateOrientation(90);
    return rotDir;
  }
  if((checkLeft())&&(checkFront())&&(checkRight())){
    rotation = true;
    rotDir =3;
    updateOrientation(180);    
    return rotDir;
  }
  

  updatePos();
  return 0;


}

void printOrientation(){
  displayTile(mazemap[posX][posY]);
  Serial1.print("Position = ( ");
  Serial1.print(posX);
  Serial1.print(", ");
  Serial1.print(posY);
  Serial1.print(") Facing: ");
  switch (absoluteOrientation){
    case NORTH:
      Serial1.println("NORTH");
      break;
    case SOUTH:
      Serial1.println("SOUTH");
    
      break;
    case EAST:
    Serial1.println("EAST");
    
      break;
    case WEST:
    Serial1.println("WEST");
    
      break;
  }
  Serial1.print("CASBYTE: ");
  Serial1.println(mazemap[posX][posY]);

}

void mazeBehavior(){
  
  int multiplier = 1;
  

  if (finishBehavior){
    //Serial1.println("FINISH BEHAVIOR");
    encoderReset();
    resetErrors();    
    finishBehavior = false;
    printOrientation();
    updateWalls();
    labState = decideMovement();
    Serial1.print("Movement: ");
    Serial1.println(labState);
    
    
    
    
    
    
    
    
  }else{
      switch(labState){
        Serial1.println("State");
        case 0:
          distanceBehavior(180);
          break;
        case 1:
          rotation = true;
          rotateBehavior(90);
          break;
        case 2:
          rotation = true;
          rotateBehavior(-90);
          break;
        case 3:
          rotation = true;        
          rotateBehavior(180);
          break;
        /*case 3:
          distanceBehavior(180*multiplier);
          break;*/
        
        
        default:
          finishBehavior = false;
          labState = 0;
          break;
        
      }
    
      
  }
}

void setupMap(){
  mazemap[1][1] = SOUTH | EAST | WEST | VISITED;
  mazemap[2][1] = WEST;
  
}


/** 
 * @brief  Main file of the supermaze robot
 * @note   
 * @retval None
 */
#include "arduino.h"
#include "main.h"
#include <SPI.h>
#include <Wire.h>
#include "pinout.h"
#include "TB6612.h"
#include "motors.h"
#include "encoders.h"
#include "elapsedMillis.h"
#include "IntervalTimer.h"
#include <Adafruit_NeoPixel.h>
#include "screen.h"
#include <VL53L0X.h>
#include "sensors.h"
#include "behaviors.h"
#include "utils.h"




double  errP, errI, errD, lastErr, lastErrL, lastErrR = 0;

bool completed = true;

double pid_err_print;
double d_kp, d_ki, d_kd = 0; //Distance pid

double r_kp, r_ki, r_kd = 0; //Speed pid



int almost_checks = 5;
int almost_count = 0;
bool almost = false;










IntervalTimer sysTimer;
unsigned long sysTickCounts = 0;
elapsedMicros systickMicros;
unsigned int sysTickMilisPeriod =  100;
unsigned int sysTickSecond = 1000/sysTickMilisPeriod;

unsigned int longCount = 0;
unsigned int mediumCount = 0;
char screenShow = 'e';
char activeBehavior = 'o';

int hb = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, 8);
bool activateController = false;




void sysTick() {
  systickMicros =0;


  sysTickCounts++;
  mediumCount++;
  longCount++;


}

void setup(){



  d_kp = 0.3 ;
  d_ki = 0.05;
  d_kd = 0.15;


  r_kp = 20 ;
  r_ki = 0;
  r_kd = 2.5;


  Wire.begin();
  setupScreen();
  //setupPins();


  delay(1000);
  displayString("Starting Serial");
  Serial.begin(9600);
  delay(1000);

  Serial.println("----- BEGIN SERIAL -----");
  Serial1.begin(115200);
  Serial1.println("----------BEGIN BT---------");
  displayString("Starting interrupts");

  setupInterrupts();
  displayString("Starting Leds");

  strip.begin();
  strip.setPixelColor(0, 10, 0, 0);
  strip.setPixelColor(1, 10, 0, 0);
  strip.show();
  delay(500);
  strip.setPixelColor(0, 0, 10, 0);
  strip.setPixelColor(1, 0, 10, 0);
  strip.show();
  delay(500);
  strip.setPixelColor(0, 0, 0, 10);
  strip.setPixelColor(1, 0, 0, 10);
  strip.show();
  delay(500);
  strip.setPixelColor(0, 0, 0, 0);
  strip.setPixelColor(1, 0, 0, 0);
  strip.show();

  displayString("Starting Distance Sensors");

  setupDistanceSensors();
  delay(100);
  

  sysTimer.begin(sysTick, sysTickMilisPeriod);
  displayString("Starting IMU");
  startIMU();


  digitalWrite(buzzer_pin, HIGH);
  delay(100);
  digitalWrite(buzzer_pin, LOW);
  delay(100);
  digitalWrite(buzzer_pin, HIGH);
  delay(100);
  digitalWrite(buzzer_pin, LOW);
  delay(100);
  displayString("Startup OK");



  updateDistances();







}

/** 
 * @brief  Resets PID error
 * @note   
 * @retval None
 */
void resetErrors(){
  errP = 0; 
  errI = 0;
  errD = 0;
  lastErr = 0;
  lastErrL = 0;
  lastErrR = 0;
  
}

/** 
 * @brief  Updates encoder count
 * @note   
 * @retval None
 */
void encoderFun(){
  updateEncoderData();
  //Serial.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());
  //Serial1.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());

}
void loop(){


  if(sysTickCounts>= sysTickMilisPeriod)//32 ms
  {
    encoderFun();
    updateDistances();
    checkSpeed();
    readAccelValues();
    readGyroValues();

    sysTickCounts = 0;
    //followBehavior(kp,ki,kd);
    if (activeBehavior == 'g'){
      gyroBehavior();
    }else if (activeBehavior == 'w'){
    
      wallFollowBehavior();
    }else if (activeBehavior == 'd'){
      if (!completed){
        distanceBehavior(300,readDistance(),d_kp,d_ki,d_kd);
      }
    }else if (activeBehavior == 'r'){
      if (!completed){
        rotateBehavior(90,readAngle(),r_kp,r_ki,r_kd);
      }
    }else if (activeBehavior == 's'){
      resetErrors();
      speedBehavior(0.1,false);
    }else {
      motorCoast(RLMOTOR);
    }

    //encoderFun();



  }

  if(mediumCount>= 10*sysTickMilisPeriod)//320 ms
  {
    mediumCount = 0;
    if (screenShow == 'p'){
      //TODO Arreglar el mostrar PID
    }else  if (screenShow =='e'){
      displayENC(encoderR,encoderL,readDistance(), readAngle(), speedR, speedL);
    } else {
      displayGyro(gyroArray);
    }
  }

  if(longCount>= (1000/3)*sysTickMilisPeriod)//3200ms
  {
    longCount = 0;
    if (Serial1.available()){
      strip.setPixelColor(0, 0, 0, 10);
      strip.setPixelColor(1, 0, 0, 10);
      strip.show();
      delay(100);
      strip.setPixelColor(0, 0, 0, 0);
      strip.setPixelColor(1, 0, 0, 0);
      strip.show();
      String read = Serial1.readString(1);
      if (read =='p'){
        screenShow = 'p';
      }else if (read =='e'){
        screenShow = 'e';
      }else if (read =='a'){
        screenShow = 'a';
      }else if (read =='g'){
        screenShow = 'g';
      }else if (read =='r'){
        encoderReset();
      }
      else if (read =='b'){
        read = Serial1.readString(1);
        if (read =='g'){
          activeBehavior = 'g';
        }else if (read =='s'){
          activeBehavior = 's';
        }else if (read =='w'){
          activeBehavior = 'w';

        }else if (read =='o'){
          activeBehavior = 'o';

        }else if (read == 'd'){
          encoderReset();
          activeBehavior = 'd';
          completed = false;
        }else if (read == 'r'){
          encoderReset();
          resetErrors();
          activeBehavior = 'r';
          completed = false;
        }
      }
      Serial1.readString(2);
      Serial1.println("ACK");
      Serial1.println(read);

    }
    //printDistances();
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
void distanceBehavior(int mm,int distance, double kp,double ki,double kd){
  completed = false;
  almost = false;

  int error = mm - distance;
  errP = error;
  errI = errI+error;
  errI = ((error*errI)<0) ? 0 : errI;

  errD = error-lastErr;


  lastErr = error;
  double pidErr = kp*errP + ki*errI + kd*errD;
  pid_err_print = (abs(pidErr)> 30)? sign(pidErr)*30 : pidErr;
  int newerror = (abs(pidErr)> 30)? sign(pidErr)*30 : pidErr;
  newerror = (abs(pidErr)< 13)? sign(pidErr)*13 : pidErr;
  newerror = (abs(pidErr)>200)? 200 : pidErr;


  if (abs(newerror) > 3){

    motorSpeed(RMOTOR, (error>0), abs(newerror));
    motorSpeed(LMOTOR, (error>0), abs(newerror));
    }else{
      if (almost_count > almost_checks){
        completed = true;
        almost_count = 0;
        motorBrake(RLMOTOR);

      }else{
        almost_count++;
        motorSpeed(RMOTOR, (error>0), abs(newerror));
        motorSpeed(LMOTOR, (error>0), abs(newerror));
      }
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

void rotateBehavior(int degrees,int angle, double kp,double ki,double kd){
  completed = false;
  almost = false;

  int error = readAngle()-degrees;
  errP = error;
  errI = errI+error;
  errI = ((error*errI)<0) ? 0 : errI;

  errD = error-lastErr;


  lastErr = error;
  double pidErr = kp*errP + ki*errI + kd*errD;
  pid_err_print = (abs(pidErr)> 30)? sign(pidErr)*30 : pidErr;
  int newerror = (abs(pidErr)> 30)? sign(pidErr)*30 : pidErr;
  newerror = (abs(pidErr)< 17)? sign(pidErr)*17 : pidErr;
  newerror = (abs(pidErr)>100)? 100 : pidErr;


  if (abs(newerror) > 1){

    motorSpeed(RMOTOR, (error>0), abs(newerror));
    motorSpeed(LMOTOR, (error<0), abs(newerror));
    }else{
      if ((almost_count > almost_checks)||(error==0)){
        completed = true;
        almost_count = 0;
        motorBrake(RLMOTOR);

      }else{
        almost_count++;
        motorSpeed(RMOTOR, (error>0), abs(newerror));
        motorSpeed(LMOTOR, (error<0), abs(newerror));
      }
  }
}

/** 
 * @brief  Prints on the serial port the distance sensor reading
 * @note   
 * @retval None
 */
void printDistances(){
  Serial.print("FL: ");
  Serial.print(distFL);
  Serial.print(" CL: ");
  Serial.print(distCL);
  Serial.print(" CR: ");
  Serial.print(distCR);
  Serial.print(" FL: ");
  Serial.print(distFL);

  Serial.println();
}






  
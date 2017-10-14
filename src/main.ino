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




int distFL, distCL, distCR, distFR;

int errP, errI, errD, lastErr = 0;

double pid_err_print;
double kp, ki, kd = 0;


int meanIndex = 0;
int distFLarray[] = {0,0,0};
int distCLarray[] = {0,0,0};
int distCRarray[] = {0,0,0};
int distFRarray[] = {0,0,0};
float accelArray[] = {0,0,0};
float gyroArray[] = {0,0,0};


VL53L0X laserFL,laserCL, laserCR, laserFR;


IntervalTimer sysTimer;
unsigned long sysTickCounts = 0;
elapsedMicros systickMicros;
unsigned int sysTickMilisPeriod =  100;
unsigned int sysTickSecond = 1000/sysTickMilisPeriod;

unsigned int longCount = 0;
unsigned int mediumCount = 0;
char screenShow = 'p';

int hb = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, 8);
bool activateController = false;


int sign(int x) {
  return (x > 0) - (x < 0);
}
void updateDistances(){//1ms


  distFL = laserFL.readRangeContinuousMillimeters();
  distCL = laserCL.readRangeContinuousMillimeters();
  distCR = laserCR.readRangeContinuousMillimeters();
  distFR = laserFR.readRangeContinuousMillimeters();
  distFLarray[meanIndex] = (distFL<1200)? distFL : 0;
  distCLarray[meanIndex] = (distCL<1200)? distCL : 0;
  distCRarray[meanIndex] = (distCR<1200)? distCR : 0;
  distFRarray[meanIndex] = (distFR<1200)? distFR : 0;
  distFL = (distFLarray[0]+distFLarray[1]+distFLarray[2])/3;
  distCL = (distCLarray[0]+distCLarray[1]+distCLarray[2])/3;
  distCR = (distCRarray[0]+distCRarray[1]+distCRarray[2])/3;
  distFR = (distFRarray[0]+distFRarray[1]+distFRarray[2])/3;
  meanIndex++;
  meanIndex = meanIndex%3;



}

void sysTick() {
  systickMicros =0;


  sysTickCounts++;
  mediumCount++;
  longCount++;


}

void setup(){

  /*kp = 0.27;
  ki = 0.01;
  kd = 0.02;*///followBehavior

  kp = 0.45 ;
  ki = 0.05;
  kd = 0.05;
  Wire.begin();
  setupScreen();


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
  strip.setPixelColor(0, 10, 0, 10);
  strip.setPixelColor(1, 10, 0, 10);
  strip.show();
  displayString("Starting Distance Sensors");

  setupDistanceSensors();
  delay(100);
  laserFL.startContinuous();
  laserCL.startContinuous();
  laserCR.startContinuous();
  laserFR.startContinuous();

  sysTimer.begin(sysTick, sysTickMilisPeriod);
  displayString("Starting IMU");
  startIMU();



  delay(100);
  updateDistances();







}
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
    readAccelValues(accelArray);
    readGyroValues(gyroArray);

    sysTickCounts = 0;
    //followBehavior(kp,ki,kd);
    gyroBehavior(kp,ki,kd);

    //encoderFun();



  }

  if(mediumCount>= 10*sysTickMilisPeriod)//320 ms
  {
    mediumCount = 0;
    if (screenShow == 'p'){
      displayPID(kp*errP,ki*errI, kd*errD, pid_err_print);
    }else  if (screenShow =='e'){
      displayENC(encoderR,encoderL,readDistance(), readAngle());
    } else {
      displayGyro(gyroArray);
    }
  }

  if(longCount>= 1000*sysTickMilisPeriod)//3200ms
  {
  longCount = 0;
  if (Serial1.available()){
    String read = Serial1.readString(1);
    if (read =='p'){
      screenShow = 'p';
    }else if (read =='e'){
      screenShow = 'e';
    }else{
      //screenShow = 'a';

    }
    Serial1.println("ACK");

  }
  //printDistances();
  }



}

void followBehavior(double kp,double ki,double kd){
  int error = distCL-distCR;
  errP = error;
  errI = abs(errI+error)>1200 ? sign(errI+error)*1200 : (errI+error);
  errI = ((error*errI)<0) ? 0 : errI;

  errD = error-lastErr;
  errD = abs(error-lastErr)>1200 ? sign(error-lastErr)*1200 : (error-lastErr);

  lastErr = error;
  double pidErr = kp*errP + ki*errI + kd*errD;
  pid_err_print = pidErr;
  int newerror = pidErr;
  //Serial.printf("eP: %i eI: %i eD: %i PID_Err: %i \n",errP,errI,errD,newerror);

  if (abs(newerror) > 5){

  motorSpeed(RMOTOR, (error>0), abs(newerror));
  motorSpeed(LMOTOR, (error<=0), abs(newerror));
  //motorSpeed(RMOTOR, true, newerror);
  //motorSpeed(LMOTOR, true, newerror);
}else{
  motorSpeed(RLMOTOR, (error>0), 0);
}
}

void gyroBehavior(double kp,double ki,double kd){
  int error = -gyroArray[2];
  errP = error;
  errI = errI+error;
  errI = ((error*errI)<0) ? 0 : errI;

  errD = error-lastErr;


  lastErr = error;
  double pidErr = kp*errP + ki*errI + kd*errD;
  pid_err_print = pidErr;
  int newerror = pidErr;

  if (abs(newerror) > 5){

  motorSpeed(RMOTOR, (error>0), abs(newerror));
  motorSpeed(LMOTOR, (error<=0), abs(newerror));
}else{
  motorSpeed(RLMOTOR, (error>0), 0);
}

}


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


void setupDistanceSensors() {
  pinMode(sensor_rst1, OUTPUT);
  pinMode(sensor_rst2, OUTPUT);
  pinMode(sensor_rst3, OUTPUT);
  pinMode(sensor_rst4, OUTPUT);

  digitalWrite(sensor_rst1, LOW);
  digitalWrite(sensor_rst2, LOW);
  digitalWrite(sensor_rst3, LOW);
  digitalWrite(sensor_rst4, LOW);


  delay(500);
  //Wire.begin();

  pinMode(sensor_rst1, INPUT);
  delay(150);
  laserFL.init(true);
  delay(100);
  laserFL.setAddress((uint8_t)22);

  pinMode(sensor_rst2, INPUT);
  delay(150);
  laserCL.init(true);
  delay(100);
  laserCL.setAddress((uint8_t)25);

  pinMode(sensor_rst3, INPUT);
  delay(150);
  laserCR.init(true);
  delay(100);
  laserCR.setAddress((uint8_t)28);

  pinMode(sensor_rst4, INPUT);
  delay(150);
  laserFR.init(true);
  delay(100);
  laserFR.setAddress((uint8_t)31);

  laserFL.setTimeout(500);
  laserCL.setTimeout(500);
  laserCR.setTimeout(500);
  laserFR.setTimeout(500); }

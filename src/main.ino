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



VL53L0X laserFL,laserCL, laserCR, laserFR;

int distFL, distCL, distCR, distFR;

int meanIndex = 0;
int distFLarray[] = {0,0,0};
int distCLarray[] = {0,0,0};
int distCRarray[] = {0,0,0};
int distFRarray[] = {0,0,0};



IntervalTimer sysTimer;
unsigned long sysTickCounts = 0;
elapsedMicros systickMicros;
unsigned int sysTickMilisPeriod =  10;
unsigned int sysTickSecond = 1000/sysTickMilisPeriod;

int hb = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, 8);
bool activateController = false;

void updateDistances(){


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


}

void setup(){
  Wire.begin();

  delay(1000);
  Serial.begin(9600);
  delay(1000);

  Serial.println("----- BEGIN SERIAL -----");
  Serial1.begin(115200);
  Serial1.println("----------BEGIN BT---------");
  setupInterrupts();
  strip.begin();
  strip.setPixelColor(0, 10, 0, 10);
  strip.setPixelColor(1, 10, 0, 10);
  strip.show();

  setupScreen();
  setupDistanceSensors();
  delay(100);

  sysTimer.begin(sysTick, sysTickMilisPeriod);

  laserFL.startContinuous();
  laserCL.startContinuous();
  laserCR.startContinuous();
  laserFR.startContinuous();






}
void encoderFun(){
  updateEncoderData();
  Serial.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());
  Serial1.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());

}
void loop(){


  if(sysTickCounts>= 5*sysTickMilisPeriod)
  {
    //encoderFun();
    updateDistances();
    sysTickCounts = 0;
    followBehavior();
    //encoderFun();



  }



}

void followBehavior(){
  int error = distCL-distCR;
  Serial.println(error);
  if (abs(error)>100){
    motorSpeed(RMOTOR, (error>0), abs(error)/10);
  motorSpeed(LMOTOR, (error<=0), abs(error)/10);
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

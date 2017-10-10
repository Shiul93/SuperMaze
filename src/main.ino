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



VL53L0X laser1,laser2, laser3, laser4;






IntervalTimer sysTimer;
unsigned long sysTickCounts = 0;
elapsedMicros systickMicros;
unsigned int sysTickMilisPeriod = 100;
unsigned int sysTickSecond = 1000/sysTickMilisPeriod;

int hb = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, 8);
bool activateController = false;

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

  sysTimer.begin(sysTick, sysTickMilisPeriod*100);

  laser1.startContinuous();
  laser2.startContinuous();
  laser3.startContinuous();
  laser4.startContinuous();






}
void encoderFun(){
  updateEncoderData();
  Serial.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());
  Serial1.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());

}
void loop(){
  if(sysTickCounts > 0)                 {}
  if(sysTickCounts > 1*sysTickSecond)   {
  }
  if(sysTickCounts > 2*sysTickSecond)   {}
  if(sysTickCounts > 3*sysTickSecond)   {}
  if(sysTickCounts > 4*sysTickSecond)   {}
  //if(sysTickCounts > (4*sysTickSecond)) {sysTimer.end();}

  if(sysTickCounts>= 10)
  {

  }
  //  gyro.read();
  //  delay(100);
  if(sysTickCounts>= 10000){



  }


  Serial.print("FL: ");

  Serial.print(laser1.readRangeContinuousMillimeters());
  Serial.print(" CL: ");

  Serial.print(laser2.readRangeContinuousMillimeters());
  Serial.print(" CR: ");

  Serial.print(laser3.readRangeContinuousMillimeters());
  Serial.print(" FL: ");

  Serial.print(laser4.readRangeContinuousMillimeters());


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
  laser1.init(true);
  delay(100);
  laser1.setAddress((uint8_t)22);

  pinMode(sensor_rst2, INPUT);
  delay(150);
  laser2.init(true);
  delay(100);
  laser2.setAddress((uint8_t)25);

  pinMode(sensor_rst3, INPUT);
  delay(150);
  laser3.init(true);
  delay(100);
  laser3.setAddress((uint8_t)28);

  pinMode(sensor_rst4, INPUT);
  delay(150);
  laser4.init(true);
  delay(100);
  laser4.setAddress((uint8_t)31);

  laser1.setTimeout(500);
  laser2.setTimeout(500);
  laser3.setTimeout(500);
  laser4.setTimeout(500); }

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
#include "MadgwickAHRS.h"
#include <ADXL345.h>



IntervalTimer encoderRefresh;
IntervalTimer heartbeat;
IntervalTimer sysTimer;
unsigned long sysTickCounts = 0;
elapsedMicros systickMicros;
unsigned int sysTickMilisPeriod = 100;
unsigned int sysTickSecond = 1000/sysTickMilisPeriod;

int hb = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, 8);
ADXL345 adxl;
bool activateController = false;

void sysTick() {
  systickMicros =0;
  sysTickCounts++;

}

void setup(){

  Serial.begin(9600);
  Serial1.begin(115200);
  Serial1.println("----------BEGIN BT---------");
  setupInterrupts();
  adxl.powerOn();
  strip.begin();
  strip.setPixelColor(0, 10, 0, 0);
  strip.setPixelColor(1, 10, 0, 0);
  strip.show();

  sysTimer.begin(sysTick, sysTickMilisPeriod*100);



}
void encoderFun(){
  updateEncoderData();
  Serial.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());
  Serial1.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());

}
void loop(){
  if(sysTickCounts > 0)                 {}
  if(sysTickCounts > 1*sysTickSecond)   {}
  if(sysTickCounts > 2*sysTickSecond)   {}
  if(sysTickCounts > 3*sysTickSecond)   {}
  if(sysTickCounts > 4*sysTickSecond)   {}
  //if(sysTickCounts > (4*sysTickSecond)) {sysTimer.end();}

  if(sysTickCounts>= 10)
  {

  }
  //  gyro.read();
  //  delay(100);
  if(sysTickCounts>= 100000){
    Serial1.println(sysTickCounts);
    Serial.println(sysTickCounts);



  }
  int x,y,z;
  adxl.readAccel(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z

  // Output x,y,z values - Commented out
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z);
  delay(100);
}

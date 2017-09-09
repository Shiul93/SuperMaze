#include "arduino.h"
#include "main.h"
#include "pinout.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "screen.h"
//#include "profiler.h"
#include "speedController.h"

#include "elapsedMillis.h"
#include "IntervalTimer.h"
#include "TB6612.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>



IntervalTimer encoderRefresh;




unsigned long sysTickCounts = 0;
elapsedMicros systickMicros;

IntervalTimer sysTimer;
unsigned int sysTickMilisPeriod = 100;
unsigned int sysTickSecond = 1000/sysTickMilisPeriod;

bool activateController = false;

void sysTick() {
  systickMicros =0;
  //readSensors();
  updateEncoderData();
  if(activateController)
  {
    PIDcontroller();
    motorSpeedUpdate(RSpeedOUT,LSpeedOUT);
  }
  else
  {
  }
  sysTickCounts++;

}



void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  setupScreen();

  setupPins();
  digitalWrite(9, HIGH);
  delay(100);
  digitalWrite(9, LOW);
  delay(100);
  digitalWrite(9, HIGH);
  delay(100);
  digitalWrite(9, LOW);
  delay(100);
  digitalWrite(9, HIGH);
  delay(100);
  digitalWrite(9, LOW);
  delay(100);
  digitalWrite(9, HIGH);
  delay(100);
  digitalWrite(9, LOW);
  delay(100);

  //setupSensors();
  analogWriteResolution(16);         // analogWrite value 0 to 65535
  analogWriteFrequency(driver_PWMA, 732.4218); // Teensy 3.0 pin 4 also changes to 732.4218 kHz
  analogWriteFrequency(driver_PWMB, 732.4218);

  setupInterrupts();
  setupDistanceSensors();
  delay(500);


  delay(1000);
  sysTimer.begin(sysTick, sysTickMilisPeriod*100);



}


/*void loop() {
  /*
  if(sysTickCounts > 0)                 {RSpeedSet = 25; LSpeedSet = 25;}
  if(sysTickCounts > 1*sysTickSecond)   {RSpeedSet = 40; LSpeedSet = 40;}
  if(sysTickCounts > 2*sysTickSecond)   {RSpeedSet = 25; LSpeedSet = 25;}
  if(sysTickCounts > 3*sysTickSecond)   {RSpeedSet = 0; LSpeedSet = 0;}
  if(sysTickCounts > 4*sysTickSecond)   {RSpeedSet = 70; LSpeedSet = 70;}
  //if(sysTickCounts > (4*sysTickSecond)) {sysTimer.end();}

  if(sysTickCounts>= 10)
  {
    //serialDebug();
    //checkBattery();
    //Serial.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());
    int dist = readDist();

    if (dist >100){
      motorSpeed(RLMOTOR, true, dist*100);
    }else{
      if (dist < 90) {
        motorSpeed(RLMOTOR, false, dist*150);
      }else{
        motorCoast(RLMOTOR);
      }
    }
    //serialDebug();
    sysTickCounts = 0;
  }





}*/

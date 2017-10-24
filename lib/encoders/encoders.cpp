#include <Arduino.h>
#include "pinout.h"

int encoderR1Pin = encoderR1;
int encoderR2Pin = encoderR2;
int encoderL1Pin = encoderL1;
int encoderL2Pin = encoderL2;


//wheel diameter 32
//100.5309649 mm/ 1 wheel rev
//1 wheel rev /75.81 motor rev
//1 motor rev/10 encoder ticks
//100.5309649/758.1 mm/ encoder ticks = 0,1326091082 mm/encoder tick
//0.1326091082 mm/encoder tick
//7.540960147 encoder ticks/mm
//double countsPer360 = 2057.196;

double mmPerTick  = 0.085;
double ticksPerMM = 20;
double countsPer360 = 1500;

volatile long encoderR = 0;
volatile long encoderL = 0;

boolean encoderR1Set = false;
boolean encoderR2Set = false;
boolean encoderL1Set = false;
boolean encoderL2Set = false;

double encoderRTick = 0;
double encoderLTick = 0;

volatile long oldEncoderRin = 0;
volatile long oldEncoderLin = 0;
volatile long encoderRin = 0;
volatile long encoderLin = 0;

int frontSensorRightValue=0;
int frontSensorLeftValue=0;
int sideSensorRightValue=0;
int sideSensorLeftValue=0;

int distance_mm = 0;
int distance_deg = 0;






void encoderR1Count(){
  // Test transition
  encoderR1Set = digitalRead(encoderR1Pin) == HIGH;
  // and adjust counter + if A leads B
  encoderR += (encoderR1Set != encoderR2Set) ? +1 : -1;
}

void encoderR2Count(){
  // Test transition
  encoderR2Set = digitalRead(encoderR2Pin) == HIGH;
  // and adjust counter + if B follows A
  encoderR += (encoderR1Set == encoderR2Set) ? +1 : -1;
}

void encoderL1Count(){
  // Test transition
  encoderL1Set = digitalRead(encoderL1Pin) == HIGH;
  // and adjust counter + if A leads B
  encoderL += (encoderL1Set != encoderL2Set) ? -1 : +1;
}

void encoderL2Count(){
  // Test transition
  encoderL2Set = digitalRead(encoderL2Pin) == HIGH;
  // and adjust counter + if B follows A
  encoderL += (encoderL1Set == encoderL2Set) ? -1 : +1;
}

void setupInterrupts(){
  attachInterrupt(encoderR1Pin, encoderR1Count, CHANGE);
  attachInterrupt(encoderR2Pin, encoderR2Count, CHANGE);
  attachInterrupt(encoderL1Pin, encoderL1Count, CHANGE);
  attachInterrupt(encoderL2Pin, encoderL2Count, CHANGE);
}

void updateEncoderData(){
  oldEncoderRin = encoderRin;
  oldEncoderLin = encoderLin;
  encoderRin = encoderR;
  encoderLin = encoderL;
  encoderRTick = encoderRin - oldEncoderRin;
  encoderLTick = encoderLin - oldEncoderLin;

  distance_mm = mmPerTick * (encoderRin + encoderLin) / 2;
  distance_deg = (360/countsPer360) * (encoderRin - encoderLin) / 4;
  //distance_deg = COUNTS_TO_DEG((encoderRin - encoderLin) / 2);
}

void encoderReset(){
  encoderR1Set = false;
  encoderR2Set = false;
  encoderL1Set = false;
  encoderL2Set = false;

  encoderR = 0;
  encoderL = 0;

  oldEncoderRin = 0;
  oldEncoderLin = 0;

  encoderRin = 0;
  encoderLin = 0;

  encoderRTick = 0;
  encoderLTick = 0;

  distance_mm = 0;
  distance_deg = 0;
}

void setupSensors(){


}

int readDistance(){
  return (int)distance_mm;
}

int readAngle(){
  return distance_deg;
}

void checkBattery(){

}
int getAligmentError(){
  return encoderL - encoderR;
}

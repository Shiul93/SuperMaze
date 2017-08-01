//#include "Arduino.h"
#include "TB6612.h"

#include "motors.h"

#include "pinout.h"



TB6612 driver (driver_AIN1, driver_AIN2, driver_BIN1, driver_BIN2, driver_PWMA, driver_PWMB, driver_STBY);

void setupPWM(){
  analogWriteResolution(16);         // analogWrite value 0 to 65535
  analogWriteFrequency(driver_PWMA, 732.4218); // Teensy 3.0 pin 4 also changes to 732.4218 kHz
  analogWriteFrequency(driver_PWMB, 732.4218); // Teensy 3.0 pin 56,9,10,20,21,22,23 also changes to 732.4218 kHz
  motorCoast(RLMOTOR);
}


void motorSpeed(int motor, boolean direction, int speed) { //speed from 0 to 65535
  if (speed > ANALOG_W_RES) speed = ANALOG_W_RES;
  if (motor == RLMOTOR)
  {
    if (direction == 0)
    {
      driver.forward(speed,speed,0);
    }
    else //direction == 1
    {
      driver.reverse(speed, speed, 0);
    }
  }
  else if (motor == LMOTOR)
  {
    if (direction == 0)
    {
      driver.setSpeed(speed, motor);

    }
    else //direction == 1
    {
      driver.setSpeed(-speed, motor);

    }
  }
  else //motor == LMOTOR
  {
    if (direction == 0)
    {
      driver.setSpeed(speed, motor);

    }
    else //direction == 1
    {
      driver.setSpeed(-speed, motor);

    }
  }
}

void motorSpeedPercent(int motor, int speed) { //speed from -100 to 100
  int PWMvalue=0;
  PWMvalue = map(abs(speed),0,100,0,ANALOG_W_RES); //anything below 50 is very weak
  if (speed > 0)
    motorSpeed(motor,0,PWMvalue);
  else if (speed < 0)
    motorSpeed(motor,1,PWMvalue);
  else {
    motorCoast(motor);
  }
}

void motorSpeedUpdate(int Rspeed, int Lspeed) { //speed from -65535 to 65535
  if (Rspeed > 0)
    motorSpeed(RMOTOR,0,Rspeed);
  else if (Rspeed < 0)
    motorSpeed(RMOTOR,1,-Rspeed);
  else {
    motorBrake(RMOTOR);
  }

  if (Lspeed > 0)
    motorSpeed(LMOTOR,0,Lspeed);
  else if (Lspeed < 0)
    motorSpeed(LMOTOR,1,-Lspeed);
  else {
    motorBrake(LMOTOR);
  }
}

void motorBrake(int motor) {
  if (motor == RLMOTOR)
  {
    driver.brake(0);
  }
  else if (motor == RMOTOR)
  {
    //analogWrite(motorR1Pin,65535);
    //analogWrite(motorR2Pin,65535);
    driver.brake(0);

  }
  else //motor == LMOTOR
  {
    //analogWrite(motorL1Pin,65535);
    //analogWrite(motorL2Pin,65535);
    driver.brake(0);

  }
}

void motorCoast(int motor) {
  if (motor == RLMOTOR)
  {
    driver.coast(0);
  }
  else if (motor == RMOTOR)
  {
    //analogWrite(motorR1Pin,0);
    //analogWrite(motorR2Pin,0);
    driver.coast(0);

  }
  else //motor == LMOTOR
  {
    //analogWrite(motorL1Pin,0);
    //analogWrite(motorL2Pin,0);
    driver.coast(0);

  }
}

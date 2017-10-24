#include "pinout.h"

//Motor driver TB6612
int driver_PWMA = 22;
int driver_PWMB = 23;
int driver_AIN1 = 21;
int driver_AIN2 = 20;
int driver_BIN1 = 15;
int driver_BIN2 = 16;
int driver_STBY = 17;

//Buzzer
int buzzer_pin = 13;

//LED_BUILTIN
int led_pin = 13;



//Encoders
int encoderL1 = 2;
int encoderL2 = 3;
int encoderR1 = 5;
int encoderR2 = 4;

//TOF Sensors
int sensor_rst1 = 9;
int sensor_rst2 = 10;
int sensor_rst3 = 11;
int sensor_rst4 = 12;

//Buttons
int button1 = 6;
int button2 = 7;

int neopixel = 8;

void setupPins() {
  pinMode(driver_PWMA,OUTPUT);
  pinMode(driver_PWMB,OUTPUT);
  pinMode(driver_AIN1,OUTPUT);
  pinMode(driver_AIN2,OUTPUT);
  pinMode(driver_BIN1,OUTPUT);
  pinMode(driver_BIN2,OUTPUT);
  pinMode(driver_STBY,OUTPUT);

  pinMode(buzzer_pin,OUTPUT);

  pinMode(led_pin,OUTPUT);

  pinMode(encoderR1,OUTPUT);
  pinMode(encoderR2,OUTPUT);
  pinMode(encoderL1,OUTPUT);
  pinMode(encoderL2,OUTPUT);

  pinMode(sensor_rst1,OUTPUT);
  pinMode(sensor_rst2,OUTPUT);
  pinMode(sensor_rst3,OUTPUT);
  pinMode(sensor_rst4,OUTPUT);

  //pinMode(button1,INPUT_PULLUP);
  //pinMode(button2,INPUT_PULLUP);

  //digitalWrite(led_pin, HIGH);

  pinMode(neopixel,OUTPUT);




}

#include "sensors.h"
#include "pinout.h"

VL53L0X laser1,laser2, laser3, laser4;



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

int measureDistance() {
  return 0;
}

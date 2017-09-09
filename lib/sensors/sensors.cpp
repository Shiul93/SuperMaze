#include "sensors.h"
#include "pinout.h"


Adafruit_VL53L0X lox = Adafruit_VL53L0X();


void setupDistanceSensors() {
  lox.begin(0x30);
 }

int measureDistance() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data

  } else {
    return -1;
    }

  if (measure.RangeMilliMeter<1300){
    return measure.RangeMilliMeter;
  }
  else{
    return 0;
  }
}

#include "sensors.h"
#include "pinout.h"

MPU9250 imu;

void startIMU(){
  imu.MPU9250SelfTest(imu.SelfTest);
  imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
  imu.initMPU9250();

}

void readAccelValues(float * coords){
  imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values
  imu.getAres();

  // Now we'll calculate the accleration value into actual g's
  // This depends on scale being set
  coords[0] = (float)imu.accelCount[0]*imu.aRes; // - accelBias[0];
  coords[1] = (float)imu.accelCount[1]*imu.aRes; // - accelBias[1];
  coords[2] = (float)imu.accelCount[2]*imu.aRes; // - accelBias[2];
}

void readGyroValues(float * coords) {
  imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values
  imu.getGres();

  // Calculate the gyro value into actual degrees per second
  // This depends on scale being set
  coords[0] = (float)imu.gyroCount[0]*imu.gRes;
  coords[1] = (float)imu.gyroCount[1]*imu.gRes;
  coords[2] = (float)imu.gyroCount[2]*imu.gRes;
}

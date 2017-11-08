#include "sensors.h"
#include "pinout.h"

MPU9250 imu;
int meanIndex = 0;
int distFLarray[] = {0,0,0};
int distCLarray[] = {0,0,0};
int distCRarray[] = {0,0,0};
int distFRarray[] = {0,0,0};
float accelArray[]= {0,0,0};
float gyroArray[] = {0,0,0};
int distFL, distCL, distCR, distFR = 0;
VL53L0X laserFL, laserCL, laserCR, laserFR;

void startIMU(){
  
  imu.MPU9250SelfTest(imu.SelfTest);
  imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
  imu.initMPU9250();

}

void readAccelValues(){
  imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values
  imu.getAres();

  // Now we'll calculate the accleration value into actual g's
  // This depends on scale being set
  accelArray[0] = (float)imu.accelCount[0]*imu.aRes; // - accelBias[0];
  accelArray[1] = (float)imu.accelCount[1]*imu.aRes; // - accelBias[1];
  accelArray[2] = (float)imu.accelCount[2]*imu.aRes; // - accelBias[2];
}

void readGyroValues() {
  imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values
  imu.getGres();

  // Calculate the gyro value into actual degrees per second
  // This depends on scale being set
  gyroArray[0] = (float)imu.gyroCount[0]*imu.gRes;
  gyroArray[1] = (float)imu.gyroCount[1]*imu.gRes;
  gyroArray[2] = (float)imu.gyroCount[2]*imu.gRes;
}

/** 
 * @brief  Sets up the distance sensors
 * @note   
 * @retval None
 */
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
  laserFR.init(true);
  delay(100);
  laserFR.setAddress((uint8_t)22);

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
  laserFL.init(true);
  delay(100);
  laserFL.setAddress((uint8_t)31);

  laserFL.setTimeout(500);
  laserCL.setTimeout(500);
  laserCR.setTimeout(500);
  laserFR.setTimeout(500); 
  delay(100);
  laserFL.startContinuous();
  laserCL.startContinuous();
  laserCR.startContinuous();
  laserFR.startContinuous();
}

/** 
 * @brief  Updates the distance sensor measures
 * @note   
 * @retval None
 */
void updateDistances(){//1ms
  
  
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

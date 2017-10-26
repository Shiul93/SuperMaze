/** 
 * @brief  Main file of the supermaze robot
 * @note   
 * @retval None
 */
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
#include "sensors.h"
#include "behaviors.h"
#include "utils.h"



IntervalTimer sysTimer;
unsigned long sysTickCounts = 0;
elapsedMicros systickMicros;
unsigned int sysTickMilisPeriod =  100;
unsigned int sysTickSecond = 1000/sysTickMilisPeriod;

unsigned int longCount = 0;
unsigned int mediumCount = 0;
char screenShow = 'e';
char activeBehavior = 'o';

int hb = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, 8);
bool activateController = false;




void sysTick() {
  systickMicros =0;


  sysTickCounts++;
  mediumCount++;
  longCount++;


}

void setup(){
  Wire.begin();
  setupScreen();
  //setupPins();


  delay(1000);
  displayString("Starting Serial");
  Serial.begin(9600);
  delay(1000);

  Serial.println("----- BEGIN SERIAL -----");
  Serial1.begin(115200);
  Serial1.println("----------BEGIN BT---------");
  displayString("Starting interrupts");

  setupInterrupts();
  displayString("Starting Leds");

  strip.begin();
  strip.setPixelColor(0, 10, 0, 0);
  strip.setPixelColor(1, 10, 0, 0);
  strip.show();
  delay(500);
  strip.setPixelColor(0, 0, 10, 0);
  strip.setPixelColor(1, 0, 10, 0);
  strip.show();
  delay(500);
  strip.setPixelColor(0, 0, 0, 10);
  strip.setPixelColor(1, 0, 0, 10);
  strip.show();
  delay(500);
  strip.setPixelColor(0, 0, 0, 0);
  strip.setPixelColor(1, 0, 0, 0);
  strip.show();

  displayString("Starting Distance Sensors");

  setupDistanceSensors();
  delay(100);
  

  sysTimer.begin(sysTick, sysTickMilisPeriod);
  displayString("Starting IMU");
  startIMU();


  digitalWrite(buzzer_pin, HIGH);
  delay(100);
  digitalWrite(buzzer_pin, LOW);
  delay(100);
  digitalWrite(buzzer_pin, HIGH);
  delay(100);
  digitalWrite(buzzer_pin, LOW);
  delay(100);
  displayString("Startup OK");



  updateDistances();







}



/** 
 * @brief  Updates encoder count
 * @note   
 * @retval None
 */
void encoderFun(){
  updateEncoderData();
  //Serial.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());
  //Serial1.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());

}
void loop(){


  if(sysTickCounts>= sysTickMilisPeriod)//32 ms
  {
    encoderFun();
    updateDistances();
    checkSpeed();
    readAccelValues();
    readGyroValues();

    sysTickCounts = 0;
    //followBehavior(kp,ki,kd);
    if (activeBehavior == 'g'){
      gyroBehavior();
    }else if (activeBehavior == 'w'){
    
      wallFollowBehavior();
    }else if (activeBehavior == 'd'){
      if (!d_completed){
        distanceBehavior(300);
      }
    }else if (activeBehavior == 'r'){
      if (!rot_completed){
        rotateBehavior(90);
      }
    }else if (activeBehavior == 's'){
      resetErrors();
      speedBehavior(0.1,false);
    }else {
      motorCoast(RLMOTOR);
    }

    //encoderFun();



  }

  if(mediumCount>= 10*sysTickMilisPeriod)//320 ms
  {
    mediumCount = 0;
    if (screenShow == 'p'){
      //TODO Arreglar el mostrar PID
    }else  if (screenShow =='e'){
      displayENC(encoderR,encoderL,readDistance(), readAngle(), speedR, speedL);
    } else {
      displayGyro(gyroArray);
    }
  }

  if(longCount>= (1000/3)*sysTickMilisPeriod)//3200ms
  {
    longCount = 0;
    if (Serial1.available()){
      strip.setPixelColor(0, 0, 0, 10);
      strip.setPixelColor(1, 0, 0, 10);
      strip.show();
      delay(100);
      strip.setPixelColor(0, 0, 0, 0);
      strip.setPixelColor(1, 0, 0, 0);
      strip.show();
      String read = Serial1.readString(1);
      if (read =='p'){
        screenShow = 'p';
      }else if (read =='e'){
        screenShow = 'e';
      }else if (read =='a'){
        screenShow = 'a';
      }else if (read =='g'){
        screenShow = 'g';
      }else if (read =='r'){
        encoderReset();
      }
      else if (read =='b'){
        read = Serial1.readString(1);
        if (read =='g'){
          activeBehavior = 'g';
        }else if (read =='s'){
          activeBehavior = 's';
        }else if (read =='w'){
          activeBehavior = 'w';

        }else if (read =='o'){
          activeBehavior = 'o';

        }else if (read == 'd'){
          encoderReset();
          activeBehavior = 'd';
          d_completed = false;
        }else if (read == 'r'){
          encoderReset();
          resetErrors();
          activeBehavior = 'r';
          rot_completed = false;
        }
      }
      Serial1.readString(2);
      Serial1.println("ACK");
      Serial1.println(read);

    }
    //printDistances();
  }



}









/** 
 * @brief  Prints on the serial port the distance sensor reading
 * @note   
 * @retval None
 */
void printDistances(){
  Serial.print("FL: ");
  Serial.print(distFL);
  Serial.print(" CL: ");
  Serial.print(distCL);
  Serial.print(" CR: ");
  Serial.print(distCR);
  Serial.print(" FL: ");
  Serial.print(distFL);

  Serial.println();
}






  
#include "Arduino.h"

/*void loop(){
  Serial1.printf("Enc L: %i, Enc R: %i, Distance %i, Angle %i \n",encoderL,encoderR,readDistance(),readAngle());
  delay(100);
  Serial1.println("Test");

  int medida = (measureDistance());
  displayInt(medida);

}*/
String receivedText = "";

void loop() {
   if (Serial.available()) Serial1.write(Serial.read());
  while(Serial1.available() > 0) { // While there is more to be read, keep reading.
    receivedText += (char)Serial1.read();
    delay(10);
  }

  if(receivedText != "") Serial.println(receivedText);
  receivedText = "";
}

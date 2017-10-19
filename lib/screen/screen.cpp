#include "pinout.h"
#include "screen.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* clock=*/ A5, /* data=*/ A4, /* reset=*/ U8X8_PIN_NONE);

void setupScreen(){
  oled.setI2CAddress(0x78);
  oled.begin();
  oled.setFont(u8g2_font_helvR08_te);
  oled.firstPage();
  oled.drawStr(0,8,"------------OLED-------------");
  oled.nextPage();


}

void clearDisp(){
  oled.clearBuffer();
}
void displayInt(int data){
  enum {BufSize=9}; // If a is short use a smaller number, eg 5 or 6
  char buf[BufSize];
  snprintf (buf, BufSize, "%d", data);

  oled.setDrawColor(0);

  oled.drawBox(0, 9, 60, 30);
  oled.setDrawColor(1);

  oled.drawStr(0, 33, buf);
  oled.nextPage();


}

void printHeader(char header[]){
  clearDisp();
  oled.drawStr(0,8,header);

}



void displayPID(double ep, double ei, double ed, double error){
  enum {BufSize=32}; // If a is short use a smaller number, eg 5 or 6
  char data[6];
  char buf[BufSize];
  printHeader("--------------PID--------------");
  dtostrf(ep, 4, 2, data);

  snprintf (buf, BufSize, "P Error: %s", data);
  oled.drawStr(0,20,buf);
  dtostrf(ei, 4, 2, data);

  snprintf (buf, BufSize, "I Error: %s", data);
  oled.drawStr(0,30,buf);
  dtostrf(ed, 4, 2, data);

  snprintf (buf, BufSize, "D Error: %s", data);
  oled.drawStr(0,40,buf);
  dtostrf(error, 4, 2, data);

  snprintf (buf, BufSize, "PID Error: %s", data);
  oled.drawStr(0,50,buf);
  oled.nextPage();
}

void displayENC (int R, int L, int dist, int angle,double RSpeed, double LSpeed){
  enum {BufSize=32}; // If a is short use a smaller number, eg 5 or 6
  char data[6];

  char buf[BufSize];
  printHeader("-------------ENC-------------");
  snprintf (buf, BufSize, "R Count: %d", R);
  oled.drawStr(0,20,buf);
  snprintf (buf, BufSize, "L Count: %d", L);
  oled.drawStr(0,30,buf);
  snprintf (buf, BufSize, "Distance: %d mm", dist);
  oled.drawStr(0,40,buf);
  snprintf (buf, BufSize, "Angle: %d", angle);
  oled.drawStr(0,50,buf);

  dtostrf(RSpeed, 4, 2, data);
  char data2[6];

  dtostrf(LSpeed, 4, 2, data2);
  snprintf (buf, BufSize, "RS: %s             LS: %s", data, data2  );
  oled.drawStr(0,60,buf);
  oled.nextPage();
}

void displayAccel (float * coords){
  enum {BufSize=32}; // If a is short use a smaller number, eg 5 or 6
  char data[6];

  /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
  char buf[BufSize];
  printHeader("--------------ACC--------------");
  dtostrf(coords[0], 4, 2, data);

  snprintf (buf, BufSize, "X: %s", data);
  oled.drawStr(0,20,buf);
  dtostrf(coords[1], 4, 2, data);

  snprintf (buf, BufSize, "Y: %s", data);
  oled.drawStr(0,30,buf);
  dtostrf(coords[2], 4, 2, data);

  snprintf (buf, BufSize, "Z: %s", data);
  oled.drawStr(0,40,buf);

  oled.nextPage();
}

void displayGyro (float * coords){
  enum {BufSize=32}; // If a is short use a smaller number, eg 5 or 6
  char data[6];

  /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
  char buf[BufSize];
  printHeader("------------GYRO--------------");
  dtostrf(coords[0], 4, 2, data);

  snprintf (buf, BufSize, "X: %s", data);
  oled.drawStr(0,20,buf);
  dtostrf(coords[1], 4, 2, data);

  snprintf (buf, BufSize, "Y: %s", data);
  oled.drawStr(0,30,buf);
  dtostrf(coords[2], 4, 2, data);

  snprintf (buf, BufSize, "Z: %s", data);
  oled.drawStr(0,40,buf);

  oled.nextPage();
}

void displayString (char * s){

  printHeader("--------------STR--------------");

  oled.drawStr(0,20,s);


  oled.nextPage();
}

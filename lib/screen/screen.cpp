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

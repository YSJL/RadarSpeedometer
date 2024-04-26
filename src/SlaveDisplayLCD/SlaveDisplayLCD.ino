#include <LiquidCrystal.h>

typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;
binaryFloat recv;

LiquidCrystal lcd(4, 6, 10, 11, 12, 13);
float keep = -1.0;

void setup()
{
  Serial.begin(115200); // Use a faster baud rate so we can print faster
  while(!Serial);

  lcd.begin(16,1);
}

void loop() {
  while(Serial.read() != '\n');
  //lcd.clear();
  Serial.readBytes(recv.binary, 4);
  // Serial.println(recv.floatingPoint);
  static unsigned long timer = millis();
  if (millis() - timer >= 500) {
    timer = millis();
    // Serial.readBytes(recv.binary, 4);
    float temp = recv.floatingPoint;
    if (temp != keep) {
      keep = temp;
      lcd.clear();
      lcd.print(keep,2);
    }
  }
}
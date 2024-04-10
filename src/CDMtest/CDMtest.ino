#include "Radar.h"

Radar radar(A0, 8); //Radarmodule input at A5

void setup()
{
  pinMode(A0, INPUT); 
  Serial.begin(115200);
  radar.settolerance(20); //Set the tolerance from 0 to 100 percent
}

void loop()
{
  radar.measure(); //Read value from radar
  int radarvalue = radar.rtv(); //Get value (0 or 1)
  //Serial.println(radar.test());
  Serial.println(analogRead(A0));
  if(radarvalue == 1){
  Serial.println("Motion detected"); 
  }else{
  Serial.println("No motion"); 
  }
  delay(50);
}
/*
  Radar Basic Library 
  More information at: https://www.aeq-web.com/arduino-radar-motion-detector/
  Created by Alex E., September 17, 2017.
*/

#include "Radar.h"

Radar::Radar(int pin, int stan)
{
  _pin = pin;
  _stan = stan;
  _test = 0;
}

void Radar::settolerance(int tol){
  _tol = tol;
}

void Radar::measure()
{
  double per = ((double)_stan/100)*_tol;
  double sval = analogRead(_pin);
  _test = sval;
  if(sval > _stan+per || sval < _stan-per){
  _rtv = 1;
  }else{
  _rtv = 0;
  }
}

int Radar::rtv(){
  return _rtv;
}

double Radar::test(){
  return _test;
}
/*
  Radar Basic Library 
  More information at: https://www.aeq-web.com/arduino-radar-motion-detector/
  Created by Alex E., September 17, 2017.
*/

#ifndef Radar_h
#define Radar_h
#include "Arduino.h"

class Radar
{
  public:
    Radar(int pin, int stan);
    void settolerance(int tol);
    void measure();
    int tol;
    int rtv();
    double test();
  private:
    int _pin;
    int _tol;
    int _rtv;
    int _stan;
    double _test;
};
#endif


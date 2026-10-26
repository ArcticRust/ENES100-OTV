#ifndef Drivetrain_h
#define Drivetrain_h

#include "Arduino.h"

class Drivetrain {
    public:
      void setup();
      void forwards(float pow);
      void backwards(float pow);
      void left(float pow);
      void right(float pow);
      void rotate_left(float pow);
      void rotate_right(float pow);
      void stop();
};


#endif

C:\Users\chris\OneDrive\Documents\Arduino\ENES100 OTV\main_code\Drivetrain.cpp
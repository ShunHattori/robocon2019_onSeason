#pragma once

#include "Servo.h"
#include "mbed.h"

class ClothHold
{
public:
  /*
        サーボ用のPWM割り当てがされているピンx2
    */
  ClothHold(PinName servoRightPin, PinName servoLeftPin);

  void setFieldMode(int);


  void release(int);


  void grasp(int);

  void center(int);

  void half(int);

  void free(int);

private:
  Servo *servoRight, *servoLeft;
  PinName RightPin, LeftPin;
  bool blueModeFlag;
  const float workingPattern[4] = {
      0.0,
      1.0,
      0.65,
      0.35,
  };
};
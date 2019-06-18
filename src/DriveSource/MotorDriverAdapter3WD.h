#pragma once

#include "mbed.h"

class MotorDriverAdapter3WD
{
public:
  /*
    *   desc:   出力ピンを設定し初期化処理を行う
    *   param:  出力するピンx6
    *   return: none
    */
  MotorDriverAdapter3WD(PinName, PinName, PinName, PinName, PinName, PinName);

  /*
    *   desc:   入力されたPWMを元にピンに出力する
    *   param:  pwm[3](float)
    *   return: none
    */
  void apply(float pwm[3]);

private:
  PinName FCWPin, FCCWPin, BRCWPin, BRCCWPin, BLCWPin, BLCCWPin;
  PwmOut *FCR, *FCN, *BRR, *BRN, *BLR, *BLN;
  float duty[3];

  static const float RCconstant = 0.7;
  float prevPWM[3];

  float absFloat(float);
};
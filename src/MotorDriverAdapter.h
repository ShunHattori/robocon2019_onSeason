#pragma once

#include "mbed.h"

class MotorDriverAdapter
{
public:
  /*
    *   desc:   出力ピンを設定し初期化処理を行う
    *   param:  出力するピンx8
    *   return: none
    */
  MotorDriverAdapter(PinName, PinName, PinName, PinName, PinName, PinName, PinName, PinName);

  /*
    *   desc:   入力されたPWMを元にピンに出力する
    *   param:  pwm[4](float)
    *   return: none
    */
  void apply(float pwm[4]);

private:
  PinName FRCWPin, FRCCWPin, FLCWPin, FLCCWPin, BRCWPin, BRCCWPin, BLCWPin, BLCCWPin;
  PwmOut *FRR, *FRN, *FLR, *FLN, *BRR, *BRN, *BLR, *BLN;
  float duty[4];

  static const float RCconstant = 0.7;
  float prevPWM[4];

  float absFloat(float);
};
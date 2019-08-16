#pragma once

#include "mbed.h"

class MotorDriverAdapter4WD
{
public:
  /*
    *   desc:   出力ピンを設定し初期化処理を行う
    *   param:  出力するピンx8
    *   return: none
    */
  MotorDriverAdapter4WD(PinName, PinName, PinName, PinName, PinName, PinName, PinName, PinName);

  /*
    *   desc:   入力されたPWMを元にピンに出力する
    *   param:  pwm[4](double)
    *   return: none
    */
  void apply(double pwm[4]);

private:
  PinName FRCWPin, FRCCWPin, FLCWPin, FLCCWPin, BRCWPin, BRCCWPin, BLCWPin, BLCCWPin;
  PwmOut *FRR, *FRN, *FLR, *FLN, *BRR, *BRN, *BLR, *BLN;
  double duty[4];

  static const double RCconstant = 0.7;
  double prevPWM[4];

  double absdouble(double);
};
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
    *   param:  pwm[3](double)
    *   return: none
    */
  void apply(double pwm[3]);

  /*
    *   desc:   MDにかかる最大PWMを設定
    *   param:  maxPWM(double)
    *   return: none
    */
  void setMaxPWM(double maxPwm)
  {
    maxAllocateOutput = maxPwm;
  }

private:
  PinName FCWPin, FCCWPin, BRCWPin, BRCCWPin, BLCWPin, BLCCWPin;
  PwmOut *FCR, *FCN, *BRR, *BRN, *BLR, *BLN;
  double duty[3];

  static const double RCconstant = 0.7;
  double maxAllocateOutput;
  double prevPWM[3];

  double absdouble(double);
};
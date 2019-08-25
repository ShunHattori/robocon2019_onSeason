#pragma once

#include "math.h"
#include "mbed.h"

class OmniKinematics3WD
{
public:
  /*
    *   desc:   三輪ロボット,四輪ロボットのどちらかを指定するコンストラクタ
    *   param:  e.g. OmniKinematics3WD Omni;
    */
  OmniKinematics3WD() : maxAllocateOutput(0.2){};

  /*
    *   desc:   MDにかかる最大PWMを設定
    *   param:  maxPWM(double)
    *   return: none
    */
  void setMaxPWM(double maxPwm)
  {
    maxAllocateOutput = maxPwm;
  }

  /*
    *   desc:   ロボット全体の移動方向を指定し、MDにかけるPWMを取得
    *   param:  x,y,yaw方向の移動量(double) 駆動輪数に応じたdouble配列
    *   return: none(引数に代入)
    */
  void getOutput(double, double, double, double, double pwm[]);

private:
  double XVector, YVector, YawVector;
  double maxAllocateOutput;
};
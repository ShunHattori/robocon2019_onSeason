#pragma once

#include "mbed.h"
#include "math.h"

class OmniKinematics
{
public:
  /*
    *   desc:   三輪ロボット,四輪ロボットのどちらかを指定するコンストラクタ
    *   param:  ロボットの駆動輪数, e.g. OmniKinematics Omni(4);
    */
  OmniKinematics(int wheel) : wheelNumber(wheel), maxAllocateOutput(0.2){};

  /*
    *   desc:   MDにかかる最大PWMを設定(10~10000)
    *   param:  maxPWM(float)
    *   return: none
    */
  void setMaxPWM(float maxPwm)
  {
    maxAllocateOutput = maxPwm;
  }

  /*
    *   desc:   ロボット全体の移動方向を指定し、MDにかけるPWMを取得
    *   param:  x,y,yaw方向の移動量(float) 駆動輪数に応じたfloat配列
    *   return: none(引数に代入)
    */
  void getOutput(float x, float y, float yaw, float pwm[]);

private:
  int wheelNumber;
  float XVector, YVector, YawVector;
  float maxAllocateOutput;
};
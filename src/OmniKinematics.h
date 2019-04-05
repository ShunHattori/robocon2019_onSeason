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
  OmniKinematics(uint8_t wheel) : wheelNumber(wheel), maxAllocateOutput(0.2){};

  /*
    *   desc:   MDにかかる最大PWMを設定(10~95)
    *   param:  maxPWM(int)
    *   return: none
    */
  void setMaxPWM(int maxPwm)
  {
    maxAllocateOutput = maxPwm;
  }

  /*
    *   desc:   ロボット全体の移動方向を指定し、MDにかけるPWMを取得
    *   param:  x,y,yaw方向の移動量(int) 駆動輪数に応じたint配列
    *   return: none(引数に代入)
    */
  void getOutput(int x, int y, int yaw, int pwm[]);

private:
  uint8_t wheelNumber;
  int XVector, YVector, YawVector;
  int maxAllocateOutput;
};
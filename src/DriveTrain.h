#pragma once

#include "mbed.h"
#include "math.h"
#include "MWodometry.h"
#include "LocationManager.h"

class DriveTrain
{
public:
  DriveTrain(LocationManager<int> &lcmObj, MWodometry &X1, MWodometry &X2, MWodometry &Y1, int ENCATCdiff, int AllocateError, int DecreaseRadius)
  {
    LCM = &lcmObj;
    XAxis_1 = &X1;
    XAxis_2 = &X2;
    YAxis_1 = &Y1;
    encoderAttachDiff = ENCATCdiff;
    allocateError = AllocateError;
    decreaseRadius = DecreaseRadius;
    Max = 2000;
    Min = 500;
  }

  void update();
  void getCurrentLocation(int, int, double);
  bool getStats() { return stats; } //移動の進行状況を返す 1=移動完了,0=移動中
  int getXVector()
  {
    return Vec[0];
  }
  int getYVector()
  {
    return Vec[1];
  }
  int getYawVector()
  {
    return Vec[2];
  }

  void setMaxOutput(int max)
  {
    if (500 < max && max < 9700)
    {
      Max = max;
    }
  }

  void setMinOutput(int min)
  {
    if (100 < min && min < 3500)
    {
      Min = min;
    }
  }

private:
  LocationManager<int> *LCM;
  MWodometry *XAxis_1, *XAxis_2, *YAxis_1;

  /*
    *   current~ : 現在のロボットの位置を保存している
    *   target~  : ロボットの目標位置 
    *   error~   : 目標座標と現在位置の偏差
    *   stats    : フラグ(1=移動完了,0=移動中)
    *   encoderAttachDiff   :   同軸の計測輪取り付け距離
    *   XEncodedDistanceDiff:   同軸エンコーダ間の誤差　これからYAWを計算
    *   allocateError       :   停止地点の許容誤差
    *   decreaseRadius      :   減速開始判定円の半径
    */
  double currentX, currentY;
  double targetX, targetY, XEncodedDistanceDiff;
  double errorX, errorY, errorYaw;
  bool stats;
  double currentYaw, targetYaw;
  int encoderAttachDiff;
  int allocateError, decreaseRadius;

  int Vec[3];
  int Max, Min;

  float mapFloat(float value, float in_min, float in_max, float out_min, float out_max);
};
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
  }

  void update(float vec[]);
  void getCurrentLocation(int, int, float);
  bool getStats() { return stats; } //移動の進行状況を返す 1=移動完了,0=移動中

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
  int currentX, currentY;
  int targetX, targetY, XEncodedDistanceDiff;
  int errorX, errorY, errorYaw;
  bool stats;
  float currentYaw, targetYaw;
  float encoderAttachDiff;
  int allocateError, decreaseRadius;

  float mapFloat(float value, float in_min, float in_max, float out_min, float out_max);
};
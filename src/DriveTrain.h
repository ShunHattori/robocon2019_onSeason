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
    ConfirmStatsInitialFlag = 1;
  }

  /*
  * desc:   速度制御アルゴリズム
  * param:  none
  * return: none
  */
  void update();

  /*
  * desc:   移動の進行状況を取得する
  * param:  none
  * return: 1=移動完了,0=移動中
  */
  bool getStats() { return stats; }

  /*
  * desc:   各軸の出力を取得する
  * param:  none
  * return: 各軸の出力値(min~max内)(int)
  */
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

  /*
  * desc:   出力の最大値を設定する
  * param:  最大値(int)(500~9700)
  * return: none
  */
  void setMaxOutput(int max)
  {
    if (500 < max && max < 9700)
    {
      Max = max;
    }
  }

  /*
  * desc:   出力の最小値を設定する
  * param:  最小値(int)(100~3500)
  * return: none
  */
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
  Timer ConfirmStats;
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
  int xReachedMaxPWM, yReachedMaxPWM;
  bool ConfirmStatsInitialFlag;

  float mapFloat(float value, float in_min, float in_max, float out_min, float out_max);
};
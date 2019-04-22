#pragma once

#include "mbed.h"
#include "math.h"
#include "QEI.h"

class MWodometry
{
public:
  /*
  *   param:  Encoder型のオブジェクト,エンコーダの分解能
  *   return: none 
  */
  MWodometry(QEI &_obj, int res, float wheRad) : QEIobj(&_obj), encoderResolution(res), wheelRadius(wheRad){};               //コンストラクタ
  ~MWodometry(void);                                                                                                         //デストラクタ

  /*
  * desc:   エンコーダのパルスを距離(cm)データに変換したデータを取得する
  * param:  none
  * return: 計測輪移動距離(double)
  */
  double getDistance(void);

  /*
  * desc:   引数の距離データから現在位置を上書きする
  * param:  上書きする距離
  * return: none
  */
  void setDistance(double);

private:
  QEI *QEIobj;
  int encoderResolution;
  float wheelRadius;
};

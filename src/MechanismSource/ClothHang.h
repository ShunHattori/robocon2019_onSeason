#pragma once

#include "SensorSource\DebounceSwitch.h"
#include "mbed.h"

class ClothHang
{
public:
  /*
        PWM割り当てがされているピンx2
    */
  ClothHang(double *, DebounceSwitch &);

  /*
        現在の移動状況を取得する　(移動完了== 1, 移動中 == 0)
     */
  bool stats(void);

  /*
        ハンガーの展開長さを指定する
     */
  void setLength(int);

  /*
        エンコーダの値を入力するセッター
     */
  void setEncoderPulse(int);

  /*
        モータに印加するPWMを指定する
     */
  void setMaxPWM(double);

  /*
        追加された機構タスクを処理する
     */
  void update(void);

private:
  DebounceSwitch *upsideLimitSW;
  double *motorPWM;
  int16_t lenghtTarget, lenghtCurrent, lenghtBias;
  double pwm, aroundZeroPointPWM;
  bool isTargetLenghtAroundZeroPoint;
};
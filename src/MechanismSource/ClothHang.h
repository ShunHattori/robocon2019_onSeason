#pragma once

#include "SensorSource\DebounceSwitch.h"
#include "SensorSource\QEI.h"
#include "mbed.h"

class ClothHang
{
public:
  /*
        PWM割り当てがされているピンx2
    */
  ClothHang(double *, DebounceSwitch &, QEI &);

  /*
        現在の移動状況を取得する　(移動完了== 1, 移動中 == 0)
     */
  bool stats(void);

  void setTop();
  void setBottom();

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
  QEI *encoder;
  double *motorPWM;
  int16_t lenghtTarget, lenghtCurrent, lenghtBias;
  double pwm, aroundZeroPointPWM;
  bool isTargetLenghtAroundZeroPoint, flagTop, flagBottom, switchState;
};
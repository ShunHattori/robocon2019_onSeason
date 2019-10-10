#pragma once

#include "mbed.h"

class Peg
{
public:
  /*
        モータのpwm, 1回あたりの機構動作時間(s)
    */
  Peg(double pwm, double movingTime, double *variableToStore);

  /*
        機構タスク追加, コンストラクタで指定した秒数分モータを回転させる
     */
  void launch(void);

  void reload(void);
  
  void setPWM(double pwm)
  {
    maxPwm = pwm;
  }

  /*
        設定された秒数に応じてピンを制御する
     */
  void update(void);

private:
  Timer *timer;
  double maxPwm, timePerOnce, *outputPWM;
  bool launchFlag = false, reloadFlag = false,
       extendFlag = false, reduceFlag = false;
};
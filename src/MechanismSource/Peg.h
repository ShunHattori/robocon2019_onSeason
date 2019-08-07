#pragma once

#include "mbed.h"

class Peg
{
public:
    /*
        PWM割り当てがされているピンx2, モータのpwm, 1回あたりの機構動作時間(s)
    */
    Peg(PinName motorCW, PinName motorCCW, float pwm, float movingTime);

    /*
        機構タスク追加, コンストラクタで指定した秒数分モータを回転させる
     */
    void launch(void);

    void reload(void);

    /*
        設定された秒数に応じてピンを制御する
     */
    void update(void);

private:
    PwmOut *MotorCW, *MotorCCW;
    Timer *timer;
    float maxPwm, timePerOnce;
    bool launchFlag = false, reloadFlag = false,
         extendFlag = false, reduceFlag = false;
};
#pragma once

#include "mbed.h"
#include "Servo.h"

class ClothHold
{
public:
    /*
        サーボ用のPWM割り当てがされているピンx2
    */
    ClothHold(PinName servoRightPin, PinName servoLeftPin);

    /*
        機構タスク追加, 洗濯物を離す(サーボを０度に動かす)
     */
    void release(char);

    /*
        機構タスク追加, 洗濯物を掴む(サーボを-90,90に動かす)
     */
    void grasp(char);

    /*
        掴むハンドの移動状況を取得する(移動完了==1, 移動中==0)
     */
    bool stats(char);


private:
    Servo *servoRight, *servoLeft;
    bool runningModeRight, runningModeLeft; //1 = grasp, 0 = release
    const float workingPattern[2] = {
        1.0,
        0.0,
    };
};
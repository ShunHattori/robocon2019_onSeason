#pragma once

#include "Servo.h"
#include "mbed.h"

class ClothHold
{
public:
  /*
        サーボ用のPWM割り当てがされているピンx2
    */
  ClothHold(PinName servoRightPin, PinName servoLeftPin);

  void setFieldMode(int);

  /*
        機構タスク追加, 洗濯物を離す(サーボを０度に動かす)
     */
  void release(int);

  /*
        機構タスク追加, 洗濯物を掴む(サーボを-90,90に動かす)
     */
  void grasp(int);

  /*
    
     */
  void center(int);

  void half(int);

  /*
    サーボをフリー状態にする
    */
  void free(int);

  /*
        掴むハンドの移動状況を取得する(移動完了==1, 移動中==0)
     */
  bool stats(int);

private:
  Servo *servoRight, *servoLeft;
  PinName RightPin, LeftPin;
  bool runningModeRight, runningModeLeft; //1 = grasp, 0 = release
  bool blueModeFlag;
  const float workingPattern[4] = {
      0.0,
      1.0,
      0.65,
      0.35,
  };
};
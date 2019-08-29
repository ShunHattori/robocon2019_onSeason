#pragma once

#include "SensorSource\DebounceSwitch.h"
#include "mbed.h"

class RojarArm
{
public:
  /*
     */
  RojarArm(double *, DebounceSwitch &);

  /*
        現在の移動状況を取得する　(展開完了== 1, 展開中or展開不可 == 0)
     */
  bool stats(void);

  /*
        ロジャーアームの展開高さを設定する
     */
  void setHeight(int);

  /*
        エンコーダの値を入力するセッター
     */
  void setEncoderPulse(int);

  /*
        モータに印加するPWMを指定する
     */
  void setMaxPWM(double);

  /*
        現在の高さを取得する
     */
  int getHeight(void);

  /*
        展開高さに合わせた制御を行い、高さを調節する
     */
  void update(void);

private:
  DebounceSwitch *bottomSwitch;
  double *motorPWM;
  double userPWM, aroundZeroPointPWM;
  int heightTarget, heightCurrent;
  bool isTargetHeightAroundZeroPoint;
};
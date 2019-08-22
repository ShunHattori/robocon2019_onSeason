#pragma once

#include "mbed.h"

class RogerArm
{
public:
    /*
        PWM割り当てがされているピンx2
     */
    RogerArm(PinName motorCW, PinName motorCCW);

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
    void setMaxPWM(float);

    /*
        現在の高さを取得する
     */
    int getHeight(void);

    /*
        展開高さに合わせた制御を行い、高さを調節する
     */
    void update(void);

private:
    PwmOut *MotorCW, *MotorCCW;

    int16_t heightTarget, heightCurrent;
    float pwm;
};
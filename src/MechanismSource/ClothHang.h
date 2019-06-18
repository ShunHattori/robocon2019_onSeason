#pragma once

#include "mbed.h"

class ClothHang
{
public:
    /*
        PWM割り当てがされているピンx2, エンコーダ用の外部割込みピンx2
    */
    ClothHang(PinName motorCW, PinName motorCCW, PinName encA, PinName encB);

    /*
        現在の移動状況を取得する　(移動完了== 1, 移動中 == 0)
     */
    int stats(void);

    /*
        ハンガーの展開長さを指定する
     */
    void setLength(int);

    /*
        モータに印加するPWMを指定する
     */
    void setMaxPWM(float);

    /*
        追加された機構タスクを処理する
     */
    void update(void);

private:
    PwmOut *MotorCW, *MotorCCW;
    uint16_t lenghtTarget, lenghtCurrent;
    float pwm;
};
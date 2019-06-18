#include "OmniKinematics4WD.h"

void OmniKinematics4WD::getOutput(float x, float y, float yaw, float pwm[4])
{
    XVector = x;
    YVector = y;
    YawVector = yaw;

    //逆運動学を使って各軸の移動量からモータの回転方向・量を計算する
    pwm[0] = +XVector + YVector - YawVector;
    pwm[1] = +XVector - YVector + YawVector;
    pwm[2] = +XVector - YVector - YawVector;
    pwm[3] = +XVector + YVector + YawVector;

    //計算上の最大出力を求める
    float max = 0;
    for (int i = 0; i < 4; i++)
    {
        if (max < abs(pwm[i]))
            max = abs(pwm[i]);
    }

    //最大出力が許容最大出力を超えていたら再計算する
    float rate = 0;
    if (maxAllocateOutput < max)
    {
        rate = max / maxAllocateOutput;
        for (int i = 0; i < 4; i++)
        {
            pwm[i] = pwm[i] / rate;
        }
    }
}
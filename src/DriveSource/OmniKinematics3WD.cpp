#include "OmniKinematics3WD.h"

void OmniKinematics3WD::getOutput(float x, float y, float yaw, float pwm[3])
{
    XVector = x;
    YVector = y;
    YawVector = yaw;

    //逆運動学を使って各軸の移動量からモータの回転方向・量を計算する
    pwm[0] = +(XVector) + YawVector;
    pwm[1] = -(XVector / 2) - (YVector * 1.732 / 2) + YawVector;
    pwm[2] = -(XVector / 2) + (YVector * 1.732 / 2) + YawVector;

    //計算上の最大出力を求める
    float max = 0;
    for (int i = 0; i < 3; i++)
    {
        if (max < abs(pwm[i]))
            max = abs(pwm[i]);
    }

    //最大出力が許容最大出力を超えていたら再計算する
    float rate = 0;
    if (maxAllocateOutput < max)
    {
        rate = max / maxAllocateOutput;
        for (int i = 0; i < 3; i++)
        {
            pwm[i] = pwm[i] / rate;
        }
    }
}
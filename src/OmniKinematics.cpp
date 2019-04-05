#include "OmniKinematics.h"

void OmniKinematics::getOutput(int x, int y, int yaw, int pwm[4])
{
    XVector = x;
    YVector = y;
    YawVector = yaw;

    if (wheelNumber == 4)
    {
        pwm[0] = -XVector + YVector + YawVector;
        pwm[1] = +XVector + YVector - YawVector;
        pwm[2] = +XVector + YVector + YawVector;
        pwm[3] = -XVector + YVector - YawVector;
        int max = 0;
        for (int i = 0; i < 4; i++)
        {
            if (max < abs(pwm[i]))
                max = abs(pwm[i]);
        }
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
    else
    {
        pwm[0] = -XVector + 0 + YawVector;
        pwm[1] = +XVector / 2 - YVector * sqrt(3) / 2 + YawVector;
        pwm[2] = +XVector / 2 + YVector * sqrt(3) / 2 + YawVector;

        int max = 0;
        for (int i = 0; i < 3; i++)
        {
            if (max < abs(pwm[i]))
                max = abs(pwm[i]);
        }
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
}
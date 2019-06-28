#include "ClothHang.h"

ClothHang::ClothHang(PinName motorCW, PinName motorCCW)
{
    MotorCW = new PwmOut(motorCW);
    MotorCCW = new PwmOut(motorCCW);
    MotorCW->period_us(10);
    MotorCCW->period_us(10);
    lenghtCurrent = 0;
    lenghtTarget = 0;
}

bool ClothHang::stats(void)
{
    if ((lenghtTarget - 24) < lenghtCurrent && lenghtCurrent < (lenghtTarget + 24))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void ClothHang::setLength(int targetValue)
{
    lenghtTarget = targetValue;
}

void ClothHang::setEncoderPulse(int pulse)
{
    lenghtCurrent = pulse;
}

void ClothHang::setMaxPWM(float targetPwm)
{
    pwm = targetPwm;
}

void ClothHang::update(void)
{
    if ((lenghtTarget - 24) < lenghtCurrent && lenghtCurrent < (lenghtTarget + 24))
    {
        MotorCW->write(0);
        MotorCCW->write(0);
    }
    else
    {
        if (lenghtCurrent < lenghtTarget)
        {
            MotorCW->write(pwm);
            MotorCCW->write(0);
        }
        else if (lenghtCurrent > lenghtTarget)
        {
            MotorCW->write(0);
            MotorCCW->write(pwm);
        }
    }
}
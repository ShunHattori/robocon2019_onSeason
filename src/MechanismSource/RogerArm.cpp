#include "RogerArm.h"

RogerArm::RogerArm(PinName motorCW, PinName motorCCW)
{
    MotorCW = new PwmOut(motorCW);
    MotorCCW = new PwmOut(motorCCW);
    MotorCW->period_us(10);
    MotorCCW->period_us(10);
}

bool RogerArm::stats(void)
{
    if ((heightTarget - 24) < heightCurrent && heightCurrent < (heightTarget + 24))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void RogerArm::setHeight(int targetValue)
{
    heightTarget = targetValue;
}

void RogerArm::setEncoderPulse(int pulse)
{
    heightCurrent = pulse;
}

void RogerArm::setMaxPWM(float targetPWM)
{
    pwm = targetPWM;
}

int RogerArm::getHeight(void)
{
    return heightCurrent;
}

void RogerArm::update(void)
{
    if ((heightCurrent - 24) < heightTarget && heightTarget < (heightCurrent + 24))
    {
        MotorCW->write(0);
        MotorCCW->write(0);
    }
    else
    {
        if (heightTarget < heightCurrent)
        {
            MotorCW->write(pwm);
            MotorCCW->write(0);
        }
        else if (heightTarget > heightCurrent)
        {
            MotorCW->write(0);
            MotorCCW->write(pwm);
        }
    }
}
#include "MotorDriverAdapter3WD.h"

MotorDriverAdapter3WD::MotorDriverAdapter3WD(PinName Fcw, PinName Fccw, PinName BRcw, PinName BRccw, PinName BLcw, PinName BLccw)
{
    FCWPin = Fcw;
    FCCWPin = Fccw;
    BRCWPin = BRcw;
    BRCCWPin = BRccw;
    BLCWPin = BLcw;
    BLCCWPin = BLccw;

    FCR = new PwmOut(FCWPin);
    FCN = new PwmOut(FCCWPin);
    BRR = new PwmOut(BRCWPin);
    BRN = new PwmOut(BRCCWPin);
    BLR = new PwmOut(BLCWPin);
    BLN = new PwmOut(BLCCWPin);

    FCR->period_us(50);
    FCN->period_us(50);
    BRR->period_us(50);
    BRN->period_us(50);
    BLR->period_us(50);
    BLN->period_us(50);

    for (int i = 0; i < 3; i++)
    {
        prevPWM[i] = 0;
    }
}

float MotorDriverAdapter3WD::absFloat(float ver)
{
    if (ver < 0)
    {
        return -ver;
    }
    else
    {
        return ver;
    }
}

void MotorDriverAdapter3WD::apply(float pwm[3])
{
    for (int i = 0; i < 3; i++)
    {
        pwm[i] = RCconstant * prevPWM[i] + (1 - RCconstant) * pwm[i];
        prevPWM[i] = pwm[i];
    }

    duty[0] = pwm[0];
    duty[1] = pwm[1];
    duty[2] = pwm[2];

    if (0 < duty[0])
    {
        FCR->write((absFloat(duty[0])));
        FCN->write(0);
    }
    else if (duty[0] < 0)
    {
        FCR->write(0);
        FCN->write(absFloat(duty[0]));
    }
    else
    {
        FCR->write(0);
        FCN->write(0);
    }

    if (0 < duty[1])
    {
        BRR->write(absFloat(duty[1]));
        BRN->write(0);
    }
    else if (duty[1] < 0)
    {
        BRR->write(0);
        BRN->write(absFloat(duty[1]));
    }
    else
    {
        BRR->write(0);
        BRN->write(0);
    }

    if (0 < duty[2])
    {
        BLR->write(absFloat(duty[2]));
        BLN->write(0);
    }
    else if (duty[2] < 0)
    {
        BLR->write(0);
        BLN->write(absFloat(duty[2]));
    }
    else
    {
        BLR->write(0);
        BLN->write(0);
    }
}
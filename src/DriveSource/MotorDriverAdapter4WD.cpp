#include "MotorDriverAdapter4WD.h"

MotorDriverAdapter4WD::MotorDriverAdapter4WD(PinName FRcw, PinName FRccw, PinName FLcw, PinName FLccw, PinName BRcw, PinName BRccw, PinName BLcw, PinName BLccw)
{
    FRCWPin = FRcw;
    FRCCWPin = FRccw;
    FLCWPin = FLcw;
    FLCCWPin = FLccw;
    BRCWPin = BRcw;
    BRCCWPin = BRccw;
    BLCWPin = BLcw;
    BLCCWPin = BLccw;

    FRR = new PwmOut(FRCWPin);
    FRN = new PwmOut(FRCCWPin);
    FLR = new PwmOut(FLCWPin);
    FLN = new PwmOut(FLCCWPin);
    BRR = new PwmOut(BRCWPin);
    BRN = new PwmOut(BRCCWPin);
    BLR = new PwmOut(BLCWPin);
    BLN = new PwmOut(BLCCWPin);
    FRR->period_us(50);
    FRN->period_us(50);
    FLR->period_us(50);
    FLN->period_us(50);
    BRR->period_us(50);
    BRN->period_us(50);
    BLR->period_us(50);
    BLN->period_us(50);
    for (int i = 0; i < 4; i++)
    {
        prevPWM[i] = 0;
    }
}

float MotorDriverAdapter4WD::absFloat(float ver)
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

void MotorDriverAdapter4WD::apply(float pwm[4])
{
    for (int i = 0; i < 4; i++)
    {
        pwm[i] = RCconstant * prevPWM[i] + (1 - RCconstant) * pwm[i];
        prevPWM[i] = pwm[i];
    }

    duty[0] = pwm[0];
    duty[1] = pwm[1];
    duty[2] = pwm[2];
    duty[3] = pwm[3];

    if (0 < duty[0])
    {
        FRR->write((absFloat(duty[0])));
        FRN->write(0);
    }
    else if (duty[0] < 0)
    {
        FRR->write(0);
        FRN->write(absFloat(duty[0]));
    }
    else
    {
        FRR->write(0);
        FRN->write(0);
    }

    if (0 < duty[1])
    {
        FLR->write(absFloat(duty[1]));
        FLN->write(0);
    }
    else if (duty[1] < 0)
    {
        FLR->write(0);
        FLN->write(absFloat(duty[1]));
    }
    else
    {
        FLR->write(0);
        FLN->write(0);
    }

    if (0 < duty[2])
    {
        BRR->write(absFloat(duty[2]));
        BRN->write(0);
    }
    else if (duty[2] < 0)
    {
        BRR->write(0);
        BRN->write(absFloat(duty[2]));
    }
    else
    {
        BRR->write(0);
        BRN->write(0);
    }

    if (0 < duty[3])
    {
        BLR->write(absFloat(duty[3]));
        BLN->write(0);
    }
    else if (duty[3] < 0)
    {
        BLR->write(0);
        BLN->write(absFloat(duty[3]));
    }
    else
    {
        BLR->write(0);
        BLN->write(0);
    }
}
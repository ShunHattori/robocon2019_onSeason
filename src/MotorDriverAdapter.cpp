#include "MotorDriverAdapter.h"

MotorDriverAdapter::MotorDriverAdapter(PinName FRcw, PinName FRccw, PinName FLcw, PinName FLccw, PinName BRcw, PinName BRccw, PinName BLcw, PinName BLccw)
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
    FRR->period_us(30);
    FRN->period_us(30);
    FLR->period_us(30);
    FLN->period_us(30);
    BRR->period_us(30);
    BRN->period_us(30);
    BLR->period_us(30);
    BLN->period_us(30);
}

void MotorDriverAdapter::apply(float pwm[4])
{
    if (0 < pwm[0])
    {
        FRR->write(pwm[0]);
        FRN->write(0);
    }
    else if (pwm[0] < 0)
    {
        FRR->write(0);
        FRN->write(pwm[0]);
    }
    else
    {
        FRR->write(0);
        FRN->write(0);
    }

    if (0 < pwm[0])
    {
        FLR->write(pwm[0]);
        FLN->write(0);
    }
    else if (pwm[0] < 0)
    {
        FLR->write(0);
        FLN->write(pwm[0]);
    }
    else
    {
        FLR->write(0);
        FLN->write(0);
    }

    if (0 < pwm[0])
    {
        BRR->write(pwm[0]);
        BRN->write(0);
    }
    else if (pwm[0] < 0)
    {
        BRR->write(0);
        BRN->write(pwm[0]);
    }
    else
    {
        BRR->write(0);
        BRN->write(0);
    }
    
    if (0 < pwm[0])
    {
        BLR->write(pwm[0]);
        BLN->write(0);
    }
    else if (pwm[0] < 0)
    {
        BLR->write(0);
        BLN->write(pwm[0]);
    }
    else
    {
        BLR->write(0);
        BLN->write(0);
    }
}
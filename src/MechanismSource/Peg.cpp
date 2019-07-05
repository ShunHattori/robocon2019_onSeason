#include "Peg.h"

Peg::Peg(PinName motorCW, PinName motorCCW, float pwm = 0.5, float movingTime = 0.5)
{
    MotorCW = new PwmOut(motorCW);
    MotorCCW = new PwmOut(motorCCW);
    MotorCW->period_us(50);
    MotorCCW->period_us(50);

    timer = new Timer();
    timer->start();

    maxPwm = pwm;
    timePerOnce = movingTime;
}

void Peg::launch(void)
{
    if (taskFlag == 0)
    {
        taskFlag = 1;
        timer->reset();
    }
}

void Peg::update(void)
{
    if (taskFlag)
    {
        if (timer->read() < timePerOnce)
        {
            extendFlag = 1;
            reduceFlag = 0;
        }
        else if (extendFlag && timer->read() > timePerOnce)
        {
            extendFlag = 0;
            reduceFlag = 1;
        }
        else if (reduceFlag && timer->read() > (timePerOnce * 2) + 0.05){
            extendFlag = 0;
            reduceFlag = 0;
            taskFlag = 0;
        }
    }
    if (extendFlag)
    {
        MotorCW->write(maxPwm);
        MotorCCW->write(0);
    }
    else if (reduceFlag)
    {
        MotorCW->write(0);
        MotorCCW->write(maxPwm);
    }
    else
    {
        MotorCW->write(0);
        MotorCCW->write(0);
    }
}
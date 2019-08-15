#include "RojarArm.h"

RojarArm::RojarArm(PinName motorCW, PinName motorCCW)
{
  MotorCW = new PwmOut(motorCW);
  MotorCCW = new PwmOut(motorCCW);
  MotorCW->period_us(40);
  MotorCCW->period_us(40);
  heightCurrent = 0;
  heightTarget = 0;
}

bool RojarArm::stats(void)
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

void RojarArm::setHeight(int targetValue)
{
  heightTarget = targetValue;
}

void RojarArm::setEncoderPulse(int pulse)
{
  heightCurrent = pulse;
}

void RojarArm::setMaxPWM(float targetPWM)
{
  pwm = targetPWM;
}

int RojarArm::getHeight(void)
{
  return heightCurrent;
}

void RojarArm::update(void)
{
  if ((heightTarget - 24) < heightCurrent && heightCurrent < (heightTarget + 24))
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
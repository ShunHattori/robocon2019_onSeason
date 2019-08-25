#include "RojarArm.h"

RojarArm::RojarArm(double* variableToStore)
{
  motorPWM = variableToStore;
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
    motorPWM[0] = 0;
    motorPWM[1] = 0;
  }
  else
  {
    if (heightTarget < heightCurrent)
    {
      motorPWM[0] = pwm;
      motorPWM[1] = 0;
    }
    else if (heightTarget > heightCurrent)
    {
      motorPWM[0] = 0;
      motorPWM[1] = pwm;
    }
  }
}
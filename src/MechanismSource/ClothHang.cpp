#include "ClothHang.h"

ClothHang::ClothHang(double* variableToStore)
{
  motorPWM = variableToStore;
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
    motorPWM[0] = 0;
    motorPWM[1] = 0;
  }
  else
  {
    if (lenghtCurrent < lenghtTarget)
    {
      motorPWM[0] = pwm;
      motorPWM[1] = 0;
    }
    else if (lenghtCurrent > lenghtTarget)
    {
      motorPWM[0] = 0;
      motorPWM[1] = pwm;
    }
  }
}
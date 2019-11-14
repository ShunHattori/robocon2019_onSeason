#include "ClothHang.h"

ClothHang::ClothHang(double* variableToStore, DebounceSwitch& mySwitch, QEI& myEncoder)
{
  upsideLimitSW = &mySwitch;
  encoder = &myEncoder;
  motorPWM = variableToStore;
  lenghtCurrent = 0;
  lenghtTarget = 0;
  lenghtBias = 0;
  isTargetLenghtAroundZeroPoint = 1;
  aroundZeroPointPWM = 0.20;
}

bool ClothHang::stats(void)
{
  if ((lenghtTarget - 120) < lenghtCurrent && lenghtCurrent < (lenghtTarget + 15))
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
  if (!targetValue)
  {
    isTargetLenghtAroundZeroPoint = 1;
  }
  else
    isTargetLenghtAroundZeroPoint = 0;
  lenghtTarget = flag ? targetValue + 100 : targetValue;
}

void ClothHang::setEncoderPulse(int pulse)
{
  lenghtCurrent = pulse;
}

void ClothHang::setMaxPWM(double targetPwm)
{
  pwm = targetPwm;
}

void ClothHang::update(void)
{
  upsideLimitSW->update();
  if (upsideLimitSW->stats() && lenghtTarget > 500)
  {
    encoder->set(lenghtTarget, 0);
    flag = 1;
    lenghtCurrent = lenghtTarget;
  }

  if ((lenghtTarget - 15) < lenghtCurrent && lenghtCurrent < (lenghtTarget + 15))
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
      motorPWM[1] = pwm - 0.1;
    }
  }
  if (!isTargetLenghtAroundZeroPoint)
  { //目標停止位置が０
    return;
  }
  if (!(lenghtCurrent < 200))
  { //現在の高さが200パルスよりも小さいか？
    return;
  }
  if (0 < lenghtCurrent)
  {
    motorPWM[0] = 0;
    motorPWM[1] = aroundZeroPointPWM;
  }
  if ((lenghtTarget - 15) < lenghtCurrent && lenghtCurrent < (lenghtTarget + 15))
  {
    motorPWM[0] = 0;
    motorPWM[1] = 0;
  }
}
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
  motorPWM[0] = 0;
  motorPWM[1] = 0;
  flagTop = 0;
  flagBottom = 0;
  switchState = 0;
}

bool ClothHang::stats(void)
{
  if (flagTop)
  {
    if (switchState)
    {
      return 1;
    }
  }
  if (flagBottom)
  {
    if (-300 < lenghtCurrent && lenghtCurrent < 300)
    {
      return 1;
    }
  }
  return 0;
}

void ClothHang::setTop()
{
  flagTop = 1;
  flagBottom = 0;
}

void ClothHang::setBottom()
{
  encoder->set(850, 0);
  flagTop = 0;
  flagBottom = 1;
  timer.start();
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
  switchState = upsideLimitSW->stats();

  if (flagTop)
  {
    motorPWM[0] = pwm;
    motorPWM[1] = 0;
  }
  else if (flagBottom)
  {
    motorPWM[0] = 0;
    motorPWM[1] = 0.4;
  }
  if (lenghtCurrent < 0 && flagBottom)
  {
    motorPWM[0] = 0.4;
    motorPWM[1] = 0;
  }
  if (-50 < lenghtCurrent && lenghtCurrent < 50 && flagBottom)
  {
    motorPWM[0] = 0;
    motorPWM[1] = 0;
  }

  if (switchState && !flagBottom)
  {
    motorPWM[0] = 0;
    motorPWM[1] = 0;
  }

  if (timer.read_ms() > 1100) //エンコーダ読まなかったよう緊急停止（シーケンスは進める）
  {
    encoder->set(0,0);
    lenghtCurrent = 0;
    motorPWM[0] = 0;
    motorPWM[1] = 0;
  }
}
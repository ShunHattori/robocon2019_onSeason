#include "RojarArm.h"

RojarArm::RojarArm(double* variableToStore, DebounceSwitch& mySwitch)
{
  bottomSwitch = &mySwitch;
  motorPWM = variableToStore;
  heightCurrent = 0;
  heightTarget = 0;
  isTargetHeightAroundZeroPoint = 1;
  aroundZeroPointPWM = 0.06;
}

bool RojarArm::stats(void)
{
  if (isTargetHeightAroundZeroPoint)
  { //ゼロ点フラグが立っている場合はリミットスイッチの状態で値を返す
    if (bottomSwitch->stats())
      return 1;
    return 0;
  }

  if ((heightTarget - 20) < heightCurrent && heightCurrent < (heightTarget + 20))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void RojarArm::setHeight(int targetHeight)
{
  if (!targetHeight)
    isTargetHeightAroundZeroPoint = 1;
  else
    isTargetHeightAroundZeroPoint = 0;
  heightTarget = targetHeight;
}

void RojarArm::setEncoderPulse(int pulse)
{
  heightCurrent = pulse;
}

void RojarArm::setMaxPWM(double targetPWM)
{
  userPWM = targetPWM;
}

int RojarArm::getHeight(void)
{
  return heightCurrent;
}

void RojarArm::update(void)
{
  bottomSwitch->update();
  if (isTargetHeightAroundZeroPoint)
  { //目標停止位置が０
    if (heightCurrent < 200)
    { //現在の高さが150パルスよりも小さい
      if (bottomSwitch->stats())
      { //最下点スイッチが押されている
        motorPWM[0] = 0;
        motorPWM[1] = 0;
        return;
      }
      if (0 < heightCurrent)
      {
        motorPWM[0] = aroundZeroPointPWM;
        motorPWM[1] = 0;
      }
      return;
    }
  }

  if ((heightTarget - 20) < heightCurrent && heightCurrent < (heightTarget + 20))
  {
    motorPWM[0] = 0;
    motorPWM[1] = 0;
  }
  else
  {
    if (heightTarget < heightCurrent)
    {
      motorPWM[0] = userPWM;
      motorPWM[1] = 0;
    }
    else if (heightTarget > heightCurrent)
    {
      motorPWM[0] = 0;
      motorPWM[1] = userPWM;
    }
  }
  return;
}
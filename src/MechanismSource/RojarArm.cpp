#include "RojarArm.h"

RojarArm::RojarArm(double* variableToStore, DebounceSwitch& mySwitch)
{
  bottomSwitch = &mySwitch;
  motorPWM = variableToStore;
  heightBias = 0;
  heightCurrent = 0;
  heightTarget = 0;
  isTargetHeightAroundZeroPoint = 1;
  aroundZeroPointPWM = 0.1;
}

bool RojarArm::stats(void)
{
  if (isTargetHeightAroundZeroPoint)
  { //ゼロ点フラグが立っている場合はリミットスイッチの状態で値を返す
    if (bottomSwitch->stats())
      return 1;
    return 0;
  }

  if ((heightTarget - 20) < (heightCurrent - heightBias) && (heightCurrent - heightBias) < (heightTarget + 20))
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
  static bool exitFlag = 0;
  if (heightCurrent < -350)
  {
    exitFlag = 1;
  }
  if (exitFlag)
  {
    motorPWM[0] = 0;
    motorPWM[1] = 0;
  }
  bottomSwitch->update();
  if (bottomSwitch->stats())
  { //最下点スイッチが押されている
    heightBias = heightCurrent;
  }
  if (isTargetHeightAroundZeroPoint)
  { //目標停止位置が０
    if ((heightCurrent - heightBias) < 200)
    { //現在の高さが200パルスよりも小さい
      if (bottomSwitch->stats())
      { //最下点スイッチが押されている
        motorPWM[0] = 0;
        motorPWM[1] = 0;
        return;
      }
      if (0 < (heightCurrent - heightBias))
      {
        motorPWM[0] = aroundZeroPointPWM;
        motorPWM[1] = 0;
      }
      return;
    }
  }

  if ((heightTarget - 20) < (heightCurrent - heightBias) && (heightCurrent - heightBias) < (heightTarget + 20))
  {
    motorPWM[0] = 0;
    motorPWM[1] = 0;
  }
  else
  {
    if (heightTarget < (heightCurrent - heightBias))
    {
      motorPWM[0] = userPWM;
      motorPWM[1] = 0;
    }
    else if (heightTarget > (heightCurrent - heightBias))
    {
      motorPWM[0] = 0;
      motorPWM[1] = userPWM;
    }
  }
  return;
}
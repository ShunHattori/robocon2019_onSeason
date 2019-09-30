#include "ClothHold.h"

ClothHold::ClothHold(PinName servoRightPin, PinName servoLeftPin)
{

  servoRight = new Servo(servoRightPin);
  servoLeft = new Servo(servoLeftPin);
  servoRight->calibrate(0.00095F, 180.0F);
  servoLeft->calibrate(0.00095F, 180.0F);
  runningModeRight = 0;
  runningModeLeft = 0;
  blueModeFlag = 0;
}

void ClothHold::setFieldMode(int whichMode)
{
  if (whichMode)
  {
    blueModeFlag = 1;
  }
  else
    blueModeFlag = 0;
  return;
}

void ClothHold::release(int whichServo)
{
  if (blueModeFlag)
  {
    switch (whichServo)
    {
      case 0:
        runningModeRight = 0;                 //0 for release
        servoRight->write(workingPattern[0]); //数字は適当
        break;
      case 1:
        runningModeLeft = 0;
        servoLeft->write(workingPattern[1]); //数字は適当
        break;
      default:
        break;
    }
  }
  else
  {
    switch (whichServo)
    {
      case 0:
        runningModeRight = 0;                 //0 for release
        servoRight->write(workingPattern[1]); //数字は適当
        break;
      case 1:
        runningModeLeft = 0;
        servoLeft->write(workingPattern[0]); //数字は適当
        break;
      default:
        break;
    }
  }
}

void ClothHold::grasp(int whichServo)
{
  if (blueModeFlag)
  {
    switch (whichServo)
    {
      case 0:
        runningModeRight = 0;                 //0 for release
        servoRight->write(workingPattern[1]); //数字は適当
        break;
      case 1:
        runningModeLeft = 0;
        servoLeft->write(workingPattern[0]); //数字は適当
        break;
      default:
        break;
    }
  }
  else
  {
    switch (whichServo)
    {
      case 0:
        runningModeRight = 0;                 //0 for release
        servoRight->write(workingPattern[0]); //数字は適当
        break;
      case 1:
        runningModeLeft = 0;
        servoLeft->write(workingPattern[1]); //数字は適当
        break;
      default:
        break;
    }
  }
}

void ClothHold::center(int whichServo)
{
  switch (whichServo)
  {
    case 0:
      servoRight->write(workingPattern[2]); //数字は適当
      break;
    case 1:
      servoLeft->write(workingPattern[3]); //数字は適当
      break;
    default:
      break;
  }
}

void ClothHold::half(int whichServo)
{
  switch (whichServo)
  {
    case 0:
      servoRight->write(workingPattern[3]); //数字は適当
      break;
    case 1:
      servoLeft->write(workingPattern[2]); //数字は適当
      break;
    default:
      break;
  }
}

void ClothHold::free(int whichServo)
{
  switch (whichServo)
  {
    case 0:
      servoRight->free();
      break;

    case 1:
      servoLeft->free();
      break;

    default:
      break;
  }
}

bool ClothHold::stats(int whichServo)
{
  switch (whichServo)
  {
    float currentPosition;
    case 0:
      currentPosition = servoRight->read();
      if (runningModeRight) //enter when grasping
      {
        if (abs(workingPattern[1] - 0.1) < currentPosition && currentPosition < abs(workingPattern[1] + 0.1))
        {
          return 1;
        }
        else
        {
          return 0;
        }
      }
      else //enter when its releasing the object
      {
        if (abs(workingPattern[0] - 0.1) < currentPosition && currentPosition < abs(workingPattern[0] + 0.1)) //arm had been moved
        {
          return 1;
        }
        else //hasn't moved or it's moving
        {
          return 0;
        }
      }
      break;
    case 1:
      currentPosition = servoLeft->read();
      if (runningModeRight) //enter when grasping
      {
        if (abs(workingPattern[0] - 0.1) < currentPosition && currentPosition < abs(workingPattern[0] + 0.1))
        {
          return 1;
        }
        else
        {
          return 0;
        }
      }
      else //enter when its releasing the object
      {
        if (abs(workingPattern[1] - 0.1) < currentPosition && currentPosition < abs(workingPattern[1] + 0.1)) //arm had been moved
        {
          return 1;
        }
        else //hasn't moved or it's moving
        {
          return 0;
        }
      }
      break;
    default:
      break;
  }
  return 0;
}
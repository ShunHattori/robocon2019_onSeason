#include "ClothHold.h"

ClothHold::ClothHold(PinName servoRightPin, PinName servoLeftPin)
{

  servoRight = new Servo(servoRightPin);
  servoLeft = new Servo(servoLeftPin);
  servoRight->calibrate(0.00095F, 180.0F);
  servoLeft->calibrate(0.00095F, 180.0F);
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
        servoRight->write(workingPattern[0]); 
        break;
      case 1:
        servoLeft->write(workingPattern[1]);
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
        servoRight->write(workingPattern[1]); 
        break;
      case 1:
        servoLeft->write(workingPattern[0]);
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
        servoRight->write(workingPattern[1]);
        break;
      case 1:
        servoLeft->write(workingPattern[0]);
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
        servoRight->write(workingPattern[0]);
        break;
      case 1:
        servoLeft->write(workingPattern[1]);
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
      servoRight->write(workingPattern[2]);
      break;
    case 1:
      servoLeft->write(workingPattern[3]);
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
      servoRight->write(workingPattern[3]);
      break;
    case 1:
      servoLeft->write(workingPattern[2]);
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

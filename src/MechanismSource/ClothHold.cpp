#include "ClothHold.h"

ClothHold::ClothHold(PinName servoRightPin, PinName servoLeftPin)
{

    servoRight = new Servo(servoRightPin);
    servoLeft = new Servo(servoLeftPin);
    servoRight->calibrate(0.0006F, 90.0F);
    servoLeft->calibrate(0.0006F, 90.0F);
    runningModeRight = 0;
    runningModeLeft = 0;
}

void ClothHold::release(char whichServo)
{
    switch (whichServo)
    {
    case 'r':
        runningModeRight = 0;                 //0 for release
        servoRight->write(workingPattern[0]); //数字は適当
        break;
    case 'l':
        runningModeLeft = 0;
        servoLeft->write(workingPattern[1]); //数字は適当
        break;
    default:
        break;
    }
}

void ClothHold::grasp(char whichServo)
{
    switch (whichServo)
    {
    case 'r':
        runningModeRight = 1;                 //1 for grasp
        servoRight->write(workingPattern[1]); //数字は適当
        break;
    case 'l':
        runningModeLeft = 1;
        servoLeft->write(workingPattern[0]); //数字は適当
        break;
    default:
        break;
    }
}

void ClothHold::center(char whichServo)
{
    switch (whichServo)
    {
    case 'r':
        servoRight->write(workingPattern[2]); //数字は適当
        break;
    case 'l':
        servoLeft->write(workingPattern[3]); //数字は適当
        break;
    default:
        break;
    }
}

void ClothHold::free(char whichServo)
{
    switch (whichServo)
    {
    case 'r':
        servoRight->free();
        break;

    case 'l':
        servoLeft->free();
        break;

    default:
        break;
    }
}

bool ClothHold::stats(char whichServo)
{
    switch (whichServo)
    {
        float currentPosition;
    case 'r':
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
    case 'l':
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
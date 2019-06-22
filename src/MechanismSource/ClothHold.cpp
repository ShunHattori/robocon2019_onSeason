#include "ClothHold.h"

ClothHold::ClothHold(PinName servoRightPin, PinName servoLeftPin)
{
    servoRight = new Servo(servoRightPin);
    servoLeft = new Servo(servoLeftPin);
    servoRight->calibrate(0.0005F, 90.0F);
    servoLeft->calibrate(0.0005F, 90.0F);
    runningModeRight = 0;
    runningModeLeft = 0;
}

void ClothHold::release(char whichServo)
{
    switch (whichServo)
    {
    case 'r':
        runningModeRight = 0;                    //0 for release
        servoRight->position(targetPosition[2]); //数字は適当
        break;
    case 'l':
        runningModeLeft = 0;
        servoLeft->position(targetPosition[2]); //数字は適当
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
        runningModeRight = 1;                    //1 for grasp
        servoRight->position(targetPosition[1]); //数字は適当
        break;
    case 'l':
        runningModeLeft = 1;
        servoLeft->position(targetPosition[3]); //数字は適当
        break;
    default:
        break;
    }
}

bool ClothHold::stats(char whichServo)
{
    switch (whichServo)
    {
        int8_t currentPosition;
    case 'r':
        currentPosition = servoRight->read();
        if (runningModeRight) //enter when grasping
        {
            if (abs(targetPosition[0] - 10) < currentPosition && currentPosition < abs(targetPosition[0] + 10))
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
            if (abs(targetPosition[1] - 10) < currentPosition && currentPosition < abs(targetPosition[1] + 10)) //arm had been moved
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
            if (abs(targetPosition[0] - 10) < currentPosition && currentPosition < abs(targetPosition[0] + 10))
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
            if (abs(targetPosition[2] - 10) < currentPosition && currentPosition < abs(targetPosition[2] + 10)) //arm had been moved
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
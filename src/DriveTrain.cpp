#include "DriveTrain.h"

void DriveTrain::update()
{
    //三点接地エンコーダによる移動距離計算
    XEncodedDistanceDiff = abs(XAxis_1->getDistance() - XAxis_2->getDistance());
    currentYaw = (asin(XEncodedDistanceDiff / (2 * encoderAttachDiff))) * 180 / M_PI;

    currentX += ((XAxis_1->getDistance() + XAxis_2->getDistance()) / 2) * cos(currentYaw * M_PI / 180);
    currentY += -((XAxis_1->getDistance() + XAxis_2->getDistance()) / 2) * sin(currentYaw * M_PI / 180);
    XAxis_1->setDistance(0);
    XAxis_2->setDistance(0);

    currentX += (YAxis_1->getDistance() * sin(currentYaw * M_PI / 180));
    currentY += (YAxis_1->getDistance() * cos(currentYaw * M_PI / 180));
    YAxis_1->setDistance(0);

    //常に最新座標を受け取り続ける
    targetX = LCM->getXLocationData();
    targetY = LCM->getYLocationData();
    targetYaw = LCM->getYawStatsData();

    errorX = targetX - currentX;
    errorY = targetY - currentY;
    errorYaw = targetYaw - currentYaw;

    if ((-allocateError < errorX && errorX < allocateError) && (-allocateError < errorY && errorY < allocateError) && (-allocateError < errorYaw && errorYaw < allocateError))
    {
        if (ConfirmStatsInitialFlag)
        {
            ConfirmStats.start();
            ConfirmStatsInitialFlag = 0;
        }
        if (ConfirmStats.read_ms() > 100)
        {
            stats = 1;
        }
    }
    else
    {
        stats = 0;
        ConfirmStats.reset();
        ConfirmStatsInitialFlag = 1;
    }

    if (stats) //本来は反転 def(stats), for testing(!stats)
    {
        if (-allocateError < errorX && errorX < allocateError)
        {
            Vec[0] = 0;
        }
        else if (errorX <= -decreaseRadius || decreaseRadius <= errorX)
        {
            if (errorX <= 0)
                Vec[0] = -Max;
            if (errorX >= 0)
                Vec[0] = Max;
        }
        else
        {
            if (errorX < 0)
                Vec[0] = mapFloat(errorX, 0, -decreaseRadius, -Min, -Max);
            else
                Vec[0] = mapFloat(errorX, 0, decreaseRadius, Min, Max);
        }

        if (-allocateError < errorY && errorY < allocateError)
        {
            Vec[1] = 0;
        }
        else if (errorY <= -decreaseRadius || decreaseRadius <= errorY)
        {
            if (errorY <= 0)
                Vec[1] = -Max;
            if (errorY >= 0)
                Vec[1] = Max;
        }
        else
        {
            if (errorY < 0)
                Vec[1] = mapFloat(errorY, 0, -decreaseRadius, -Min, -Max);
            else
                Vec[1] = mapFloat(errorY, 0, decreaseRadius, Min, Max);
        }

        if (-2 < errorYaw && errorYaw < 2)
        {
            Vec[2] = 0;
        }
        else
        {
            Vec[2] = mapFloat(errorYaw, 0, 45, 0, Max);
        }
    }
    else //ここに加速制御
    {

        if (abs(errorX) >= decreaseRadius) //for X error
        {
            if (0 < errorX) //increase P control
            {
                Vec[0] += 50;
                xReachedMaxPWM = Vec[0];
            }
            else
            {
                Vec[0] -= 50;
                xReachedMaxPWM = -Vec[0];
            }
            if (Vec[0] > Max) //constrain
            {
                Vec[0] = Max;
            }
            else if(Vec[0] < -Max)
            {
                Vec[0] = -Max;
            }
        }
        else
        { //decrease curved control
            if (-allocateError < errorX && errorX < allocateError)
            {
                Vec[0] = 0;
            }
            else if (errorX <= -decreaseRadius || decreaseRadius <= errorX)
            {
                if (errorX <= 0)
                    Vec[0] = -Max;
                if (errorX >= 0)
                    Vec[0] = Max;
            }
            else
            {
                if (errorX < 0)
                    Vec[0] = mapFloat(errorX, 0, -decreaseRadius, -Min, -xReachedMaxPWM);
                else
                    Vec[0] = mapFloat(errorX, 0, decreaseRadius, Min, xReachedMaxPWM);
            }
        }

        if (abs(errorY) >= decreaseRadius) //for Y error
        {
            if (0 < errorY) //increase P control
            {
                Vec[1] += 50;
                xReachedMaxPWM = Vec[1];
            }
            else
            {
                Vec[1] -= 50;
                xReachedMaxPWM = -Vec[1];
            }
            if (Vec[1] > Max) //constrain
            {
                Vec[1] = Max;
            }
            else if (Vec[1] < -Max)
            {
                Vec[1] = -Max;
            }
        }
        else
        { //decrease curved control
            if (-allocateError < errorY && errorY < allocateError)
            {
                Vec[1] = 0;
            }
            else if (errorY <= -decreaseRadius || decreaseRadius <= errorY)
            {
                if (errorY <= 0)
                    Vec[1] = -Max;
                if (errorY >= 0)
                    Vec[1] = Max;
            }
            else
            {
                if (errorY < 0)
                    Vec[1] = mapFloat(errorY, 0, -decreaseRadius, -Min, -xReachedMaxPWM);
                else
                    Vec[1] = mapFloat(errorY, 0, decreaseRadius, Min, xReachedMaxPWM);
            }
        }

        if (-2 < errorYaw && errorYaw < 2) //for Yaw control
        {
            Vec[2] = 0;
        }
        else
        {
            Vec[2] = mapFloat(errorYaw, 0, 45, 0, Max);
        }
    }
}

float DriveTrain::mapFloat(float value, float in_min, float in_max, float out_min, float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

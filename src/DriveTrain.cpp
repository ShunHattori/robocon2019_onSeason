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

    if ((-ここを変える < errorX && errorX < ここを変える) && (-ここを変える < errorY && errorY < ここを変える) && (-ここを変える < errorYaw && errorYaw < ここを変える))
    {
        stats = 1;
    }
    else
    {
        stats = 0;
    }

    if (stats)//本来は反転
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
        else if (errorX == 0)
            Vec[0] = 0;
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
        else if (errorY == 0)
            Vec[1] = 0;
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
    else    //ここに加速制御
    {
        
    }
}

float DriveTrain::mapFloat(float value, float in_min, float in_max, float out_min, float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
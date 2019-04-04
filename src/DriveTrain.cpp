#include "DriveTrain.h"

void DriveTrain::update(float vec[])
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

    if ((-allocateError < errorX || errorX < allocateError) && (-allocateError < errorY || errorY < allocateError) && (-allocateError < errorYaw || errorYaw < allocateError))
    {
        stats = 1;
    }
    else
    {
        stats = 0;
    }

    if (stats)
    {
        if (errorX <= -decreaseRadius || decreaseRadius <= errorX)
        {
            if (errorX <= 0)
                vec[0] = -0.9;
            if (errorX >= 0)
                vec[0] = 0.9;
        }
        else if (errorX == 0)
            vec[0] = 0;
        else
        {
            if (errorX < 0)
                vec[0] = mapFloat(errorX, 0, -decreaseRadius, -0.1, -0.9);
            else
                vec[0] = mapFloat(errorX, 0, decreaseRadius, 0.1, 0.9);
        }

        if (errorY <= -decreaseRadius || decreaseRadius <= errorY)
        {
            if (errorY <= 0)
                vec[1] = -0.9;
            if (errorY >= 0)
                vec[1] = 0.9;
        }
        else if (errorY == 0)
            vec[1] = 0;
        else
        {
            if (errorY < 0)
                vec[1] = mapFloat(errorY, 0, -decreaseRadius, -0.1, -0.9);
            else
                vec[1] = mapFloat(errorY, 0, decreaseRadius, 0.1, 0.9);
        }

        vec[2] = mapFloat(errorYaw, 0, 45, 0, 0.9);
    }
}

float DriveTrain::mapFloat(float value, float in_min, float in_max, float out_min, float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
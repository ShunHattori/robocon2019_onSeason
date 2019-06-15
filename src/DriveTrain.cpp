#include "DriveTrain.h"

void DriveTrain::update()
{
    xReachedMaxPWM = 0.3;
    yReachedMaxPWM = 0.3;
    if (encoderMode)
    { //encoderMode

        tempX = XAxis_1->getDistance();
        XAxis_1->setDistance(0);
        tempY = YAxis_1->getDistance();
        YAxis_1->setDistance(0);
        //currentYaw = imu->gyro_Yaw();

        //各計測輪の移動量とロボットの傾きから全体の移動量を計算する
        currentX += tempX * cos(currentYaw * 3.1415 / 180);
        currentY += tempX * sin(currentYaw * 3.1415 / 180);
        currentX -= tempY * sin(currentYaw * 3.1415 / 180);
        currentY += tempY * cos(currentYaw * 3.1415 / 180);

        //currentX += tempX * cos(currentYaw * 3.1415 / 180);
        //currentY += tempY * cos(currentYaw * 3.1415 / 180);

        /*テストには下のコードを使う LCDのＣＰに各エンコーダの移動距離が表示されるはず
        currentX = tempX;
        currentY = tempSub;
        currentYaw = tempY;*/
    }
    else
    { //sensor mode

        //X軸は計測輪,Y軸は測距センサで自己位置を計算する　※移動後Y軸現在位置を更新してあげる必要あり
        //currentYaw = imu->gyro_Yaw();
        tempX = XAxis_1->getDistance();
        XAxis_1->setDistance(0);
        tempY = YAxis_1->getDistance();
        YAxis_1->setDistance(0);
        currentY += tempY * cos(currentYaw * 3.1415 / 180);
        currentY += tempX * sin(currentYaw * 3.1415 / 180);
        currentX = sensorDistance;
    }
    //常に最新座標を受け取り続ける
    targetX = LCM->getXLocationData();
    targetY = LCM->getYLocationData();
    targetYaw = LCM->getYawStatsData();
    //取得した目標位置と現在位置で偏差を計算する
    errorX = targetX - currentX;
    errorY = targetY - currentY;
    errorYaw = targetYaw - currentYaw;

    //ロボットの自己位置が指定された許容範囲内であればstatsを更新する
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

    //statsにより出力の計算を切り替える
    if (stats) //stats
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

        if (-0.3 < errorYaw && errorYaw < 0.3)
        {
            Vec[2] = 0;
        }
        else
        {
            Vec[2] = mapFloat(errorYaw, -30, 30, -0.3, 0.3);
        }
    }
    else //ここに加速制御
    {

        if (abs(errorX) >= decreaseRadius) //for X error
        {
            if (0 < errorX) //increase P control
            {
                Vec[0] += 0.055;
                xReachedMaxPWM = Vec[0];
            }
            else
            {
                Vec[0] -= 0.055;
                xReachedMaxPWM = -Vec[0];
            }
            if (Vec[0] > Max) //constrain
            {
                Vec[0] = Max;
            }
            else if (Vec[0] < -Max)
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
                Vec[1] += 0.055;
                xReachedMaxPWM = Vec[1];
            }
            else
            {
                Vec[1] -= 0.055;
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

        if (-0.3 < errorYaw && errorYaw < 0.3) //for Yaw control
        {
            Vec[2] = 0;
        }
        else
        {
            Vec[2] = mapFloat(errorYaw, -30, 30, -0.3, 0.3);
        }
    }
}

float DriveTrain::mapFloat(float value, float in_min, float in_max, float out_min, float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

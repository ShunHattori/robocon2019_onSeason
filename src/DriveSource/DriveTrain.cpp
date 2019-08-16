#include "DriveTrain.h"
#include "math.h"

void DriveTrain::update()
{
  xReachedMaxPWM = 0.19;
  yReachedMaxPWM = 0.19;
  if (encoderMode)
  {
    tempX = XAxis_1->getDistance() / 2;
    XAxis_1->setDistance(0);
    tempY = YAxis_1->getDistance() / 2;
    YAxis_1->setDistance(0);
    //各計測輪の移動量とロボットの傾きから全体の移動量を計算する
    currentX += tempX * cos(currentYaw * M_PI / 180);
    currentY += tempX * sin(currentYaw * M_PI / 180);
    currentX -= tempY * sin(currentYaw * M_PI / 180);
    currentY += tempY * cos(currentYaw * M_PI / 180);
  }
  else
  { //sensor mode

    //X軸は計測輪,Y軸は測距センサで自己位置を計算する　※移動後Y軸現在位置を更新してあげる必要あり
    //currentYaw = imu->gyro_Yaw();
    tempX = XAxis_1->getDistance() / 2;
    XAxis_1->setDistance(0);
    tempY = YAxis_1->getDistance() / 2;
    YAxis_1->setDistance(0);
    currentY += tempY * cos(currentYaw * 3.1415 / 180);
    currentY += tempX * sin(currentYaw * 3.1415 / 180);
    currentX = sensorDistance;
  }
  //常に最新目標座標を受け取り続ける
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
        Vec[0] = mapdouble(errorX, 0, -decreaseRadius, -Min, -Max);
      else
        Vec[0] = mapdouble(errorX, 0, decreaseRadius, Min, Max);
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
        Vec[1] = mapdouble(errorY, 0, -decreaseRadius, -Min, -Max);
      else
        Vec[1] = mapdouble(errorY, 0, decreaseRadius, Min, Max);
    }

    if (-0.1 < errorYaw && errorYaw < 0.1)
    {
      Vec[2] = 0;
    }
    else
    {
      Vec[2] = mapdouble(errorYaw, -20, 20, -0.18, 0.18);
    }
  }
  else //ここに加速制御
  {

    if (abs(errorX) >= decreaseRadius) //for X error
    {
      if (0 < errorX) //increase P control
      {
        Vec[0] += 0.00045;
        xReachedMaxPWM = Vec[0];
      }
      else
      {
        Vec[0] -= 0.00045;
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
          Vec[0] = mapdouble(errorX, 0, -decreaseRadius, -Min, -xReachedMaxPWM);
        else
          Vec[0] = mapdouble(errorX, 0, decreaseRadius, Min, xReachedMaxPWM);
      }
    }

    if (abs(errorY) >= decreaseRadius) //for Y error
    {
      if (0 < errorY) //increase P control
      {
        Vec[1] += 0.00045;
        xReachedMaxPWM = Vec[1];
      }
      else
      {
        Vec[1] -= 0.00045;
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
          Vec[1] = mapdouble(errorY, 0, -decreaseRadius, -Min, -xReachedMaxPWM);
        else
          Vec[1] = mapdouble(errorY, 0, decreaseRadius, Min, xReachedMaxPWM);
      }
    }

    if (-0.1 < errorYaw && errorYaw < 0.1) //for Yaw control
    {
      Vec[2] = 0;
    }
    else
    {
      Vec[2] = mapdouble(errorYaw, -20, 20, -0.18, 0.18);
    }
  }
}

double DriveTrain::mapdouble(double value, double in_min, double in_max, double out_min, double out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#include "DriveTrain.h"
#include "math.h"

double DriveTrain::mapDouble(double value, double in_min, double in_max, double out_min, double out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool DriveTrain::getMovingStats()
{
  if (!(-Drive.allocateError < xAxis.error && xAxis.error < Drive.allocateError))
  {
    movingComfirmTimer.reset();
    movingComfirmTimer.stop();
    Drive.ConfirmStatsInitialFlag = 1;
    return 0;
  }
  if (!(-Drive.allocateError < yAxis.error && yAxis.error < Drive.allocateError))
  {
    movingComfirmTimer.reset();
    movingComfirmTimer.stop();
    Drive.ConfirmStatsInitialFlag = 1;
    return 0;
  }
  if (!(-Drive.allocateError < yawAxis.error && yawAxis.error < Drive.allocateError))
  {
    movingComfirmTimer.reset();
    movingComfirmTimer.stop();
    Drive.ConfirmStatsInitialFlag = 1;
    return 0;
  }

  if (Drive.ConfirmStatsInitialFlag)
  {
    movingComfirmTimer.reset();
    movingComfirmTimer.start();
    Drive.ConfirmStatsInitialFlag = 0;
    return 0;
  }
  if (movingComfirmTimer.read_ms() > 50)
  {
    movingComfirmTimer.reset();
    movingComfirmTimer.stop();
    return 1;
  }
  return 0;
}

void DriveTrain::retentionDriving()
{
  //xAxis Process
  if (-Drive.allocateError < xAxis.error && xAxis.error < Drive.allocateError)
  {
    Drive.vector[0] = 0;
  }
  else if (xAxis.error < -Drive.decreaseRadius || Drive.decreaseRadius < xAxis.error)
  {
    Drive.vector[0] = xAxis.error < 0 ? -Drive.maxPWM : Drive.maxPWM;
  }
  else
  {
    Drive.vector[0] = xAxis.error < 0 ? mapDouble(xAxis.error, 0, -Drive.decreaseRadius, -Drive.minPWM, -Drive.maxPWM) : mapDouble(xAxis.error, 0, Drive.decreaseRadius, Drive.minPWM, Drive.maxPWM);
  }
  //yAxis Process
  if (-Drive.allocateError < yAxis.error && yAxis.error < Drive.allocateError)
  {
    Drive.vector[1] = 0;
  }
  else if (yAxis.error < -Drive.decreaseRadius || Drive.decreaseRadius < yAxis.error)
  {
    Drive.vector[1] = yAxis.error < 0 ? -Drive.maxPWM : Drive.maxPWM;
  }
  else
  {
    Drive.vector[1] = yAxis.error < 0 ? mapDouble(yAxis.error, 0, -Drive.decreaseRadius, -Drive.minPWM, -Drive.maxPWM) : mapDouble(yAxis.error, 0, Drive.decreaseRadius, Drive.minPWM, Drive.maxPWM);
  }
}

void DriveTrain::accelerationDriving()
{
  //Drive.decreaseRadiusより次点の偏差が小さい場合、最大値が０でMAP処理をしてしまうのでDrive.minPWMより小さい場合少なくともminPWMでMAP処理をするようにする
  for (int axis = 0; axis < 2; axis++)
  {
    if (abs(Drive.reachedPWM[axis]) < Drive.minPWM)
    {
      Drive.reachedPWM[axis] = Drive.minPWM;
    }
  }
  //xAxis Process
  if (Drive.decreaseRadius < abs(xAxis.error)) //減速半径外の時
  {
    if (0 < xAxis.error)
    {
      Drive.vector[0] += xAxisAccelor->getValue();
      Drive.reachedPWM[0] = Drive.vector[0];
    }
    else
    {
      Drive.vector[0] -= xAxisAccelor->getValue();
      Drive.reachedPWM[0] = -Drive.vector[0];
    }
    if (Drive.vector[0] < -Drive.decreaseRadius || Drive.decreaseRadius < Drive.vector[0])
    {
      Drive.vector[0] = Drive.vector[0] < 0 ? -Drive.maxPWM : Drive.maxPWM;
    }
  }
  else
  {
    if (-Drive.allocateError < xAxis.error && xAxis.error < Drive.allocateError)
    {
      Drive.vector[0] = 0;
    }
    else if (xAxis.error < -Drive.decreaseRadius || Drive.decreaseRadius < xAxis.error)
    {
      Drive.vector[0] = xAxis.error < 0 ? -Drive.maxPWM : Drive.maxPWM;
    }
    else
    {
      Drive.vector[0] = xAxis.error < 0 ? mapDouble(xAxis.error, 0, -Drive.decreaseRadius, -Drive.minPWM, -Drive.reachedPWM[0]) : mapDouble(xAxis.error, 0, Drive.decreaseRadius, Drive.minPWM, Drive.reachedPWM[0]);
    }
  }

  //yAxis Process
  if (Drive.decreaseRadius < abs(yAxis.error))
  {
    if (0 < yAxis.error)
    {
      Drive.vector[1] += yAxisAccelor->getValue();
      Drive.reachedPWM[0] = Drive.vector[1];
    }
    else
    {
      Drive.vector[1] -= yAxisAccelor->getValue();
      Drive.reachedPWM[0] = -Drive.vector[1];
    }
    if (Drive.vector[1] < -Drive.decreaseRadius || Drive.decreaseRadius < Drive.vector[1])
    {
      Drive.vector[1] = Drive.vector[1] < 0 ? -Drive.maxPWM : Drive.maxPWM;
    }
  }
  else
  {
    if (-Drive.allocateError < yAxis.error && yAxis.error < Drive.allocateError)
    {
      Drive.vector[1] = 0;
    }
    else if (yAxis.error < -Drive.decreaseRadius || Drive.decreaseRadius < yAxis.error)
    {
      Drive.vector[1] = yAxis.error < 0 ? -Drive.maxPWM : Drive.maxPWM;
    }
    else
    {
      Drive.vector[1] = yAxis.error < 0 ? mapDouble(yAxis.error, 0, -Drive.decreaseRadius, -Drive.minPWM, -Drive.reachedPWM[0]) : mapDouble(yAxis.error, 0, Drive.decreaseRadius, Drive.minPWM, Drive.reachedPWM[0]);
    }
  }
}

void DriveTrain::yawAxisRetentionDriving()
{
  //yawAxis Process
  if (-yawSensitivity.allocateError < yawAxis.error && yawAxis.error < yawSensitivity.allocateError)
  {
    Drive.vector[2] = 0;
  }
  else
  {
    Drive.vector[2] = mapDouble(yawAxis.error, -1 / yawSensitivity.turningStrength, 1 / yawSensitivity.turningStrength, -yawSensitivity.turningPWM, yawSensitivity.turningPWM);
  }
}

void DriveTrain::update()
{
  Drive.reachedPWM[0] = 0.19;
  Drive.reachedPWM[1] = 0.19;
  xAxis.temp = XAxis_1->getDistance() / 2;
  XAxis_1->setDistance(0);
  yAxis.temp = YAxis_1->getDistance() / 2;
  YAxis_1->setDistance(0);
  //各計測輪の移動量とロボットの傾きから全体の移動量を計算する
  xAxis.current += xAxis.temp * cos(yawAxis.current * M_PI / 180);
  yAxis.current += xAxis.temp * sin(yawAxis.current * M_PI / 180);
  xAxis.current -= yAxis.temp * sin(yawAxis.current * M_PI / 180);
  yAxis.current += yAxis.temp * cos(yawAxis.current * M_PI / 180);
  //常に最新目標座標を受け取り続ける
  xAxis.target = LCM->getXLocationData();
  yAxis.target = LCM->getYLocationData();
  yawAxis.target = LCM->getYawStatsData();
  //取得した目標位置と現在位置で偏差を計算する
  xAxis.error = xAxis.target - xAxis.current;
  yAxis.error = yAxis.target - yAxis.current;
  yawAxis.error = yawAxis.target - yawAxis.current;
  //ロボットの自己位置が指定された許容範囲内であればstatsを更新する
  Drive.stats = getMovingStats();
  //yaw軸は常に姿勢維持制御
  yawAxisRetentionDriving();
  //Drive.statsにより出力の計算を切り替える
  if (Drive.stats) //現在地維持制御
  {
    retentionDriving();
    return;
  }
  accelerationDriving();
  return;
}
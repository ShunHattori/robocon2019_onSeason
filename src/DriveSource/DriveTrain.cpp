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
    Drive.isTargetPositionChanged = 0;
    return 1;
  }
  return 0;
}

void DriveTrain::retentionDriving()
{
  //xAxis Process
  if (-Drive.driveDisableError < xAxis.error && xAxis.error < Drive.driveDisableError)
  { //偏差がモーター停止円半径内だったらベクトルを0に設定する
    Drive.vector[0] = 0;
  }
  else if (xAxis.error < -Drive.decreaseRadius || Drive.decreaseRadius < xAxis.error)
  { //減速判定円より偏差が大きい場合ベクトルを最大に設定する
    Drive.vector[0] = xAxis.error < 0 ? -Drive.maxPWM : Drive.maxPWM;
  }
  else
  { //偏差が二つの円の間にある場合p制御を行う
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
  if (Drive.decreaseRadius < abs(xAxis.error))
  { //減速半径外の時
    if (0 < xAxis.error)
    {
      Drive.vector[0] += xAxisAccelor->getValue();
      Drive.reachedPWM[0] = abs(Drive.vector[0]);
    }
    else
    {
      Drive.vector[0] -= xAxisAccelor->getValue();
      Drive.reachedPWM[0] = abs(Drive.vector[0]);
    }
    if (Drive.vector[0] < -Drive.maxPWM || Drive.maxPWM < Drive.vector[0])
    { //台形制御により最大出力よりもベクトルが大きくなった場合制限する
      Drive.vector[0] = Drive.vector[0] < 0 ? -Drive.maxPWM : Drive.maxPWM;
    }
  }
  else
  { //偏差が減速半径よりも小さくなったorもとから小さかった場合
    if (-Drive.driveDisableError < xAxis.error && xAxis.error < Drive.driveDisableError)
    { //偏差がモーター停止円半径内だったらベクトルを0に設定する
      Drive.vector[0] = 0;
    }
    else
    { //加速時に到達した最大速度を上限にp制御を行う、無加速時はminPWM,minPWMでmapするので等速移動になる
      Drive.vector[0] = xAxis.error < 0 ? mapDouble(xAxis.error, Drive.allocateError, -Drive.decreaseRadius, -Drive.minPWM, -Drive.reachedPWM[0]) : mapDouble(xAxis.error, -Drive.allocateError, Drive.decreaseRadius, Drive.minPWM, Drive.reachedPWM[0]);
    }
  }

  //yAxis Process
  if (Drive.decreaseRadius < abs(yAxis.error))
  {
    if (0 < yAxis.error)
    {
      Drive.vector[1] += yAxisAccelor->getValue();
      Drive.reachedPWM[1] = abs(Drive.vector[1]);
    }
    else
    {
      Drive.vector[1] -= yAxisAccelor->getValue();
      Drive.reachedPWM[1] = abs(Drive.vector[1]);
    }
    if (Drive.vector[1] < -Drive.maxPWM || Drive.maxPWM < Drive.vector[1])
    {
      Drive.vector[1] = Drive.vector[1] < 0 ? -Drive.maxPWM : Drive.maxPWM;
    }
  }
  else
  {
    if (-Drive.driveDisableError < yAxis.error && yAxis.error < Drive.driveDisableError)
    {
      Drive.vector[1] = 0;
    }
    else
    {
      Drive.vector[1] = yAxis.error < 0 ? mapDouble(yAxis.error, Drive.allocateError, -Drive.decreaseRadius, -Drive.minPWM, -Drive.reachedPWM[0]) : mapDouble(yAxis.error, -Drive.allocateError, Drive.decreaseRadius, Drive.minPWM, Drive.reachedPWM[0]);
    }
  }
  //目標位置まで直線で移動できるように補正
  Drive.vector[0] = Drive.outputRate[0];
  Drive.vector[1] = Drive.outputRate[1];
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
  calcOutputRateWhenTargetChanged();
  //yaw軸は常に姿勢維持制御
  yawAxisRetentionDriving();
  //Drive.statsにより出力の計算を切り替える
  if (!Drive.isTargetPositionChanged) //現在地維持制御
  {
    retentionDriving();
    return;
  }
  accelerationDriving();
  return;
}

void DriveTrain::calcOutputRateWhenTargetChanged()
{
  static bool prevFlagState;
  if (Drive.isTargetPositionChanged && (prevFlagState != Drive.isTargetPositionChanged))
  { //新しく座標が設定されたとき
    double tempRate[2];
    tempRate[0] = abs(xAxis.error) / abs(xAxis.error) + abs(yAxis.error);
    tempRate[1] = abs(yAxis.error) / abs(xAxis.error) + abs(yAxis.error);
    double biggerRate = tempRate[0] > tempRate[1] ? tempRate[0] : tempRate[1];
    double correctionMultiplyToOne = 1 / biggerRate;
    Drive.outputRate[0] = tempRate[0] * correctionMultiplyToOne;
    Drive.outputRate[1] = tempRate[1] * correctionMultiplyToOne;
  }
  prevFlagState = Drive.isTargetPositionChanged;
}
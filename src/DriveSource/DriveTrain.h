#pragma once

#include "LocationManager.h"
#include "MWodometry.h"
#include "SensorSource\MPU9250.h"
#include "TimeIncreaser.h"
#include "math.h"
#include "mbed.h"

class DriveTrain
{
public:
  DriveTrain(LocationManager<double> &lcmObj, MWodometry &X1, MWodometry &Y1, MPU9250 &IMUobj, double AllocateError, double DecreaseRadius)
  {
    LCM = &lcmObj;
    XAxis_1 = &X1;
    YAxis_1 = &Y1;
    imu = &IMUobj;
    xAxisAccelor = new TimeIncreaser(5, 0.002);
    yAxisAccelor = new TimeIncreaser(5, 0.002);
    Drive.allocateError = AllocateError;
    Drive.decreaseRadius = DecreaseRadius;
    Drive.ConfirmStatsInitialFlag = 1;
    Drive.maxPWM = 0.3;
    Drive.maxPWM = 0.1;
    Drive.reachedPWM[0] = 0.35;
    Drive.reachedPWM[1] = 0.35;
    Drive.vector[0] = 0;
    Drive.vector[1] = 0;
    Drive.vector[2] = 0;
    xAxis.current = 0;
    yAxis.current = 0;
    yawAxis.current = 0;
    yawSensitivity.turningStrength = 0.5;
    yawSensitivity.allocateError = 0.2;
    yawSensitivity.turningPWM = 0.12;
  }

  /*
     * desc:   速度制御アルゴリズム
     * param:  none
     * return: none
     */
  void update();

  /*
     * desc:   移動の進行状況を取得する
     * param:  none
     * return: 1=移動完了,0=移動中
     */
  bool getStats()
  {
    return Drive.stats;
  }

  /*
     * desc:　  到達判定円の半径閾値変更
     * param:   半径(double)
     * return:  none
     */
  void setAllocateErrorCircleRadius(double radius)
  {
    Drive.allocateError = radius;
  }

  /*
     * desc:　  減速開始判定円の半径閾値変更
     * param:   半径(double)
     * return:  none
     */
  void setDecreaseCircleRadius(double radius)
  {
    Drive.decreaseRadius = radius;
  }

  /*
     * desc:   各軸の出力を取得する
     * param:  none
     * return: 各軸の出力値(min~max内)
     */
  double getXVector()
  {
    return Drive.vector[0];
  }
  double getYVector()
  {
    return Drive.vector[1];
  }
  double getYawVector()
  {
    return Drive.vector[2];
  }

  /*
     * desc:   現在のロボットの位置を取得する
     * param:  none
     * return: 各軸座標(double)
     */
  double getCurrentXPosition()
  {
    return xAxis.current;
  }
  double getCurrentYPosition()
  {
    return yAxis.current;
  }
  double getCurrentYawPosition()
  {
    return yawAxis.current;
  }

  /*
     * desc:   現在のロボットの位置を上書きする
     * param:  各軸座標(double)
     * return: none
     */
  void setCurrentXPosition(double position)
  {
    xAxis.current = position;
  }
  void setCurrentYPosition(double position)
  {
    yAxis.current = position;
  }
  void setCurrentYawPosition(double position)
  {
    yawAxis.current = -position;
  }

  /*
     * desc:   出力の最大値を設定する
     * param:  最大値
     * return: none
     */
  void setMaxOutput(double max)
  {
    Drive.maxPWM = max;
  }
  /*
     * desc:   出力の最小値を設定する
     * param:  最小値
     * return: none
     */
  void setMinOutput(double min)
  {
    Drive.minPWM = min;
  }

private:
  LocationManager<double> *LCM;
  MWodometry *XAxis_1, *YAxis_1;
  MPU9250 *imu;
  Timer movingComfirmTimer;
  TimeIncreaser *xAxisAccelor, *yAxisAccelor;

  //array[0]:X ,[1]:Y ,[2]:YAW
  struct
  {
    double vector[3];
    double allocateError;
    double decreaseRadius;
    double maxPWM, minPWM;
    double reachedPWM[2];
    bool ConfirmStatsInitialFlag;
    bool stats;
  } Drive;

  struct location
  {
    double current;
    double target;
    double error;
    double temp;
  } xAxis, yAxis, yawAxis;

  struct yawParam
  {
    double allocateError;
    double turningStrength; //Be applied with reciprocal of 1
    double turningPWM;
  } yawSensitivity;

  double mapDouble(double value, double in_min, double in_max, double out_min, double out_max);
  bool getMovingStats();
  void retentionDriving();
  void accelerationDriving();
  void yawAxisRetentionDriving();
};

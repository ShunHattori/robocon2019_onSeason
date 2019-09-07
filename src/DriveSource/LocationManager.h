#pragma once

#include <vector>
#include "MWodometry.h"
#include "mbed.h"

template <typename TYPE>
class LocationManager
{

public:
  /*
    *   desc: コンストラクタ
    */
  LocationManager(TYPE x = 0, TYPE y = 0, TYPE yaw = 0)
  {
    XPointArray.resize(0);
    YPointArray.resize(0);
    YawPointArray.resize(0);
    XPointArray.insert(XPointArray.begin(), x);
    YPointArray.insert(YPointArray.begin(), y);
    YawPointArray.insert(YawPointArray.begin(), yaw);
    currentPointNumber = 0;
  }

  /*
    *   desc:   ロボットの移動状況を確認
    *   param:  DriveTrainのgetStatsメソッド
    *   return: 1=移動完了,0=移動中orエラー
    */
  bool checkMovingStats(bool stats)
  {
    if (stats)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

  /*
    *   desc:   取得可能な座標を一つ先に更新する
    *   param:  none
    *   return: none
    */
  void sendNext()
  {
    currentPointNumber++;
  }

  /*
    *   desc:   指定した位置に移動
    *   param:  移動先X座標,Y座標
    *   return: none
    */
  void addPoint(TYPE x, TYPE y)
  {
    XPointArray.push_back(x);
    YPointArray.push_back(y);
    YawPointArray.push_back(getYawStatsData());
  }

  /*
    *   desc:   指定した位置に移動
    *   param:  移動先X座標,Y座標,YAW軸角度
    *   return: none
    */
  void addPoint(TYPE x, TYPE y, TYPE yaw)
  {
    XPointArray.push_back(x);
    YPointArray.push_back(y);
    YawPointArray.push_back(yaw);
  }

  /*
    *   desc:   先頭のデータを取得
    *   param:  Vector型のオブジェクト
    *   return: 引数のオブジェクトの先頭要素
    */
  int getXLocationData()
  {
    return (XPointArray.at(currentPointNumber));
  }
  int getYLocationData()
  {
    return (YPointArray.at(currentPointNumber));
  }
  int getYawStatsData()
  {
    return (YawPointArray.at(currentPointNumber));
  }
  /*
    *   desc:   目標位置を更新
    *   param:  更新後X座標,Y座標
    *   return: none
    */
  void setCurrentPoint(TYPE x, TYPE y, TYPE yaw)
  {
    XPointArray.at(currentPointNumber) = x;
    YPointArray.at(currentPointNumber) = y;
    YawPointArray.at(currentPointNumber) = yaw;
  }

private:
  std::vector<int> XPointArray, YPointArray, YawPointArray;

  uint8_t currentPointNumber, pointArraySize;
  int XLocationData, YLocationData, YawStatsData; //取得する現在位置の変数
  int XTargetData, YTargetData, YawTargetData;
  int XCurrentLocation, YCurrentLocation, YawCurrentStats;
};

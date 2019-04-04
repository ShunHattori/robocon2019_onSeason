#pragma once

#include "mbed.h"
#include "encoder.h"
#include "MWodometry.h"
#include "DriveTrain.h"
#include <vector>

template <typename TYPE>
class LocationManager
{

  public:
    /*
    *   desc: コンストラクタ
    */
    LocationManager(DriveTrain &_obj, TYPE x = 0, TYPE y = 0, TYPE yaw = 0)
    {
        DRObj = _obj;
        XPointArray.resize(100);
        YPointArray.resize(100);
        YawPointArray.resize(100);
        XPointArray.insert(XPointArray.begin(), x);
        YPointArray.insert(YPointArray.begin(), y);
        YawPointArray.insert(YawPointArray.begin(), yaw);
        currentPointNumber = 0;
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
        pointArraySize++;
    }

    /*
    *   desc:   先頭のデータを取得
    *   param:  Vector型のオブジェクト
    *   return: 引数のオブジェクトの先頭要素
    */
    int getXLocationData()
    {
        return XPointArray.front();
    }
    int getYLocationData()
    {
        return YPointArray.front();
    }
    int getYawStatsData()
    {
        return YawPointArray.front();
    }
    /*
    *   desc:   自己位置を更新
    *   param:  更新後X座標,Y座標
    *   return: none
    */
    void setCurrentPoint(TYPE x, TYPE y, TYPE yaw)
    {
        XPointArray.insert(XPointArray.begin(), x);
        YPointArray.insert(YPointArray.begin(), y);
        YawPointArray.insert(YawPointArray.begin(), yaw);
    }

    void update()
    {
    }

    bool isHere()
    {
        XLocationData = XPointArray.front() + currentPointNumber;
        YLocationData = YPointArray.front() + currentPointNumber;
        YawStatsData = YawPointArray.front() + currentPointNumber;
    }

  private:
    std::vector<int> XPointArray, YPointArray, YawPointArray;
    DriveTrain *DRObj;
    uint8_t currentPointNumber, pointArraySize;
    int XLocationData, YLocationData, YawStatsData; //取得する現在位置の変数
    int XTargetData, YTargetData, YawTargetData;
    int XCurrentLocation, YCurrentLocation, YawCurrentStats;
};

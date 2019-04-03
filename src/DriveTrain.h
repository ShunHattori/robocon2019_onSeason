#pragma once

#include "mbed.h"
#include "encoder.h"
#include "MWodometry.h"
#include <vector>

template <typename TYPE>
class DriveTrain
{

  public:
    /*
    *   desc: コンストラクタ
    */
    DriveTrain(TYPE x, TYPE y, TYPE yaw)
    {
        XPoint.resize(20);
        YPoint.resize(20);
        YawData.resize(20);
        XPoint.insert(XPoint.begin(), x);
        YPoint.insert(YPoint.begin(), y);
        YawData.insert(YawData.begin(), yaw);
    }

    /*
    *   desc:   指定した位置に移動
    *   param:  移動先X座標,Y座標
    *   return: none
    */
    void addPoint(TYPE x, TYPE y)
    {
        XPoint.push_back(x);
        YPoint.push_back(y);
        YawData.push_back(getYawData());
    }

    /*
    *   desc:   指定した位置に移動
    *   param:  移動先X座標,Y座標,YAW軸角度
    *   return: none
    */
    void addPoint(TYPE x, TYPE y, TYPE yaw)
    {
        XPoint.push_back(x);
        YPoint.push_back(y);
        YawData.push_back(yaw);
    }

    /*
    *   desc:   先頭のデータを取得
    *   param:  Vector型のオブジェクト
    *   return: 引数のオブジェクトの先頭要素
    */
    int getXData()
    {
        return XPoint.front();
    }
    int getYData()
    {
        return YPoint.front();
    }
    int getYawData()
    {
        return YawData.front();
    }
    /*
    *   desc:   自己位置を更新
    *   param:  更新後X座標,Y座標
    *   return: none
    */
    void setCurrentPoint(TYPE x, TYPE y, TYPE yaw)
    {
        XPoint.insert(XPoint.begin(), x);
        YPoint.insert(YPoint.begin(), y);
        YawData.insert(YawData.begin(), yaw);
    }

  private:
    std::vector<int> XPoint, YPoint, YawData;
};

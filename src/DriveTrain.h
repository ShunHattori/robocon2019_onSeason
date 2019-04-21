#pragma once

#include "mbed.h"
#include "math.h"
#include "MWodometry.h"
#include "LocationManager.h"

class DriveTrain
{
  public:
    DriveTrain(LocationManager<int> &lcmObj, MWodometry &X1, MWodometry &X2, MWodometry &Y1, int ENCATCdiff, int AllocateError, int DecreaseRadius)
    {
        LCM = &lcmObj;
        XAxis_1 = &X1;
        SubXAxis = &X2;
        YAxis_1 = &Y1;
        encoderAttachDiff = ENCATCdiff;
        allocateError = AllocateError;
        decreaseRadius = DecreaseRadius;
        Max = 0.3;
        Min = 0.05;
        ConfirmStatsInitialFlag = 1;
        encoderMode = 1;
        currentX = 0;
        currentY = 0;
        currentYaw = 0;
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
        return stats;
    }

    /*
     * desc:   各軸の出力を取得する
     * param:  none
     * return: 各軸の出力値(min~max内)
     */
    double getXVector()
    {
        return Vec[0];
    }
    double getYVector()
    {
        return Vec[1];
    }
    double getYawVector()
    {
        return Vec[2];
    }

    void setSensorDistance(int distance)
    {
        sensorDistance = distance;
    }

    void switchMode()
    {
        encoderMode = !encoderMode;
    }

    /*
     * desc:   現在のロボットの位置を取得する
     * param:  none
     * return: 各軸座標(double)
     */
    double getCurrentXPosition()
    {
        return currentX;
    }
    double getCurrentYPosition()
    {
        return currentY;
    }
    double getCurrentYawPosition()
    {
        return currentYaw;
    }

    /*
     * desc:   現在のロボットの位置を上書きする
     * param:  各軸座標(double)
     * return: none
     */
    void setCurrentXPosition(double position)
    {
        currentX = position;
    }
    void setCurrentYPosition(double position)
    {
        currentY = position;
    }
    void setCurrentYawPosition(double position)
    {
        currentYaw = position;
    }

    /*
     * desc:   出力の最大値を設定する
     * param:  最大値
     * return: none
     */
    void setMaxOutput(double max)
    {
        Max = max;
    }

    /*
     * desc:   出力の最小値を設定する
     * param:  最小値
     * return: none
     */
    void setMinOutput(double min)
    {
        Min = min;
    }

  private:
    LocationManager<int> *LCM;
    MWodometry *XAxis_1, *SubXAxis, *YAxis_1;
    Timer ConfirmStats;
    /*
     *   current~ : 現在のロボットの位置を保存している
     *   target~  : ロボットの目標位置
     *   error~   : 目標座標と現在位置の偏差
     *   stats    : フラグ(1=移動完了,0=移動中)
     *   encoderAttachDiff   :   同軸の計測輪取り付け距離
     *   XEncodedDistanceDiff:   同軸エンコーダ間の誤差　これからYAWを計算
     *   allocateError       :   停止地点の許容誤差
     *   decreaseRadius      :   減速開始判定円の半径
     */
    double currentX, currentY,tempX,tempSub,tempY;
    double targetX, targetY, XEncodedDistanceDiff;
    double errorX, errorY, errorYaw;
    bool stats;
    double currentYaw, targetYaw;
    float encoderAttachDiff;
    float allocateError, decreaseRadius;

    float Vec[3];
    float sensorDistance;
    float Max, Min;
    float xReachedMaxPWM, yReachedMaxPWM;
    bool ConfirmStatsInitialFlag, encoderMode;

    float mapFloat(float value, float in_min, float in_max, float out_min, float out_max);
};

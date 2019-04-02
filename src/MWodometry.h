#pragma once

#include "mbed.h"
#include "math.h"
#include "Encoder.h"

class MWodometry
{
  public:
    /*
    *   param:  Encoder型のオブジェクト,エンコーダの分解能
    *   return: none 
    */
    MWodometry(Encoder &_obj, uint16_t res, uint8_t wheRad) : encObject(&_obj), encoderResolution(res), wheelRadius(wheRad){}; //コンストラクタ
    ~MWodometry(void);                                                                        //デストラクタ

    long getDistance(void)const;
    bool setDistance(long);

  private:
    Encoder *encObject;
    uint16_t encoderResolution;
    int16_t distance;
    uint8_t wheelRadius;
};

#pragma once

#include "math.h"
#include "mbed.h"

/*
 *  前回の値取得から一定時間後に値を返す
 *  周期を10ms, 返却値を0.585に設定
 *  0ms->return 0.585 ... 1~9ms return 0 ...10ms return 0.585 ...   
 */

class TimeIncreaser
{
public:
  TimeIncreaser(const double intervalTime, const double returnValue) //intervalTime in ms
  {
    time = (int)(intervalTime * 1000); // ms to us
    value = returnValue;
    intervalTimer.start();
  }

  void start()
  {
    intervalTimer.start();
  }

  void stop()
  {
    intervalTimer.stop();
  }

  void reset()
  {
    intervalTimer.reset();
  }

  /*
   *    add or subtract internal value;
   */
  void modifyValue(const double amount)
  {
    value += amount;
  }
  void setValue(const double amount)
  {
    value = amount;
  }

  void setTime(const double amount)
  {
    time = amount;
  }

  double getValue()
  {
    if (time < intervalTimer.read_us())
    {
      intervalTimer.reset();
      return value;
    }
    return 0;
  }

private:
  Timer intervalTimer;
  int time;
  double value;
};
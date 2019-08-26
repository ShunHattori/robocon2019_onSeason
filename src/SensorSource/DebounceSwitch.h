#pragma once

#include "mbed.h"

class DebounceSwitch
{
public:
  typedef enum Pinmode {
    PULLUP,
    PULLDOWN,
  } Pinmode;

  DebounceSwitch(PinName, Pinmode);
  void update();
  void setDetectCount(int);
  bool stats()
  {
    return ButtonStats;
  }

private:
  Timer samplingTimer;
  DigitalIn _switch;
  int detectFlagNumber, samplingPeriod, mode, buttonPressCount;
  bool ButtonStats;
};
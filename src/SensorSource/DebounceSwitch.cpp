#include "DebounceSwitch.h"

DebounceSwitch::DebounceSwitch(PinName inputPin, Pinmode pinmode) : _switch(inputPin)
{
  if (pinmode == PULLUP)
  {
    _switch.mode(PullUp);
  }
  else if (pinmode == PULLDOWN)
  {
    _switch.mode(PullDown);
  }
  else
  {
    _switch.mode(PullNone);
  }
  mode = pinmode;
  samplingTimer.start();
  samplingPeriod = 10;
  detectFlagNumber = 3;
  ButtonStats = 0;
}

void DebounceSwitch::update()
{
  static int buttonPressCount = 0;

  if (samplingPeriod < samplingTimer.read_ms()) //サンプリング方式
  {
    switch (mode) //カウントアップ方式
    {
    case PULLUP:
      if (!_switch.read())
      {
        buttonPressCount++;
      }
      else
        buttonPressCount = 0;
      break;
    case PULLDOWN:
      if (_switch.read())
      {
        buttonPressCount++;
      }
      else
        buttonPressCount = 0;
      break;
    }
    samplingTimer.reset();
  }

  if (detectFlagNumber < buttonPressCount)
  {
    ButtonStats = 1;
  }
  else
  {
    ButtonStats = 0;
  }
}

void DebounceSwitch::setDetectCount(int count)
{
  detectFlagNumber = count;
}
#include "Peg.h"

Peg::Peg(double pwm, double movingTime, double* variableToStore)
{
  timer = new Timer();
  timer->start();
  outputPWM = variableToStore;
  maxPwm = pwm;
  timePerOnce = movingTime;
}

void Peg::launch(void)
{
  if (launchFlag == 0)
  {
    launchFlag = 1;
    timer->reset();
  }
}

void Peg::reload(void)
{
  if (reloadFlag == 0)
  {
    reloadFlag = 1;
    timer->reset();
  }
}
void Peg::update(void)
{
  if (launchFlag)
  {
    if (timer->read() < timePerOnce)
    {
      extendFlag = 1;
      reduceFlag = 0;
    }
    else if (extendFlag && timer->read() > timePerOnce)
    {
      extendFlag = 0;
      launchFlag = 0;
    }
  }
  if (reloadFlag)
  {
    if (timer->read() < timePerOnce)
    {
      reduceFlag = 1;
      extendFlag = 0;
    }
    else if (reduceFlag && timer->read() > timePerOnce / 2)
    {
      reduceFlag = 0;
      reloadFlag = 0;
    }
  }
  if (extendFlag)
  {
    outputPWM[0] = maxPwm;
    outputPWM[1] = 0;
  }
  else if (reduceFlag)
  {
    outputPWM[0] = 0;
    outputPWM[1] = maxPwm / 2;
  }
  else
  {
    outputPWM[0] = 0;
    outputPWM[1] = 0;
  }
}
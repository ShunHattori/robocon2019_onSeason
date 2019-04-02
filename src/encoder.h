#pragma once

#include <mbed.h>

class Encoder
{

public:
  Encoder(PinName APulsePin, PinName BPulsePin); //constructor A,B相のピンを指定
  ~Encoder();                                    //destructor

  bool EnableDebugOutput(PinName); //割り込みピン,出力ピンを指定
  long getPulse(void);             //検出パルスを取得
  bool setPulse(long);             //メンバ変数を更新

private:
  bool init(void);
  void update(void);
  void toggleDebugOutput(void);
  void encA_riseHandler(void);
  void encoderUpdate(void);

  InterruptIn *encA, *encB, *encObject;
  DigitalOut *debugOutput;
  PinName debugPin, ApulsePin, BpulsePin;
  volatile long pulse;
  volatile uint8_t currentAPulse, currentBPulse, currentABPulse;
};
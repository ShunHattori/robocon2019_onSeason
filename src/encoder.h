#include <mbed.h>

class Encoder
{

  public:
    Encoder(PinName APulsePin, PinName BPulsePin);  //constructor A,B相のピンを指定
    ~Encoder();                                     //destructor

    bool setBebugOutput(PinName, PinName);  //割り込みピン,出力ピンを指定
    long getPulse(void);  //検出パルスを取得
    bool setPulse(long);  //メンバ変数を更新

  private:

    void update(void);
    void debugUpdate(void);
    void encA_riseHandler(void);
    void encA_fallHandler(void);
    void encB_riseHandler(void);
    void encB_fallHandler(void);

    InterruptIn *encA, *encB, *encObject;
    DigitalOut *debugOut;
    PinName debugPin, interruptPin;
    long pulse;

};
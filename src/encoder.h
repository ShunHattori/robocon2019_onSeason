#include <mbed.h>

class encoder
{

  public:
    encoder(InterruptIn &_enc);
    ~encoder();

    long getPulse(void);
    void setPulse(long);

  private:

    void update(void);

    InterruptIn *encObject;
    long pulse;
    uint8_t stats;
};
#include "encoder.h"

encoder::encoder(InterruptIn &_enc)
{
    encObject = &_enc;
    encObject->rise(callback(this,&encoder::update));
}

long encoder::getPulse()
{
    return pulse;
}

void encoder::setPulse(long _pulse)
{
    pulse = _pulse;
}

void encoder::update(){
    stats = !stats;
}
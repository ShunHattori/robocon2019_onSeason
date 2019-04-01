#include "Encoder.h"

Encoder::Encoder(PinName APulsePin, PinName BPulsePin)
{
    encA = new InterruptIn(APulsePin);
    encB = new InterruptIn(BPulsePin);
    encA->mode(PullUp);
    encB->mode(PullUp);
    encA->rise(callback(this, &Encoder::encA_riseHandler));
    encA->fall(callback(this, &Encoder::encA_fallHandler));
    encB->rise(callback(this, &Encoder::encB_riseHandler));
    encB->fall(callback(this, &Encoder::encB_fallHandler));
    pulse = 0;
}

Encoder::~Encoder()
{
    printf("Instance removed.");
}

void Encoder::encA_riseHandler()
{
    if(encB->read()) pulse++;
    else pulse--;
}

void Encoder::encA_fallHandler()
{
    if(encB->read()) pulse--;
    else pulse++;
}

void Encoder::encB_riseHandler()
{
    if(encA->read()) pulse--;
    else pulse++;
}

void Encoder::encB_fallHandler()
{
    if(encA->read()) pulse++;
    else pulse--;
}

bool Encoder::setBebugOutput(PinName _irtPin, PinName _pin)
{
    debugPin = _pin;
    interruptPin = _irtPin;
    encObject = new InterruptIn(interruptPin);
    debugOut = new DigitalOut(debugPin);
    encObject->rise(callback(this, &Encoder::debugUpdate));
    return 1;
}

long Encoder::getPulse()
{
    return pulse;
}

bool Encoder::setPulse(long _pulse)
{
    pulse = _pulse;
    return 1;
}

void Encoder::debugUpdate()
{
    if (debugOut != NULL)
    {
        *debugOut = !debugOut;
    }
}
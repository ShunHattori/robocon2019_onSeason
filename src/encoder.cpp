#include "Encoder.h"

Encoder::Encoder(PinName _APulsePin, PinName _BPulsePin)
{
    ApulsePin = _APulsePin;
    BpulsePin = _BPulsePin;
    while (!Encoder::init())
    {
    }
}

Encoder::~Encoder()
{
    printf("Instance removed.");
}

bool Encoder::init()
{
    encA = new InterruptIn(ApulsePin);
    encB = new InterruptIn(BpulsePin);
    encA->rise(callback(this, &Encoder::encoderUpdate));
    encA->fall(callback(this, &Encoder::encoderUpdate));
    encB->rise(callback(this, &Encoder::encoderUpdate));
    encB->fall(callback(this, &Encoder::encoderUpdate));
    pulse = 0;
    return 1;
}

void Encoder::encoderUpdate()
{
    static volatile uint8_t prevAB = 0;
    currentAPulse = encA->read();
    currentBPulse = encB->read();
    currentABPulse = (currentAPulse << 1) | currentBPulse;
    if ((prevAB == 0x3 && currentABPulse == 0x0) || (prevAB == 0x0 && currentABPulse == 0x3))
    {
        pulse++;
    }
    else if ((prevAB == 0x2 && currentABPulse == 0x1) || (prevAB == 0x01 && currentABPulse == 0x2))
    {
        pulse--;
    }
    prevAB = currentABPulse;
}

bool Encoder::EnableDebugOutput(PinName _pin)
{
    debugPin = _pin;
    debugOutput = new DigitalOut(debugPin);
    encObject->rise(callback(this, &Encoder::toggleDebugOutput));
    return 1;
}

long Encoder::getPulse()
{
    return pulse * 2; //二逓倍
}

bool Encoder::setPulse(long _pulse)
{
    pulse = _pulse / 2; //二逓倍
    return 1;
}

void Encoder::toggleDebugOutput()
{
    if (debugOutput != NULL)
    {
        *debugOutput = !debugOutput;
    }
}
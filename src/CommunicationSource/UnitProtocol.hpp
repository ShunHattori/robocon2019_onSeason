#pragma once
#include "mbed.h"

typedef enum ASCII
{
    STX = 0x02, //Start of Text
    ETX = 0x03, //End of Text
    ENQ = 0x05, //Enquiry
    ACK = 0x06, //Acknowledge
} ControlCodes;

class UnitProtocol
{
public:
    UnitProtocol(PinName uartTX, PinName uartRX, int uartBaud);
    bool transmit(int arrayLenght, uint8_t *packet);
    bool receive(uint8_t *variableToStore);
    void setTimeout(uint8_t timeoutTimeInMs);
    void addDataFlowLED(PinName LEDPin, char *whichDataFlow);

private:
    Serial *_PORT;
    Timer *_watchDogTimer;
    int _baudPort, _timeoutMs, _arrayLenght;
};
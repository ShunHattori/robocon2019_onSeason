/*
    TODO:ENQsend,ACKreceiveが正常に完了したか、タイムアウトしたかを返す戻り値をつける(0,1)
         データの送受信時に一瞬点灯するLEDの追加
 */

#pragma once
#include "mbed.h"

typedef enum ASCII {
  STX = 0x02, //Start of Text
  ETX = 0x03, //End of Text
  ENQ = 0x05, //Enquiry
  ACK = 0x06, //Acknowledge
} ControlCodes;

class UnitProtocol
{
public:
  UnitProtocol(PinName uartTX, PinName uartRX, int uartBaud);
  int transmit(int arrayLenght, int *packet);
  int receive(int *variableToStore);
  void setTimeout(int timeoutTimeInMs);
  void addDataFlowLED(PinName LEDPin, char *whichDataFlow);

private:
  Serial *_PORT;
  Timer *_watchDogTimer;
  DigitalOut *_TXLED, *_RXLED;
  PinName _TXLEDPin, _RXLEDPin;
  int _baudPort, _timeoutMs, _arrayLenght;
  bool _isTransmittable, _isReceivable;

  void ENQsend(char data);
  char ACKreceive();
};
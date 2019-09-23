#include "UnitProtocol.hpp"

UnitProtocol::UnitProtocol(PinName uartTX, PinName uartRX, int uartBaud)
{
  _baudPort = uartBaud;
  _PORT = new Serial(uartTX, uartRX, _baudPort);
  _watchDogTimer = new Timer();
  _watchDogTimer->start();
  _timeoutMs = 20; //set the timeout time to 20ms
}

bool UnitProtocol::transmit(int arrayLenght, uint8_t *packet)
{
  _arrayLenght = arrayLenght;
  _PORT->putc(ENQ);
  _watchDogTimer->reset();
  while (1)
  {
    if (_watchDogTimer->read_ms() > _timeoutMs)
    {
      return 0;
    }
    if (!(_PORT->readable()))
    {
      continue;
    }
    if (_PORT->getc() == ACK)
    {
      break;
    }
  }
  _PORT->putc(_arrayLenght);
  char _checkSum = 0;
  for (int i = 0; i < arrayLenght; i++)
  {
    _PORT->putc(packet[i]);
    _checkSum ^= packet[i];
  }
  _PORT->putc(_checkSum);
  return 1;
}

bool UnitProtocol::receive(uint8_t *variableToStore)
{
  if (!(_PORT->readable()))
  {
    return 0;
  }
  if (_PORT->getc() != ENQ)
  {
    return 0;
  }
  _PORT->putc(ACK);
  int _incomingCounter = 0;
  char _buffer[500], _bufferSum = 0;
  _watchDogTimer->reset();
  while (1)
  {
    if (_watchDogTimer->read_ms() > _timeoutMs)
    {
      return 0;
    }
    if (!(_PORT->readable()))
    {
      continue;
    }
    break;
  }
  _arrayLenght = _PORT->getc();
  for (int i = 0; i < _arrayLenght; i++)
  {
    _watchDogTimer->reset();
    while (1)
    {
      if (_watchDogTimer->read_ms() > _timeoutMs)
      {
        return 0;
      }
      if (!(_PORT->readable()))
      {
        continue;
      }
      _buffer[_incomingCounter++] = _PORT->getc();
      break;
    }
  }
  if (_arrayLenght != _incomingCounter)
  {
    return 0;
  }
  for (int i = 0; i < _incomingCounter; i++)
  {
    _bufferSum ^= _buffer[i];
  }
  while (1)
  {
    if (_watchDogTimer->read_ms() > _timeoutMs)
    {
      return 0;
    }
    if (!(_PORT->readable()))
    {
      continue;
    }
    if (_PORT->getc() != _bufferSum)
    {
      return 0;
    }
    break;
  }
  for (int i = 0; i < _incomingCounter; i++)
  {
    variableToStore[i] = _buffer[i];
  }
  return 1;
}

void UnitProtocol::setTimeout(uint8_t timeoutTimeInMs)
{
  _timeoutMs = timeoutTimeInMs < 10 ? 10 : timeoutTimeInMs; //minimum = 10ms
}
#include "UnitProtocol.hpp"

UnitProtocol::UnitProtocol(PinName uartTX, PinName uartRX, int uartBaud)
{
  _baudPort = uartBaud;
  _PORT = new Serial(uartTX, uartRX, _baudPort);
  _watchDogTimer = new Timer();
  _watchDogTimer->start();
  _timeoutMs = 20; //set the timeout time to 20ms
  _isTransmittable = 1;
  _isReceivable = 1;
}

int UnitProtocol::transmit(int arrayLenght, int *packet)
{
  _arrayLenght = arrayLenght;
  _PORT->putc(ENQ);        //送信してもいいですか
  _watchDogTimer->reset(); //タイムアウト用タイマーリセット
  while (1)                //返事待ち
  {
    if (_watchDogTimer->read_ms() > _timeoutMs) //_timeoutMs間返答なし
    {
      return 0; //タイムアウト
    }
    if (!(_PORT->readable())) //バッファに何もない
    {
      continue; //タイムアウト判定に戻る
    }
    if (_PORT->getc() == ACK) //バッファの中身がACKだったら
    {
      break; //次へゴー！
    }
  }
  _PORT->putc(_arrayLenght); //データの長さはこれだけです。
  char _checkSum = 0;
  for (int i = 0; i < arrayLenght; i++)
  {
    _PORT->putc(packet[i]); //データの中身
    _checkSum ^= packet[i]; //チェックサム計算
  }
  _PORT->putc(_checkSum); //これと比較して破損確認
  return 1;
}

int UnitProtocol::receive(int *variableToStore)
{
  if (!(_PORT->readable())) //バッファに何もない
  {
    return 0;
  }
  if (_PORT->getc() != ENQ) //中身がENQじゃなかったら
  {
    return 0;
  }
  _PORT->putc(ACK); //中身がENQじゃなかったらACK送信
  int _incomingCounter = 0;
  char _buffer[500], _bufferSum = 0;
  _watchDogTimer->reset();
  while (1) //返事待ち
  {
    if (_watchDogTimer->read_ms() > _timeoutMs) //_timeoutMs間返答なし
    {
      return 0; //タイムアウト
    }
    if (!(_PORT->readable())) //バッファに何もない
    {
      continue; //タイムアウト判定に戻る
    }
    break; //次へゴー！
  }
  _arrayLenght = _PORT->getc();
  for (int i = 0; i < _arrayLenght; i++)
  {
    _watchDogTimer->reset();
    while (1)
    {
      if (_watchDogTimer->read_ms() > _timeoutMs) //_timeoutMs間返答なし
      {
        return 0; //タイムアウト
      }
      if (!(_PORT->readable())) //バッファに何もない
      {
        continue; //タイムアウト判定に戻る
      }
      _buffer[_incomingCounter++] = _PORT->getc();
      break; //次へゴー！
    }
  }
  if (_arrayLenght != _incomingCounter) //あらかじめ教えられていたデータ長と実際に来たデータ長が違う場合を弾く
  {
    return 0;
  }
  for (int i = 0; i < _incomingCounter; i++)
  {
    _bufferSum ^= _buffer[i];
  }
  while (1)
  {
    if (_watchDogTimer->read_ms() > _timeoutMs) //_timeoutMs間返答なし
    {
      return 0; //タイムアウト
    }
    if (!(_PORT->readable())) //バッファに何もない
    {
      continue; //タイムアウト判定に戻る
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

void UnitProtocol::setTimeout(int timeoutTimeInMs)
{
  _timeoutMs = timeoutTimeInMs;
}

void UnitProtocol::addDataFlowLED(PinName LEDPin, char *whichDataFlow)
{
}
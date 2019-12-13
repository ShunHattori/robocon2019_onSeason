#include "MPU9250.h"
#include "mbed.h"

MPU9250::MPU9250(PinName sda, PinName scl, int baud)
{
  init();
  i2c_ = new I2C(sda, scl);
  i2c_->frequency(baud);
  timer_ = new Timer();
  timer_->start();
}

void MPU9250::init()
{
  offsetGyroZ = 0;
  lapTime = 0;
  prevTime = 0;
}

void MPU9250::setup()
{
  writeByte(gyroAddress, 0x6B, 0x00);
  writeByte(gyroAddress, 0x37, 0x02);

  while (readByte(gyroAddress, regWAI) != 0x71)
  {
    printf("BOOT FAILED. (WAI)");
  }

  int16_t rawGyroZ = 0;

  for (int i = 0; i < 3000; i++) // loop for calclation offsets 3000times
  {
    uint8_t readGyro[2];

    readBytes(gyroAddress, regYaw, 2, &readGyro[0]);
    rawGyroZ = (int16_t)(((int16_t)readGyro[0] << 8) | readGyro[1]);

    offsetGyroZ += (double)rawGyroZ / 131.0;
  }
  offsetGyroZ /= 3000;
}

void MPU9250::update()
{
  uint8_t gyroRead[2];
  readBytes(gyroAddress, regYaw, 2, &gyroRead[0]);

  gyroZ = (int16_t)(((int16_t)gyroRead[0] << 8) | gyroRead[1]);

  lapTime = timer_->read_us() - prevTime;
  prevTime = timer_->read_us();

  dpsGyroZ = (double)gyroZ / 131.0;
  if (!(dpsGyroZ - offsetGyroZ < 1 && dpsGyroZ - offsetGyroZ > -1)) //角速度が1[degree/s]に満たない場合計算しない
    yaw += (dpsGyroZ - offsetGyroZ) * lapTime * 0.000001;
}

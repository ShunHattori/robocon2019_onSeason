#pragma once

#include "mbed.h"

// Address of MPUs
#define gyroAddress 0x68 << 1 // address of MPU9025(加速度、ジャイロ)

// Read Registers
#define regYaw 0x47 // register of gyro_Z_H
#define regWAI 0x75 // register of "Who am I"

class MPU9250
{
public:
  MPU9250(PinName, PinName, int);
  void setup(); //オフセット値の計算
  void update();
  double getYaw() { return yaw; }
  double getOffsetYaw() { return offsetGyroZ; }

  void setYaw(double targetYaw)
  {
    yaw = targetYaw;
  }
  void setOffsetYaw(double targetOffset)
  {
    offsetGyroZ = targetOffset;
  }

private:
  I2C *i2c_;
  Timer *timer_;

  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
  {
    char data_write[2];
    data_write[0] = subAddress;
    data_write[1] = data;
    i2c_->write(address, data_write, 2, 0);
  }

  char readByte(uint8_t address, uint8_t subAddress)
  {
    char data[1]; // `data` will store the register data
    char data_write[1];
    data_write[0] = subAddress;
    i2c_->write(address, data_write, 1, 1); // no stop
    i2c_->read(address, data, 1, 0);
    return data[0];
  }

  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
  {
    char data[14];
    char data_write[1];
    data_write[0] = subAddress;
    i2c_->write(address, data_write, 1, 1); // no stop
    i2c_->read(address, data, count, 0);
    for (int i = 0; i < count; i++) {
      dest[i] = data[i];
    }
  }

  void init();
  double offsetGyroZ, dpsGyroZ, yaw;
  uint64_t lapTime, prevTime;
  int16_t gyroZ;
};
#ifndef MPU9250_h
#define MPU9250_h

#include <math.h>
#include "mbed.h"

// Address of MPUs
#define gyro_address 0x68 << 1 //adress of MPU9025(加速度、ジャイロ)
#define mag_address 0x0c << 1  //adress of AK8963(地磁気)

// Read Registers
#define reg_accel_X 0x3B //register of accel_X_H 未使用
#define reg_accel_Y 0x3D //register of accel_Y_H 未使用
#define reg_accel_Z 0x3f //register of accel_Z_H
#define reg_temp 0x41    //register of temp 未使用
#define reg_roll 0x43    //register of gyro_X_H 未使用
#define reg_pitch 0x45   //register of gyro_Y_H 未使用
#define reg_yaw 0x47     //register of gyro_Z_H
#define reg_WAI 0x75     //register of "Who am I"
#define reg_mag 0x03     //reister of Magnetometer

class MPU9250
{
public:
  MPU9250(PinName, PinName, int);
  void setup();                                                            //オフセット値の計算
  void read_accel(double *accel_roll, double *accel_pitch);                //指定したアドレスに加速度センサから算出したroll軸,pitch軸の回転角を返す
  void read_gyro(double *gyro_roll, double *gyro_pitch, double *gyro_yaw); //指定したアドレスにジャイロセンサから算出した各軸の回転角を返す
  void read_compass(double *Mx, double *My, double *Mz);                   //指定したアドレスに地磁気センサの生値を返す
  double complement_Yaw();                                                 //各センサから補正したYaw軸の回転角を返す
  double gyro_Yaw();                                                       //ジャイロセンサから算出したYaw軸の回転角を返す
  double compass_Yaw();                                                    //地磁気センサから算出したYaw軸の回転角を返す
  double getYaw() { return yaw; }
  void setYaw(double targetYaw) { yaw = targetYaw; }

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
    for (int ii = 0; ii < count; ii++)
    {
      dest[ii] = data[ii];
    }
  }
  void init();

  double offset_gx, offset_gy, offset_gz;
  double offset_mx, offset_my, offset_mz;
  double offset_mag, offset_mag_plus, offset_mag_minus;
  double gyro_roll, gyro_pitch, gyro_yaw;
  double Mx, My, Mz;
  double compass_angle;
  double yaw;

  double complement_angle;
  int16_t ax, ay, az;
  double accel_roll, accel_pitch;
  unsigned long int elapsed_time, preterit_time;
  int16_t gx, gy, gz;
  double dps_gx, dps_gy, dps_gz;
  int16_t mx, my, mz;
  int16_t tmx, tmy, tmz;
};
#endif

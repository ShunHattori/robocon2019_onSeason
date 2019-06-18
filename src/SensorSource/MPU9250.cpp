#include "mbed.h"
#include "MPU9250.h"
#include <math.h>

void MPU9250::init()
{
  offset_gx = 0;
  offset_gy = 0;
  offset_gz = 0;
  offset_mx = 0;
  offset_my = 0;
  offset_mz = 0;
  gyro_roll = 0;
  gyro_pitch = 0;
  gyro_yaw = 0;
  elapsed_time = 0;
  preterit_time = 0;
  tmx = 0;
  tmy = 0;
  tmz = 0;
}

void MPU9250::setup(PinName sda, PinName scl)
{
  init();
  i2c_ = new I2C(sda, scl);
  i2c_->frequency(400000);
  pc_ = new Serial(USBTX, USBRX);
  timer_ = new Timer();
  timer_->start();

  writeByte(gyro_address, 0x6B, 0x00);
  writeByte(gyro_address, 0x37, 0x02);
  writeByte(mag_address, 0x0A, 0x06);

  while (readByte(gyro_address, reg_WAI) != 0x71)
  {
    pc_->printf("WAI = %d", readByte(gyro_address, reg_WAI));
  }

  int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;
  int16_t raw_mx = 0, raw_my = 0, raw_mz = 0;
  for (int i = 0; i < 3000; i++) // loop for calclation offsets 3000times
  {
    uint8_t readGyro[6];
    uint8_t magData[7];
    readBytes(gyro_address, reg_roll, 6, &readGyro[0]);
    raw_gx = (int16_t)(((int16_t)readGyro[0] << 8) | readGyro[1]);
    raw_gy = (int16_t)(((int16_t)readGyro[2] << 8) | readGyro[3]);
    raw_gz = (int16_t)(((int16_t)readGyro[4] << 8) | readGyro[5]);
    //pc_->printf("%d\t", raw_gx);
    //pc_->printf("%d\t", raw_gy);
    //pc_->printf("%d\t", raw_gz);

    readBytes(mag_address, reg_mag, 7, &magData[0]);

    uint8_t c = magData[6];
    if (!(c & 0x08))
    {
      raw_mx = (int16_t)(((int16_t)magData[1] << 8) | magData[0]);
      raw_my = (int16_t)(((int16_t)magData[3] << 8) | magData[2]);
      raw_mz = (int16_t)(((int16_t)magData[5] << 8) | magData[4]);
    }
    //pc_->printf("%d\t", raw_mx);
    //pc_->printf("%d\t", raw_my);
    //pc_->printf("%d\r\n", raw_mz);

    offset_gx += raw_gx / 131.0;
    offset_gy += raw_gy / 131.0;
    offset_gz += raw_gz / 131.0;
    offset_mx += (double)raw_mx * 0.15;
    offset_my += (double)raw_my * 0.15;
    offset_mz += (double)raw_mz * 0.15;
  }

  offset_gx /= 3000;
  offset_gy /= 3000;
  offset_gz /= 3000;
  offset_mx /= 3000;
  offset_my /= 3000;
  offset_mz /= 3000;
  pc_->printf("--OFFSETS--\r\n");
  pc_->printf("%.2lf\r\n", offset_gx);
  pc_->printf("%.2lf\r\n", offset_gy);
  pc_->printf("%.2lf\r\n", offset_gz);
  pc_->printf("%.2lf\r\n", offset_mx);
  pc_->printf("%.2lf\r\n", offset_my);
  pc_->printf("%.2lf\r\n", offset_mz);

  offset_mag = atan2(offset_mx, offset_my) * 180 / 3.141593;
  if (offset_mag >= 0)
  {
    offset_mag_plus = offset_mag;
    offset_mag_minus = offset_mag - 180;
  }
  else
  {
    offset_mag_plus = offset_mag + 180;
    offset_mag_minus = offset_mag;
  }
}

void MPU9250::complement_Yaw()
{
  //gyro_yaw = gyro_Yaw();
  //compass_angle = compass_Yaw();
  //complement_angle = 0.9 * gyro_yaw + 0.1 * compass_angle; //値のブレが大きい場合は適宜調整してくれ
  //yaw = complement_angle;
  yaw = gyro_Yaw();
}

void MPU9250::read_accel(double *accel_roll, double *accel_pitch)
{
  uint8_t gyroRead[6];
  readBytes(gyro_address, reg_accel_X, 6, &gyroRead[0]);

  ax = (int16_t)(((int16_t)gyroRead[0] << 8) | gyroRead[1]);
  ay = (int16_t)(((int16_t)gyroRead[2] << 8) | gyroRead[3]);
  az = (int16_t)(((int16_t)gyroRead[4] << 8) | gyroRead[5]);

  *accel_roll = atan2(ay, az);
  *accel_pitch = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2)));
}

void MPU9250::read_gyro(double *gyro_roll, double *gyro_pitch, double *gyro_yaw)
{
  static double roll = 0, pitch = 0, yaw = 0;
  uint8_t gyroRead[6];
  readBytes(gyro_address, reg_roll, 6, &gyroRead[0]);

  gx = (int16_t)(((int16_t)gyroRead[0] << 8) | gyroRead[1]);
  gy = (int16_t)(((int16_t)gyroRead[2] << 8) | gyroRead[3]);
  gz = (int16_t)(((int16_t)gyroRead[4] << 8) | gyroRead[5]);

  elapsed_time = timer_->read_ms() - preterit_time;
  preterit_time = timer_->read_ms();

  dps_gx = (double)gx / 131;
  dps_gy = (double)gy / 131;
  dps_gz = (double)gz / 131;

  read_accel(&accel_roll, &accel_pitch);
  accel_roll = accel_roll * 180 / 3.141593;
  accel_pitch = accel_pitch * 180 / 3.141593;
  if (!(dps_gx - offset_gx < 1 && dps_gx - offset_gx > -1))
    roll += (dps_gx - offset_gx) * elapsed_time * 0.001;

  if (!(dps_gy - offset_gy < 1 && dps_gy - offset_gy > -1))
    pitch += (dps_gy - offset_gy) * elapsed_time * 0.001;

  if (!(dps_gz - offset_gz < 1 && dps_gz - offset_gz > -1))
    yaw += (dps_gz - offset_gz) * elapsed_time * 0.001;

  *gyro_roll = 0.995 * roll + (0.005 * accel_roll);
  *gyro_pitch = 0.995 * pitch + (0.005 * accel_pitch);

  if (yaw > 180)
  {
    *gyro_yaw = -180 + (yaw - 180);
    if (yaw > 360)
      yaw = yaw - 360;
  }
  else if (yaw < -180)
  {
    *gyro_yaw = 180 + (yaw + 180);
    if (yaw < 360)
      yaw = yaw + 360;
  }
  else
    *gyro_yaw = yaw;
}

double MPU9250::gyro_Yaw()
{
  static double yaw = 0, yaw_angle = 0;
  uint8_t gyroRead[2];
  readBytes(gyro_address, reg_yaw, 2, &gyroRead[0]);

  gz = (int16_t)(((int16_t)gyroRead[0] << 8) | gyroRead[1]);

  elapsed_time = timer_->read_ms() - preterit_time;
  preterit_time = timer_->read_ms();

  dps_gz = (double)gz / 131;
  if (!(dps_gz - offset_gz < 1 && dps_gz - offset_gz > -1)) //角速度が1[degree/s]に満たない場合計算しない
    yaw += (dps_gz - offset_gz) * elapsed_time * 0.001;

  if (yaw > 180)
  { //０°~180°,-180°~0°に変換
    yaw_angle = -180 + (yaw - 180);
    if (yaw > 360)
      yaw = yaw - 360;
  }
  else if (yaw < -180)
  {
    yaw_angle = 180 + (yaw + 180);
    if (yaw < -360)
      yaw = yaw + 360;
  }
  else
    yaw_angle = yaw;
  return yaw_angle;
}

void MPU9250::read_compass(double *MX, double *MY, double *MZ)
{
  uint8_t magRead[7];
  readBytes(mag_address, reg_mag, 7, &magRead[0]);
  if ((uint8_t)magRead[6] != 0x08)
  {
    mx = (int16_t)(((int16_t)magRead[1] << 8) | magRead[0]);
    my = (int16_t)(((int16_t)magRead[3] << 8) | magRead[2]);
    mz = (int16_t)(((int16_t)magRead[5] << 8) | magRead[4]);
    tmx = (double)mx * 0.15; //生値の単位を[T]に変更
    tmy = (double)my * 0.15;
    tmz = (double)mz * 0.15;
  }

  *MX = tmx;
  *MY = tmy;
  *MZ = tmz;
}

double MPU9250::compass_Yaw()
{
  double yaw;
  uint8_t magRead[7];
  readBytes(mag_address, reg_mag, 7, &magRead[0]);

  uint8_t c = magRead[6];
  if (c != 0x18)
  {
    mx = (int16_t)(((int16_t)magRead[1] << 8) | magRead[0]);
    my = (int16_t)(((int16_t)magRead[3] << 8) | magRead[2]);
    tmx = (double)mx * 0.15;
    tmy = (double)my * 0.15;
  }
  yaw = atan2(tmx, tmy) * 180 / 3.141593;

  if (offset_mag >= 0)
  {
    if (yaw - offset_mag_plus < -180)
      yaw = 180 + (yaw - offset_mag_minus);
    else
      yaw = yaw - offset_mag_plus;
  }
  else
  {
    if (yaw - offset_mag_minus > 180)
      yaw = -180 + (yaw - offset_mag_plus);
    else
      yaw = (yaw - offset_mag_minus);
  }
  return yaw;
}

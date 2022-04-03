#include "mpu9250sensor.h"

extern "C" {
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
}

#include <iostream>
#include <thread>

MPU9250Sensor::MPU9250Sensor(std::unique_ptr<I2cCommunicator> i2cBus) : i2cBus_(std::move(i2cBus))
{
  initImuI2c();

  // Wake up sensor
  int result = i2cBus_->write(PWR_MGMT_1, 0);

  // Disable I2C master interface
  result = i2cBus_->write(MPU9250_USER_CTRL, 0x00);
  // Enable bypass mode for magnetometer
  result = i2cBus_->write(MPU9250_BYPASS_ADDR, 0x02);

  // Set magnetometer to 100 Hz continuous measurement mode
  initMagnI2c();

  result = i2cBus_->write(MAG_MEAS_MODE, 0x00);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  result = i2cBus_->write(MAG_MEAS_MODE, FUSE_ROM_ACCESS_MODE);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  mag_asax = (i2cBus_->read(0x10) - 128) * 0.5 / 128 + 1;
  mag_asay = (i2cBus_->read(0x11) - 128) * 0.5 / 128 + 1;
  mag_asaz = (i2cBus_->read(0x12) - 128) * 0.5 / 128 + 1;

  result = i2cBus_->write(MAG_MEAS_MODE, 0x00);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  result = i2cBus_->write(MAG_MEAS_MODE, 0x16);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  initImuI2c();

  return;
}

void MPU9250Sensor::initImuI2c() const
{
  if (ioctl(i2cBus_->getFile(), I2C_SLAVE, MPU9250_ADDRESS_DEFAULT) < 0)
  {
    std::cerr << "Failed to select IMU device";
    exit(1);
  }

  return;
}

void MPU9250Sensor::initMagnI2c() const
{
  if (ioctl(i2cBus_->getFile(), I2C_SLAVE, AK8963_ADDRESS_DEFAULT) < 0)
  {
    std::cerr << "Failed to select magnetometer device";
    exit(1);
  }

  return;
}

double MPU9250Sensor::getAccelerationX() const
{
  int16_t accel_x_msb = i2cBus_->read(ACCEL_XOUT_H);
  int16_t accel_x_lsb = i2cBus_->read(ACCEL_XOUT_H + 1);
  int16_t accel_x     = accel_x_lsb | accel_x_msb << 8;
  double accel_x_conv = convertRawAccelerometerData(accel_x - accel_x_offset_);

  return accel_x_conv;
}

double MPU9250Sensor::getAccelerationY() const
{
  int16_t accel_y_msb = i2cBus_->read(ACCEL_YOUT_H);
  int16_t accel_y_lsb = i2cBus_->read(ACCEL_YOUT_H + 1);
  int16_t accel_y     = accel_y_lsb | accel_y_msb << 8;
  double accel_y_conv = convertRawAccelerometerData(accel_y - accel_y_offset_);

  return accel_y_conv;
}

double MPU9250Sensor::getAccelerationZ() const
{
  int16_t accel_z_msb = i2cBus_->read(ACCEL_ZOUT_H);
  int16_t accel_z_lsb = i2cBus_->read(ACCEL_ZOUT_H + 1);
  int16_t accel_z     = accel_z_lsb | accel_z_msb << 8;
  double accel_z_conv = convertRawAccelerometerData(accel_z - accel_z_offset_);

  return accel_z_conv;
}

double MPU9250Sensor::getAngularVelocityX() const
{
  int16_t gyro_x_msb = i2cBus_->read(GYRO_XOUT_H);
  int16_t gyro_x_lsb = i2cBus_->read(GYRO_XOUT_H + 1);
  int16_t gyro_x     = gyro_x_lsb | gyro_x_msb << 8;
  double gyro_x_conv = convertRawGyroscopeData(gyro_x - gyro_x_offset_);

  return gyro_x_conv;
}

double MPU9250Sensor::getAngularVelocityY() const
{
  int16_t gyro_y_msb = i2cBus_->read(GYRO_YOUT_H);
  int16_t gyro_y_lsb = i2cBus_->read(GYRO_YOUT_H + 1);
  int16_t gyro_y     = gyro_y_lsb | gyro_y_msb << 8;
  double gyro_y_conv = convertRawGyroscopeData(gyro_y - gyro_y_offset_);

  return gyro_y_conv;
}

double MPU9250Sensor::getAngularVelocityZ() const
{
  int16_t gyro_z_msb = i2cBus_->read(GYRO_ZOUT_H);
  int16_t gyro_z_lsb = i2cBus_->read(GYRO_ZOUT_H + 1);
  int16_t gyro_z     = gyro_z_lsb | gyro_z_msb << 8;
  double gyro_z_conv = convertRawGyroscopeData(gyro_z - gyro_z_offset_);

  return gyro_z_conv;
}

double MPU9250Sensor::getMagneticFluxDensityX() const
{
  initMagnI2c();
  int16_t mag_x_msb = i2cBus_->read(MAG_XOUT_L + 1);
  int16_t mag_x_lsb = i2cBus_->read(MAG_XOUT_L);
  int16_t mag_x     = mag_x_lsb | mag_x_msb << 8;
  double mag_x_conv = convertRawMagnetometerData(mag_x - mag_x_offset_);
  initImuI2c();

  return mag_x_conv * mag_asax * mag_x_scale_;
}

double MPU9250Sensor::getMagneticFluxDensityY() const
{
  initMagnI2c();
  int16_t mag_y_msb = i2cBus_->read(MAG_YOUT_L + 1);
  int16_t mag_y_lsb = i2cBus_->read(MAG_YOUT_L);
  int16_t mag_y     = mag_y_lsb | mag_y_msb << 8;
  double mag_y_conv = convertRawMagnetometerData(mag_y - mag_y_offset_);
  initImuI2c();

  return mag_y_conv * mag_asay * mag_y_scale_;
}

double MPU9250Sensor::getMagneticFluxDensityZ() const
{
  initMagnI2c();
  int16_t mag_z_msb = i2cBus_->read(MAG_ZOUT_L + 1);
  int16_t mag_z_lsb = i2cBus_->read(MAG_ZOUT_L);
  int16_t mag_z     = mag_z_lsb | mag_z_msb << 8;
  double mag_z_conv = convertRawMagnetometerData(mag_z - mag_z_offset_);
  initImuI2c();

  return mag_z_conv * mag_asaz * mag_z_scale_;
}

void MPU9250Sensor::triggerNextMagReading() const
{
  initMagnI2c();
  int8_t status2 = i2cBus_->read(STATUS_2);

  return;
}

double MPU9250Sensor::convertRawGyroscopeData(int16_t gyro_raw) const
{
  const double ang_vel_in_deg_per_s = static_cast<double>(gyro_raw) / GYRO_SCALE_FACTOR;
  return ang_vel_in_deg_per_s;
}

double MPU9250Sensor::convertRawMagnetometerData(int16_t mag_raw) const
{
  const double mag_in_mu_tesla = static_cast<double>(mag_raw) * MAX_CONV_MAG_FLUX / MAX_RAW_MAG_FLUX;
  return mag_in_mu_tesla;
}

double MPU9250Sensor::convertRawAccelerometerData(int16_t accel_raw) const
{
  const double accel_in_m_per_s = static_cast<double>(accel_raw) / ACCEL_SCALE_FACTOR * GRAVITY;
  return accel_in_m_per_s;
}

void MPU9250Sensor::setGyroscopeOffset(double gyro_x_offset, double gyro_y_offset, double gyro_z_offset)
{
  gyro_x_offset_ = gyro_x_offset;
  gyro_y_offset_ = gyro_y_offset;
  gyro_z_offset_ = gyro_z_offset;

  return;
}

void MPU9250Sensor::setAccelerometerOffset(double accel_x_offset, double accel_y_offset, double accel_z_offset)
{
  accel_x_offset_ = accel_x_offset;
  accel_y_offset_ = accel_y_offset;
  accel_z_offset_ = accel_z_offset;

  return;
}

void MPU9250Sensor::setMagnetometerOffset (double mag_x_offset, double mag_y_offset, double mag_z_offset)
{
  mag_x_offset_ = mag_x_offset;
  mag_y_offset_ = mag_y_offset;
  mag_z_offset_ = mag_z_offset;

  return;
}

void MPU9250Sensor::setMagnetometerScale  (double mag_x_scale, double mag_y_scale, double mag_z_scale)
{
  mag_x_scale_ = mag_x_scale;
  mag_y_scale_ = mag_y_scale;
  mag_z_scale_ = mag_z_scale;

  return;
}
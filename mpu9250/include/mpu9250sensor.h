#ifndef MPU9250SENSOR_H
#define MPU9250SENSOR_H

#include <memory>
#include <string>
#include <unordered_map>

#include "I2cCommunicator.h"

class MPU9250Sensor {
 public:
  MPU9250Sensor(std::unique_ptr<I2cCommunicator> i2cBus);

  double getAccelerationX() const;
  double getAccelerationY() const;
  double getAccelerationZ() const;
  double getAngularVelocityX() const;
  double getAngularVelocityY() const;
  double getAngularVelocityZ() const;
  double getMagneticFluxDensityX() const;
  double getMagneticFluxDensityY() const;
  double getMagneticFluxDensityZ() const;
  void   triggerNextMagReading  () const;
  void   setGyroscopeOffset    (int    gyro_x_offset , int    gyro_y_offset , int    gyro_z_offset );
  void   setAccelerometerOffset(int    accel_x_offset, int    accel_y_offset, int    accel_z_offset);
  void   setMagnetometerOffset (int    mag_x_offset  , int    mag_y_offset  , int    mag_z_offset  );
  void   setMagnetometerScale  (double mag_x_scale   , double mag_y_scale   , double mag_z_scale   );

 private:
  void   initImuI2c () const;
  void   initMagnI2c() const;
  double convertRawGyroscopeData    (int16_t gyro_raw ) const;
  double convertRawAccelerometerData(int16_t accel_raw) const;
  double convertRawMagnetometerData (int16_t mag_raw  ) const;

  std::unique_ptr<I2cCommunicator> i2cBus_;
  int    accel_range_{2};
  int    gyro_range_{250};
  int    dlpf_range_{260};
  int    gyro_x_offset_{0};
  int    gyro_y_offset_{0};
  int    gyro_z_offset_{0};
  int    accel_x_offset_{0};
  int    accel_y_offset_{0};
  int    accel_z_offset_{0};
  int    mag_x_offset_{0};
  int    mag_y_offset_{0};
  int    mag_z_offset_{0};
  double mag_x_scale_{0};
  double mag_y_scale_{0};
  double mag_z_scale_{0};
  double mag_asax;
  double mag_asay;
  double mag_asaz;

  // MPU9250 registers and addresses (s. datasheet for details)
  static constexpr int MPU9250_ADDRESS_DEFAULT = 0x68;
  static constexpr int AK8963_ADDRESS_DEFAULT  = 0x0C;
  static constexpr int MPU9250_USER_CTRL   = 0x6A;
  static constexpr int MPU9250_BYPASS_ADDR = 0x37;
  static constexpr int PWR_MGMT_1   = 0x6B;
  static constexpr int GYRO_CONFIG  = 0x1B;
  static constexpr int ACCEL_CONFIG = 0x1C;
  static constexpr int ACCEL_XOUT_H = 0x3B;
  static constexpr int ACCEL_YOUT_H = 0x3D;
  static constexpr int ACCEL_ZOUT_H = 0x3F;
  static constexpr int GYRO_XOUT_H  = 0x43;
  static constexpr int GYRO_YOUT_H  = 0x45;
  static constexpr int GYRO_ZOUT_H  = 0x47;
  static constexpr int MAG_ADDR     = 0x0C;
  static constexpr int MAG_XOUT_L   = 0x03;
  static constexpr int MAG_YOUT_L   = 0x05;
  static constexpr int MAG_ZOUT_L   = 0x07;
  static constexpr int STATUS_2     = 0x09;
  static constexpr int MAG_MEAS_MODE        = 0x0A;
  static constexpr int FUSE_ROM_ACCESS_MODE = 0x0F;
  // Helper constants
  static constexpr double MAX_RAW_MAG_FLUX   = 32760.0;
  static constexpr double MAX_CONV_MAG_FLUX  = 4912.0;
  static constexpr double ACCEL_SCALE_FACTOR = 16384.0;
  static constexpr double GYRO_SCALE_FACTOR  = 131.0;
};

#endif  // MPU9250SENSOR_H

#include "mpu9250driver/mpu9250driver.h"

#include <chrono>
#include <memory>

#include "LinuxI2cCommunicator.h"

using namespace std::chrono_literals;

MPU9250Driver::MPU9250Driver() : Node("mpu9250driver")
{
  RCLCPP_INFO(this->get_logger(), "Starting MPU9250Driver Node");

  // Create concrete I2C communicator and pass to sensor
  std::unique_ptr<I2cCommunicator> i2cBus = std::make_unique<LinuxI2cCommunicator>();
  mpu9250_ = std::make_unique<MPU9250Sensor>(std::move(i2cBus));
  
  // Declare parameters
  declareParameters();
  
  // Set parameters
  mpu9250_->setGyroscopeOffset    (this->get_parameter("gyroscope_x_offset"   ).as_int(),
                                   this->get_parameter("gyroscope_y_offset"   ).as_int(),
                                   this->get_parameter("gyroscope_z_offset"   ).as_int());
  mpu9250_->setAccelerometerOffset(this->get_parameter("acceleration_x_offset").as_int(),
                                   this->get_parameter("acceleration_y_offset").as_int(),
                                   this->get_parameter("acceleration_z_offset").as_int());
  mpu9250_->setMagnetometerOffset (this->get_parameter("magnetometer_x_offset").as_int(),
                                   this->get_parameter("magnetometer_y_offset").as_int(),
                                   this->get_parameter("magnetometer_z_offset").as_int());
  mpu9250_->setMagnetometerScale  (this->get_parameter("magnetometer_x_scale" ).as_double(),
                                   this->get_parameter("magnetometer_y_scale" ).as_double(),
                                   this->get_parameter("magnetometer_z_scale" ).as_double());

  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  std::chrono::duration<int64_t, std::milli> frequency = 1000ms / 250;
  timer_ = this->create_wall_timer(frequency, std::bind(&MPU9250Driver::handleInput, this));
}

void MPU9250Driver::handleInput()
{
  RCLCPP_DEBUG(this->get_logger(), "Creating IMU message");

  auto message            = sensor_msgs::msg::Imu();
  message.header.stamp    = this->get_clock()->now();
  message.header.frame_id = "imu_link";

  // Direct measurements
  message.linear_acceleration_covariance = {0};
  message.linear_acceleration.x = mpu9250_->getAccelerationX();
  message.linear_acceleration.y = mpu9250_->getAccelerationY();
  message.linear_acceleration.z = mpu9250_->getAccelerationZ();
  message.angular_velocity_covariance[0] = {0};
  message.angular_velocity.x = mpu9250_->getAngularVelocityX();
  message.angular_velocity.y = mpu9250_->getAngularVelocityY();
  message.angular_velocity.z = mpu9250_->getAngularVelocityZ();
  
  // Calculate euler angles, convert to quaternion and store in message
  message.orientation_covariance = {0};
  calculateOrientation(message);
  publisher_->publish (message);

  return;
}

void MPU9250Driver::declareParameters()
{
  this->declare_parameter<int>   ("gyroscope_x_offset"   , 0);
  this->declare_parameter<int>   ("gyroscope_y_offset"   , 0);
  this->declare_parameter<int>   ("gyroscope_z_offset"   , 0);
  this->declare_parameter<int>   ("acceleration_x_offset", 0);
  this->declare_parameter<int>   ("acceleration_y_offset", 0);
  this->declare_parameter<int>   ("acceleration_z_offset", 0);
  this->declare_parameter<int>   ("magnetometer_x_offset", 0);
  this->declare_parameter<int>   ("magnetometer_y_offset", 0);
  this->declare_parameter<int>   ("magnetometer_z_offset", 0);
  this->declare_parameter<double>("magnetometer_x_scale" , 0);
  this->declare_parameter<double>("magnetometer_y_scale" , 0);
  this->declare_parameter<double>("magnetometer_z_scale" , 0);

  return;
}

void MPU9250Driver::calculateOrientation(sensor_msgs::msg::Imu& imu_message)
{
  // Calculate Euler angles
  double roll, pitch, yaw, magneticFluxDensityX, magneticFluxDensityY,  magneticFluxDensityZ;

  magneticFluxDensityX = mpu9250_->getMagneticFluxDensityX();
  magneticFluxDensityY = mpu9250_->getMagneticFluxDensityY();
  magneticFluxDensityZ = mpu9250_->getMagneticFluxDensityZ();
  mpu9250_->triggerNextMagReading();

  roll  = atan2(imu_message.linear_acceleration.y, imu_message.linear_acceleration.z);

  pitch = atan2(-imu_message.linear_acceleration.x,
           (sqrt(imu_message.linear_acceleration.y * imu_message.linear_acceleration.y +
                 imu_message.linear_acceleration.z * imu_message.linear_acceleration.z)));

  yaw   = atan2(-magneticFluxDensityY, magneticFluxDensityX);

  RCLCPP_INFO(this->get_logger(), "Roll: %f / Pitch: %f / Yaw: %f", roll * 180.0 / 3.1416, pitch * 180.0 / 3.1416, yaw * 180.0 / 3.1416);

  // Convert to quaternion
  double cy = cos(yaw   * 0.5);
  double sy = sin(yaw   * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll  * 0.5);
  double sr = sin(roll  * 0.5);

  imu_message.orientation.x = cy * cp * sr - sy * sp * cr;
  imu_message.orientation.y = sy * cp * sr + cy * sp * cr;
  imu_message.orientation.z = sy * cp * cr - cy * sp * sr;
  imu_message.orientation.w = cy * cp * cr + sy * sp * sr;

  return;
}

int main(int argc, char* argv[])
{
  rclcpp::init    (argc, argv);
  rclcpp::spin    (std::make_shared<MPU9250Driver>());
  rclcpp::shutdown();
  return 0;
}


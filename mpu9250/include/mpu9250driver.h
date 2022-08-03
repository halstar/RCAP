#ifndef MPU9250DRIVER_H
#define MPU9250DRIVER_H

#include <list>

#include "mpu9250sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class MPU9250Driver : public rclcpp::Node {
 public:
  MPU9250Driver();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::unique_ptr<MPU9250Sensor> mpu9250_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::list<double> averageRoll_;
  std::list<double> averagePitch_;
  std::list<double> averageYaw_;
  void handleInput();
  void declareParameters();
  void computeOrientation(sensor_msgs::msg::Imu& imu_message);
  void computeAverages(double roll, double pitch, double yaw, double *averageRoll, double *averagePitch, double *averageYaw);
};

#endif  // MPU9250DRIVER_H

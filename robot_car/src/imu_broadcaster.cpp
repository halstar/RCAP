/**
**  RCAP - Robot car - IMU broadcaster
**/

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <list>
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class FramePublisher : public rclcpp::Node
{

public:

  FramePublisher():Node("imu_broadcaster")
  {
    RCLCPP_INFO(this->get_logger(), "Starting IMU Broadcaster Node");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        qos_profile.history,
        qos_profile.depth),
      qos_profile);

    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_  = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      qos,
      std::bind(&FramePublisher::handle_message, this, _1));
  }

private:

  std::list<double> orientationXList_;
  std::list<double> orientationYList_;
  std::list<double> orientationZList_;
  std::list<double> orientationWList_;

  double average(const std::list<double>& values)
  {
    double sum = 0;
    for (const double &value : values)
    {
      sum += value;
    }

    return sum / values.size();
  }

  void handle_message(const sensor_msgs::msg::Imu & msg)
  {
    orientationXList_.push_back(msg.orientation.x);
    orientationYList_.push_back(msg.orientation.y);
    orientationZList_.push_back(msg.orientation.z);
    orientationWList_.push_back(msg.orientation.w);

    if (orientationXList_.size() > 60)
    {
      orientationXList_.pop_front();
      orientationYList_.pop_front();
      orientationZList_.pop_front();
      orientationWList_.pop_front();
    }

    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped transformStamped;
    
    transformStamped.header.stamp    = msg.header.stamp;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id  = "base_link"; 

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = average(orientationXList_);
    transformStamped.transform.rotation.y = average(orientationYList_);
    transformStamped.transform.rotation.z = average(orientationZList_);
    transformStamped.transform.rotation.w = average(orientationWList_);

    RCLCPP_DEBUG(this->get_logger(), "%u - x: %f / y: %f / z: %f / w: %f", transformStamped.header.stamp.sec, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

    tfBroadcaster_->sendTransform(transformStamped);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>         tfBroadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init    (argc, argv);
  rclcpp::spin    (std::make_shared<FramePublisher>());
  rclcpp::shutdown();

  return 0;
}


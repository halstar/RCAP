/**
**  RCAP - Robot car - IMU broadcaster
**/

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
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

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_   = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      qos,
      std::bind(&FramePublisher::handle_message, this, _1));
  }

private:

  void handle_message(const sensor_msgs::msg::Imu & msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Handling IMU message");

    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // transformStamped.header.stamp    = now;
     transformStamped.header.stamp   = msg.header.stamp;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id  = "base_link"; 

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = msg.orientation.x;
    transformStamped.transform.rotation.y = msg.orientation.y;
    transformStamped.transform.rotation.z = msg.orientation.z;
    transformStamped.transform.rotation.w = msg.orientation.w;

    tf_broadcaster_->sendTransform(transformStamped);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>         tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init    (argc, argv);
  rclcpp::spin    (std::make_shared<FramePublisher>());
  rclcpp::shutdown();

  return 0;
}

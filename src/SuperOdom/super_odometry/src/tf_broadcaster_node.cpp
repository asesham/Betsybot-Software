#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomTfPublisher : public rclcpp::Node
{
public:
  OdomTfPublisher()
  : Node("odom_tf_publisher")
  {
    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to /lidar_odometry
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/laser_odometry", 10,
      std::bind(&OdomTfPublisher::odom_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Use same timestamp as odometry message
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    // Set translation
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    // Set orientation
    t.transform.rotation = msg->pose.pose.orientation;

    // Broadcast transform
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfPublisher>());
  rclcpp::shutdown();
  return 0;
}


#include <cstdio>
#include <chrono>
#include "color-detector/color-detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include <rclcpp_components/register_node_macro.hpp>

ObstacleAvoidance::ObstacleAvoidance(const rclcpp::NodeOptions & options) : Node("obstacle-avoidance", options)
{
    rgbd_subscriber_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>
     ("/camera/camera/rgbd", 10, std::bind(&ColorDetector::rgbd_callback, this, std::placeholders::_1));
     
    motor_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/motion_planning/cmd_vel", 10);
    trigger = std_msgs::msg::String();
    trigger.data = "NONE";
    color_publisher_ = this->create_publisher<std_msgs::msg::String>("/color_detector/Trigger", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&ColorDetector::timer_callback, this));
}v
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world obstacle-avoidance package\n");
  return 0;
}

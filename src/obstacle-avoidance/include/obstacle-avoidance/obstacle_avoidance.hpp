operator#pragma once
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "realsense2_camera_msgs/msg/rgbd.hpp"
#include "cv_bridge/cv_bridge.h"
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>


class ObstacleAvoidance : public rclcpp::Node
{
public:
	ObstacleAvoidance(const rclcpp::NodeOptions& options);
private:
	void rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg);
	void signal_callback(const std_msgs::msg::String::SharedPtr msg);
	void timer_callback();
		
	rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_subscriber_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_subscriber_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_speed_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	
	std::string signal_msg = "NONE";
	rclcpp::Time last_signal_time;
};
#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "obstacle-avoidance/obstacle_avoidance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include <rclcpp_components/register_node_macro.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/time.hpp>

#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace cv;


ObstacleAvoidance::ObstacleAvoidance(const rclcpp::NodeOptions & options) : Node("obstacle-avoidance", options)
{
    rgbd_subscriber_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>
     ("/camera/camera/rgbd", 10, std::bind(&ObstacleAvoidance::rgbd_callback, this, std::placeholders::_1));
    rgbd_subscriber_ = this->create_subscription<std_msgs::msg::String>
        ("/color_detector/Trigger", 10, std::bind(&ObstacleAvoidance::signal_callback, this, std::placeholders::_1));
    rgbd_subscriber_ = this->create_subscription<std_msgs::msg::String>
        ("/diff-drive/cmd_vel", 10, std::bind(&ObstacleAvoidance::diff_callback, this, std::placeholders::_1));
     
    motor_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/motion_planning/cmd_vel", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&ObstacleAvoidance::timer_callback, this));
}

void ObstacleAvoidance::rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg)
{
    cv::Mat rgb_image = cv_bridge::toCvShare(msg->rgb, "bgr8")->image;
    cv::Mat depth_image = cv_bridge::toCvShare(msg->depth, "32FC1")->image;

    sensor_msgs::msg::Image::SharedPtr rgb_image = std::make_shared<sensor_msgs::msg::Image>(msg->rgb);
    sensor_msgs::msg::Image::SharedPtr depth_image = std::make_shared<sensor_msgs::msg::Image>(msg->depth);

    Mat rgb = cv_bridge::toCvShare(rgb_image, "bgr8")->image;
    Mat depth = cv_bridge::toCvShare(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    // Process the RGB and depth images to detect obstacles
    // This is a placeholder for actual obstacle detection logic
    bool obstacle_detected = false; // Replace with actual detection result

	//Region of Interest (ROI) for obstacle detection
	// select a region in front of the robot and bottom half of the image

	Rect roi(200, 100, 400, 300); // Adjust these values based on your camera setup

	Mat depth_roi = depth(roi);
	double minVal, maxVal;
	Point minLoc, maxLoc;
	minMaxLoc(depth_roi, &minVal, &maxVal, &minLoc, &maxLoc);
	if (minVal < 0.5) { // If an obstacle is closer than 0.5 meters
		obstacle_detected = true;
	}
    
    if (obstacle_detected) {
        RCLCPP_INFO(this->get_logger(), "Obstacle detected!");
        // Implement avoidance maneuver
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0; // Stop forward movement
        cmd_vel.angular.z = 0.5; // Turn to avoid obstacle
        motor_speed_publisher_->publish(cmd_vel);
    }
}

void ObstacleAvoidance::signal_callback(const std_msgs::msg::String::SharedPtr msg)
{
	signal_msg = msg->data;
	last_signal_time = this->get_clock()->now();
	RCL_CPP_INFO(this->get_logger(), "Signal received: %s", signal_msg.c_str());
}

void ObstacleAvoidance::timer_callback()
{
    // Periodic tasks can be handled here
    RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node is running");
	time now = this->get_clock()->now();

    if (signal_msg != "NONE" && (now - last_signal_time > rclcpp::Duration::from_seconds(0.2)) {
        if (signal_msg == "GREEN" && !move) {
            move = true;
            RCLCPP_INFO(this->get_logger(), "Green signal received, resuming movement");
        }
        else if (signal_msg == "RED" && move) {
            move = false;
            RCLCPP_INFO(this->get_logger(), "Red signal received, stopping movement");
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0; // Stop movement
            cmd_vel.angular.z = 0.0;
            motor_speed_publisher_->publish(cmd_vel);
        }
	}

    if (move) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.2; // Move forward
        cmd_vel.angular.z = 0.0;
        motor_speed_publisher_->publish(cmd_vel);
	}
}



RCLCPP_COMPONENTS_REGISTER_NODE(ObstacleAvoidance)
/*
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world obstacle-avoidance package\n");
  return 0;
}
*/

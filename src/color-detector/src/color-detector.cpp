#include <chrono>
#include "color-detector/color-detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

ColorDetector::ColorDetector(const rclcpp::NodeOptions & options) : Node("color_detector", options)
{
    rgbd_subscriber_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>
     ("/camera/camera/rgbd", 10, std::bind(&ColorDetector::rgbd_callback, this, std::placeholders::_1));
        
    this->declare_parameter("green_min_depth", 0);
    this->declare_parameter("green_max_depth", 1400);
    this->declare_parameter("green_threshold", 1000000);
    this->declare_parameter("red_min_depth", 0);
    this->declare_parameter("red_max_depth", 2000);
    this->declare_parameter("red_threshold", 1000000);
    
    trigger = std_msgs::msg::String();
    trigger.data = "NONE";
    color_publisher_ = this->create_publisher<std_msgs::msg::String>("/color_detector/Trigger", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&ColorDetector::timer_callback, this));
}
    
void ColorDetector::detectCircles(Mat src)
{
	Mat gray;
	cvtColor(src, gray, COLOR_BGR2GRAY);
	
	GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
	imshow("Gray image", gray);
	std::vector<Vec3f> circles;
	HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
             	1,  // change this value to detect circles with different distances to each other
             	100, 30, 1, 100 // change the last two parameters
        	// (min_radius & max_radius) to detect larger circles
	);
	for( size_t i = 0; i < circles.size(); i++ )
	{
    	Vec3i c = circles[i];
   		Point center = Point(c[0], c[1]);
    	// circle center
    	circle( src, center, 1, Scalar(0,100,100), 3, LINE_AA);
    	// circle outline
    	int radius = c[2];
    	circle( src, center, radius, Scalar(255,0,255), 3, LINE_AA);
	}
	
	imshow("detected circles", src);
}

bool ColorDetector::detect_green(Mat hsv_image, Mat depth_image)
{
    // HSV lower bound for green
	Scalar lower_green = cv::Scalar(GREEN_LOW_H, GREEN_LOW_S, GREEN_LOW_V);
	// HSV upper bound for green
	Scalar upper_green = cv::Scalar(GREEN_HIGH_H, GREEN_HIGH_S, GREEN_HIGH_V);
	        	
    Mat depth_mask, green_mask;    
    inRange(hsv_image, lower_green, upper_green, green_mask);
    
    green_min_depth = this->get_parameter("green_min_depth").as_int();
    green_max_depth = this->get_parameter("green_max_depth").as_int();
    
    inRange(depth_image, green_min_depth, green_max_depth, depth_mask);
    
    green_mask = green_mask & depth_mask;
    
    imshow("green mask", green_mask);
    green_threshold = this->get_parameter("green_threshold").as_int();
    if (cv::sum(green_mask)[0] > green_threshold)
    	return true;
    return false;
}

bool ColorDetector::detect_red(Mat hsv_image, Mat depth_image)
{
    Mat lower_red_hue_range;
	Mat upper_red_hue_range;
	
	//Lower Red Hue
	inRange(hsv_image, cv::Scalar(LOW_RED_LOW_H, 85, 75), \
	                   cv::Scalar(LOW_RED_HIGH_H, 255, 255), lower_red_hue_range);
	
	// Upper red hue
	inRange(hsv_image, cv::Scalar(HIGH_RED_LOW_H, 85, 75), \
	                   cv::Scalar(HIGH_RED_HIGH_H, 255, 255), upper_red_hue_range);
	    	        	
    Mat depth_mask, red_mask;      
    
    // Combine masks
	addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_mask);
    red_min_depth = this->get_parameter("red_min_depth").as_int();
    red_max_depth = this->get_parameter("red_max_depth").as_int();
    
    inRange(depth_image, red_min_depth, red_max_depth, depth_mask);
    
    red_mask = red_mask & depth_mask;
    
    imshow("red mask", red_mask);
    red_threshold = this->get_parameter("red_threshold").as_int();
    if (cv::sum(red_mask)[0] > red_threshold)
    	return true;
    return false;
}

void ColorDetector::rgbd_callback (const realsense2_camera_msgs::msg::RGBD::SharedPtr rgbd_image)
{
    try
    {
        sensor_msgs::msg::Image::SharedPtr rgb_image = std::make_shared<sensor_msgs::msg::Image>(rgbd_image->rgb);
        sensor_msgs::msg::Image::SharedPtr depth_image = std::make_shared<sensor_msgs::msg::Image>(rgbd_image->depth);
        
        Mat rgb = cv_bridge::toCvShare(rgb_image, "bgr8")->image;
        Mat depth = cv_bridge::toCvShare(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        Mat hsv_image, red_mask1, red_mask2, red_mask, green_mask, depth_mask, gray_mask;
        
        Mat sub_rgb(rgb, Range(1, 360), Range(640, 1279));
        Mat sub_depth(depth, Range(1, 360), Range(640, 1279));
                  
        cvtColor(sub_rgb, hsv_image, COLOR_BGR2HSV);
        
        bool green = detect_green(hsv_image, sub_depth);
        bool red = detect_red(hsv_image, sub_depth);
        
        if (red)
        	trigger.data = "RED";
        else if (green)
        	trigger.data = "GREEN";
        else
        	trigger.data = "NONE";

        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e){
        RCLCPP_INFO(this->get_logger(), "I Could not convert from '%s' to 'bgr8'.", rgbd_image->depth.encoding.c_str());
    }
}

void ColorDetector::timer_callback()
{
    color_publisher_->publish(trigger);
} 

/*
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetector>());
    rclcpp::shutdown();
    return 0;
}
*/
RCLCPP_COMPONENTS_REGISTER_NODE(ColorDetector)



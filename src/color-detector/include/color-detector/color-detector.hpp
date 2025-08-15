
#ifndef COLOR_DETECTOR_HPP_
#define COLOR_DETECTOR_HPP_

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "realsense2_camera_msgs/msg/rgbd.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
using CameraInfoConstPtr = sensor_msgs::msg::CameraInfo::ConstSharedPtr;

#define GREEN_LOW_H     37
#define GREEN_LOW_S     38
#define GREEN_LOW_V     70
#define GREEN_HIGH_H    85
#define GREEN_HIGH_S    255
#define GREEN_HIGH_V    200

#define LOW_RED_LOW_H   0
#define LOW_RED_HIGH_H  10
#define HIGH_RED_LOW_H  160
#define HIGH_RED_HIGH_H 180

class ColorDetector : public rclcpp::Node
{
public:
    ColorDetector();
private:

    void detectCircles(Mat src);
    
    bool detect_green(Mat hsv_image, Mat depth_image);
    
    bool detect_red(Mat hsv_image, Mat depth_image);
    
    void rgbd_callback (const realsense2_camera_msgs::msg::RGBD::SharedPtr rgbd_image);
    
    void timer_callback();
    
    rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_publisher_;
    
    int green_min_depth;
    int green_max_depth;
    int red_min_depth;
    int red_max_depth;
    int green_threshold;
    int red_threshold;
    
    std_msgs::msg::String trigger;
};

#endif  // COLOR_DETECTOR_HPP_

/*
void topic_callback (const ImageConstPtr& image_rect,
                     const CameraInfoConstPtr& camera_info)
{
    try{
        Mat image = cv_bridge::toCvShare(image_rect, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        Mat hsv_image, red_mask1, red_mask2, red_mask, green_mask;

        cv::Mat depth_image(image.size(), CV_8UC1); // Assuming 8-bit depth
        cv::Mat bgr_image(image.size(), CV_8UC3); // 8-bit BGR
        

        for (int row = 0; row < image.rows; ++row) {
            for (int col = 0; col < image.cols; ++col) {
                unsigned short combined_value = image.at<unsigned short>(row, col);
                unsigned short depth_value = (combined_value >> 8);
                unsigned char bgr_value = (combined_value & 0xFF);

                depth_image.at<unsigned char>(row,col) = static_cast<unsigned char>(depth_value);

                bgr_image.at<cv::Vec3b>(row, col) = cv::Vec3b(
                    (bgr_value & 0x07),
                    (bgr_value & 0x38) >> 3,
                    (bgr_value & 0xC0) >> 6
                    );
            }
        }

        cv::imshow("Depth Image", depth_image);
        cv::imshow("BGR Image", bgr_image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception &e){
        RCLCPP_INFO(this->get_logger(), "I Could not convert from '%s' to 'bgr8'.", image_rect->encoding.c_str());
    }
}
*/

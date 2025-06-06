#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.hpp>
using namespace cv;

using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
using CameraInfoConstPtr = sensor_msgs::msg::CameraInfo::ConstSharedPtr;

int red_lower_range[3] = {180, 0, 0};
int red_upper_range[3] = {255, 50, 50};
int green_lower_range[3] = {170, 0, 0};
int green_upper_range[3] = {255, 100, 100};

int red_bottom_mean;
int red_sensitivity;
int red_top_mean;
int green_mean;
int green_sensitivity;

class ColorDetector : public rclcpp::Node
{
public:
    ColorDetector() : Node("color_detector")
    {
      it_ = std::shared_ptr<image_transport::ImageTransport>(
            new image_transport::ImageTransport(std::shared_ptr<rclcpp::Node>(this)));
  
      const image_transport::TransportHints hints(this, "raw", "");
      camera_image_subscriber_ =
          it_->subscribeCamera("/camera/camera/color/image_raw", 1,
                               &ColorDetector::topic_callback, this,
                               &hints); // TODO
    
        //image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>
        //("/camera/camera/color/image_raw", 10, std::bind(&ColorDetector::topic_callback, this, std::placeholders::_1));
        //cam_info_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>
        //("/camera/camera/color/camera_info", 10, std::bind(&ColorDetector::topic_callback, this, std::placeholders::_1));
        this->declare_parameter("red_bottom_mean", 5);
        this->declare_parameter("red_sensitivity", 5);
        this->declare_parameter("red_top_mean", 175);
        this->declare_parameter("green_mean", 60);
        this->declare_parameter("green_sensitivity", 10);
        //cv::namedWindow("view");
    }
private:

    void topic_callback (const ImageConstPtr& image_rect,
                         const CameraInfoConstPtr& camera_info)
    //void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        try{
            Mat image = cv_bridge::toCvShare(image_rect, "bgr8")->image;
            Mat hsv_image, red_mask1, red_mask2, red_mask, green_mask;

            cvtColor(image, hsv_image, COLOR_BGR2HSV);

            //Scalar red_lower_bound = Scalar(0, 70, 50);
            //Scalar red_upper_bound = Scalar(10, 255, 255);
            
            //Scalar green_lower_bound = Scalar(0, 70, 50);
            //Scalar green_upper_bound = Scalar(10, 255, 255);
            
            red_bottom_mean = this->get_parameter("red_bottom_mean").as_int();
            red_sensitivity = this->get_parameter("red_sensitivity").as_int();
            red_top_mean = this->get_parameter("red_top_mean").as_int();
            green_mean = this->get_parameter("green_mean").as_int();
            green_sensitivity = this->get_parameter("green_sensitivity").as_int();

            inRange(hsv_image, Scalar(red_bottom_mean - red_sensitivity, 70, 50), \
                               Scalar(red_bottom_mean + red_sensitivity, 255, 255), red_mask1);
            inRange(hsv_image, Scalar(red_top_mean - red_sensitivity, 70, 50), \
                               Scalar(red_top_mean + red_sensitivity, 255, 255), red_mask2);
            
            inRange(hsv_image, Scalar(green_mean - green_sensitivity, 100, 100), 
                               Scalar(green_mean + green_sensitivity, 255, 255), green_mask);
            
            red_mask = red_mask1 | red_mask2;

            imshow("Original Image", image);
            imshow("red Mask", red_mask);
            imshow("Green Mask", green_mask);
            
            //cv::imshow("view", cv_image);
            waitKey(1);
        } catch (cv_bridge::Exception &e){
            RCLCPP_INFO(this->get_logger(), "I Could not convert from '%s' to 'bgr8'.", image_rect->encoding.c_str());
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_subscriber_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber camera_image_subscriber_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetector>());
    rclcpp::shutdown();
    //cv::destroyWindow("view");
  return 0;
}

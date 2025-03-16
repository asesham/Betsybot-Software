/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/single_image_detector.h"

#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/msg/header.h>

namespace apriltag_ros
{

SingleImageDetector::SingleImageDetector() : Node("apriltag_ros_single_image_server")
{
  single_image_analysis_service_ = this->create_service<apriltag_msgs::srv::AnalyzeSingleImage>(
            "single_image_tag_detection",
            [&](const apriltag_msgs::srv::AnalyzeSingleImage::Request::SharedPtr req,
    apriltag_msgs::srv::AnalyzeSingleImage::Response::SharedPtr res)
                        {analyzeImage(req, res);});

  tag_detections_publisher_ =
      this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("tag_detections", 1);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Ready to do tag detection on single images");
}

bool SingleImageDetector::analyzeImage(
    const apriltag_msgs::srv::AnalyzeSingleImage::Request::SharedPtr req,
    apriltag_msgs::srv::AnalyzeSingleImage::Response::SharedPtr res)
{

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ Summoned to analyze image ]");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Image load path: %s",
           req->full_path_where_to_get_image.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Image save path: %s",
           req->full_path_where_to_save_image.c_str());

  // Read the image
  cv::Mat image = cv::imread(req->full_path_where_to_get_image,
                             cv::IMREAD_COLOR);
  if (image.data == NULL)
  {
    // Cannot read image
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Could not read image " <<
                     req->full_path_where_to_get_image.c_str());
    return false;
  }

  // Detect tags in the image
  cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::msg::Header(),
                                                            "bgr8", image));
  loaded_image->header.frame_id = "camera";
  res->tag_detections =
      tag_detector_.detectTags(loaded_image,sensor_msgs::msg::CameraInfo::ConstSharedPtr(
          new sensor_msgs::msg::CameraInfo(req->camera_info)));

  // Publish detected tags (AprilTagDetectionArray, basically an array of
  // geometry_msgs/PoseWithCovarianceStamped)
  tag_detections_publisher_->publish(res->tag_detections);

  // Save tag detections image
  tag_detector_.drawDetections(loaded_image);
  cv::imwrite(req->full_path_where_to_save_image, loaded_image->image);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done!\n");

  return true;
}

} // namespace apriltag_ros

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

#include "apriltag_ros/continuous_detector.h"

namespace apriltag_ros
{
ContinuousDetector::ContinuousDetector(const rclcpp::NodeOptions & options)
: Node("ContinuousDetector", options)
{
  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector());
  
  draw_tag_detections_image_ = getAprilTagOption<bool>(tag_detector_.get(), "publish_tag_detections_image", false);
  
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(std::shared_ptr<rclcpp::Node>(this))); // TODO: Need to look at the definition of image transport

  std::string transport_hint;
  tag_detector_->get_parameter_or<std::string>("transport_hint", transport_hint, "raw");
  
  const image_transport::TransportHints hints(this, transport_hint, "");
  uint32_t queue_size;
  tag_detector_->get_parameter_or<uint32_t>("queue_size", queue_size, 1);
  const std::string img_str = "image_rect";
  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", queue_size,
                          &ContinuousDetector::imageCallback, this,
                          &hints); // TODO
                          
  tag_detections_publisher_ =
      tag_detector_->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("tag_detections", 1);

  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1); // TODO
  }

  refresh_params_service_ = tag_detector_->create_service<std_srvs::srv::Empty>(
            "refresh_tag_params",
            [&](const std_srvs::srv::Empty::Request::SharedPtr req,
                std_srvs::srv::Empty::Response::SharedPtr res)
                        {refreshParamsCallback(req, res);});
}

void ContinuousDetector::refreshTagParameters()
{
  // Resetting the tag detector will cause a new param server lookup
  // So if the parameters have changed (by someone/something), 
  // they will be updated dynamically
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  tag_detector_.reset(new TagDetector());
}

bool ContinuousDetector::refreshParamsCallback(const std_srvs::srv::Empty::Request::SharedPtr,
                                               std_srvs::srv::Empty::Response::SharedPtr)
{
  refreshTagParameters();
  return true;
}

void ContinuousDetector::imageCallback (
    const ImageConstPtr& image_rect,
    const CameraInfoConstPtr& camera_info)
{
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_->get_subscription_count() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_->publish(
      tag_detector_->detectTags(cv_image_,camera_info));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::ContinuousDetector)

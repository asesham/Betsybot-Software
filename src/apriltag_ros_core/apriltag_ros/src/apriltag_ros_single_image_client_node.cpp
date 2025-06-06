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

#include "apriltag_ros/common_functions.h"
#include "apriltag_msgs/srv/analyze_single_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

bool getRosParameter (rclcpp::Node* node_ptr, const std::string name, double& param)
{
  // Write parameter "name" from ROS Parameter Server into param
  // Return true if successful, false otherwise
  rclcpp::Parameter temp_param;
  if (node_ptr->get_parameter(name, temp_param))
  {
    param = temp_param.as_double();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Set camera " << name.c_str() << " = " << param);
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Could not find " << name.c_str() << " parameter!");
    return false;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("apriltag_ros_single_image_client");
  
  rclcpp::Client<apriltag_msgs::srv::AnalyzeSingleImage>::SharedPtr client =
    node->create_client<apriltag_msgs::srv::AnalyzeSingleImage>("single_image_tag_detection");

  auto request = std::make_shared<apriltag_msgs::srv::AnalyzeSingleImage::Request>();
  request->full_path_where_to_get_image =
      apriltag_ros::getAprilTagOption<std::string>(
          node.get(), "image_load_path", "/home/arun/apriltag-imgs-master/tag36h11/tag36_11_00001.png");
  if (request->full_path_where_to_get_image.empty())
  {
    return 1;
  }
  request->full_path_where_to_save_image =
      apriltag_ros::getAprilTagOption<std::string>(
          node.get(), "image_save_path", "~/apriltag-imgs-master/tag36h11-dups/");
  if (request->full_path_where_to_save_image.empty())
  {
    return 1;
  }

  // Replicate sensors_msgs/CameraInfo message (must be up-to-date with the
  // analyzed image!)  
  request->camera_info.distortion_model = "plumb_bob";
  double fx, fy, cx, cy;
  /*if (!getRosParameter(node.get(), "fx", fx))
    return 1;
  if (!getRosParameter(node.get(), "fy", fy))
    return 1;
  if (!getRosParameter(node.get(), "cx", cx))
    return 1;
  if (!getRosParameter(node.get(), "cy", cy))
    return 1;
    */
  fx = 652.7934615847107;
  fy = 653.9480389077635;
  cx = 307.1288710375904;
  cy = 258.7823279214385;
  // Intrinsic camera matrix for the raw (distorted) images
  request->camera_info.k[0] = fx;
  request->camera_info.k[2] = cx;
  request->camera_info.k[4] = fy;
  request->camera_info.k[5] = cy;
  request->camera_info.k[8] = 1.0;
  request->camera_info.p[0] = fx;
  request->camera_info.p[2] = cx;
  request->camera_info.p[5] = fy;
  request->camera_info.p[6] = cy;
  request->camera_info.p[10] = 1.0;


  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  
  // Call the service (detect tags in the image specified by the
  // image_load_path)
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {    
    // use parameter run_quielty=false in order to have the service
    // print out the tag position and orientation
    if (result.get()->tag_detections.detections.size() == 0)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),"No detected tags!");
    }
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service single_image_tag_detection");
    return 1;
  }
  
  return 0; // happy ending
}

// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_ros/transform_broadcaster.h>

// struct PoseSE3 {
//   tf2::Vector3 t{0,0,0};
//   tf2::Quaternion q{0,0,0,1};
//   bool valid{false};
//   rclcpp::Time stamp;
// };

// class MapOdomBroadcaster : public rclcpp::Node {
// public:
//   MapOdomBroadcaster() : Node("map_odom_broadcaster")
//   {
//     // Parameters
//     global_odom_topic_ = declare_parameter<std::string>("global_odom_topic", "/state_estimation");    // base in map
//     local_odom_topic_  = declare_parameter<std::string>("local_odom_topic",  "/laser_odometry");      // base in odom
//     //
//     map_frame_         = declare_parameter<std::string>("map_frame",  "map");
//     odom_frame_        = declare_parameter<std::string>("odom_frame", "odom");
//     base_frame_hint_   = declare_parameter<std::string>("base_frame_hint", "base_footprint");
//     //
//     publish_rate_hz_   = declare_parameter<double>("publish_rate_hz", 50.0);
//     use_msg_frame_ids_ = declare_parameter<bool>("use_msg_frame_ids", true);
//     identity_if_missing_ = declare_parameter<bool>("identity_if_missing", false);

//     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

//     // Subscriptions
//     sub_global_ = create_subscription<nav_msgs::msg::Odometry>(
//       global_odom_topic_, rclcpp::QoS(10),
//       std::bind(&MapOdomBroadcaster::onGlobalOdom, this, std::placeholders::_1));

//     sub_local_ = create_subscription<nav_msgs::msg::Odometry>(
//       local_odom_topic_, rclcpp::QoS(100),
//       std::bind(&MapOdomBroadcaster::onLocalOdom, this, std::placeholders::_1));

//     // Timer to publish TF at a steady rate (avoids jitter if msgs are bursty)
//     const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
//     timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
//                                std::bind(&MapOdomBroadcaster::publishTf, this));

//     RCLCPP_INFO(get_logger(), "map_odom_broadcaster: global='%s', local='%s', frames: map='%s', odom='%s'",
//                 global_odom_topic_.c_str(), local_odom_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str());
//   }

// private:
//   static tf2::Transform toTf(const PoseSE3& p) {
//     tf2::Transform T;
//     T.setOrigin(p.t);
//     T.setRotation(p.q);
//     return T;
//   }

//   void onGlobalOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     if (use_msg_frame_ids_) {
//       map_frame_  = msg->header.frame_id.empty() ? map_frame_  : msg->header.frame_id;
//       base_frame_global_ = msg->child_frame_id.empty() ? base_frame_hint_ : msg->child_frame_id;
//     }
//     global_pose_.t = tf2::Vector3(msg->pose.pose.position.x,
//                                   msg->pose.pose.position.y,
//                                   msg->pose.pose.position.z);
//     global_pose_.q = tf2::Quaternion(msg->pose.pose.orientation.x,
//                                      msg->pose.pose.orientation.y,
//                                      msg->pose.pose.orientation.z,
//                                      msg->pose.pose.orientation.w);
//     global_pose_.valid = true;
//     global_pose_.stamp = msg->header.stamp;
//   }

//   void onLocalOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     if (use_msg_frame_ids_) {
//       odom_frame_ = msg->header.frame_id.empty() ? odom_frame_ : msg->header.frame_id;
//       base_frame_local_ = msg->child_frame_id.empty() ? base_frame_hint_ : msg->child_frame_id;
//     }
//     local_pose_.t = tf2::Vector3(msg->pose.pose.position.x,
//                                  msg->pose.pose.position.y,
//                                  msg->pose.pose.position.z);
//     local_pose_.q = tf2::Quaternion(msg->pose.pose.orientation.x,
//                                     msg->pose.pose.orientation.y,
//                                     msg->pose.pose.orientation.z,
//                                     msg->pose.pose.orientation.w);
//     local_pose_.valid = true;
//     local_pose_.stamp = msg->header.stamp;
//   }

//   void publishTf() {
//     if (!global_pose_.valid || !local_pose_.valid) {
//       if (identity_if_missing_ && (global_pose_.valid || local_pose_.valid)) {
//         // Publish identity if one stream is missing, useful for bring-up.
//         geometry_msgs::msg::TransformStamped tf_msg;
//         tf_msg.header.stamp = now();
//         tf_msg.header.frame_id = map_frame_;
//         tf_msg.child_frame_id  = odom_frame_;
//         tf_msg.transform.translation.x = 0.0;
//         tf_msg.transform.translation.y = 0.0;
//         tf_msg.transform.translation.z = 0.0;
//         tf_msg.transform.rotation.w = 1.0;
//         tf_msg.transform.rotation.x = 0.0;
//         tf_msg.transform.rotation.y = 0.0;
//         tf_msg.transform.rotation.z = 0.0;
//         tf_broadcaster_->sendTransform(tf_msg);
//       }
//       return;
//     }

//     // Compute T_map_odom = T_map_base * inv(T_odom_base)
//     tf2::Transform T_map_base = toTf(global_pose_);
//     tf2::Transform T_odom_base = toTf(local_pose_);
//     tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

//     geometry_msgs::msg::TransformStamped tf_msg;
//     tf_msg.header.stamp = now();
//     tf_msg.header.frame_id = map_frame_;
//     tf_msg.child_frame_id  = odom_frame_;
//     tf_msg.transform = tf2::toMsg(T_map_odom);

//     tf_broadcaster_->sendTransform(tf_msg);
//   }

//   // Params
//   std::string global_odom_topic_, local_odom_topic_;
//   std::string map_frame_, odom_frame_, base_frame_hint_;
//   std::string base_frame_global_{"base_link"}, base_frame_local_{"base_link"};
//   double publish_rate_hz_;
//   bool use_msg_frame_ids_;
//   bool identity_if_missing_;

//   // State
//   PoseSE3 global_pose_; // base in map
//   PoseSE3 local_pose_;  // base in odom

//   // ROS
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_global_, sub_local_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MapOdomBroadcaster>());
//   rclcpp::shutdown();
//   return 0;
// }


/*

      --------------------------------------------------

*/

// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_ros/transform_broadcaster.h>

// #include <algorithm>
// #include <chrono>
// #include <memory>
// #include <string>

// struct PoseSE3 {
//   tf2::Vector3 t{0,0,0};
//   tf2::Quaternion q{0,0,0,1};
//   bool valid{false};
//   rclcpp::Time stamp;
// };

// class MapOdomBroadcaster : public rclcpp::Node {
// public:
//   MapOdomBroadcaster() : Node("map_odom_broadcaster")
//   {
//     // Parameters
//     global_odom_topic_   = declare_parameter<std::string>("global_odom_topic", "/state_estimation"); // base in map
//     local_odom_topic_    = declare_parameter<std::string>("local_odom_topic",  "/laser_odometry");   // base in odom
//     //
//     map_frame_           = declare_parameter<std::string>("map_frame",  "map");
//     odom_frame_          = declare_parameter<std::string>("odom_frame", "odom");
//     base_frame_hint_     = declare_parameter<std::string>("base_frame_hint", "base_link");
//     base_frame_out_      = declare_parameter<std::string>("base_frame_out", "base_footprint"); // TF child
//     //
//     publish_rate_hz_     = declare_parameter<double>("publish_rate_hz", 50.0);
//     use_msg_frame_ids_   = declare_parameter<bool>("use_msg_frame_ids", true);
//     identity_if_missing_ = declare_parameter<bool>("identity_if_missing", false);
//     project_to_2d_       = declare_parameter<bool>("project_to_2d", true); // zero roll/pitch
//     zero_z_              = declare_parameter<bool>("zero_z", true);        // put base_footprint on z=0
//     base_z_offset_       = declare_parameter<double>("base_z_offset", 0.0);

//     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

//     // Subscriptions
//     sub_global_ = create_subscription<nav_msgs::msg::Odometry>(
//       global_odom_topic_, rclcpp::QoS(10),
//       std::bind(&MapOdomBroadcaster::onGlobalOdom, this, std::placeholders::_1));

//     sub_local_ = create_subscription<nav_msgs::msg::Odometry>(
//       local_odom_topic_, rclcpp::QoS(100),
//       std::bind(&MapOdomBroadcaster::onLocalOdom, this, std::placeholders::_1));

    

//     // Timer for steady TF publish
//     const double hz = std::max(1.0, publish_rate_hz_);
//     const auto period = std::chrono::duration<double>(1.0 / hz);
//     timer_ = create_wall_timer(
//       std::chrono::duration_cast<std::chrono::nanoseconds>(period),
//       std::bind(&MapOdomBroadcaster::publishTf, this));

//     RCLCPP_INFO(get_logger(),
//       "map_odom_broadcaster: global='%s', local='%s', map_frame='%s', odom_frame='%s', use_msg_frame_ids=%s",
//       global_odom_topic_.c_str(), local_odom_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str(),
//       use_msg_frame_ids_ ? "true" : "false");
//   }

// private:

//   static tf2::Transform toTf(const PoseSE3& p) 
//   {
//     tf2::Transform T;
//     T.setOrigin(p.t);
//     tf2::Quaternion q = p.q;
//     if (q.length2() > 0.0) q.normalize();
//     T.setRotation(q);
//     return T;
//   }

//   void maybeWarnBaseFrameMismatch() 
//   {
//     if (global_pose_.valid && local_pose_.valid && base_frame_global_ != base_frame_local_) {
//       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
//         "Global child_frame_id ('%s') != Local child_frame_id ('%s'). "
//         "Assuming both represent the same physical base.",
//         base_frame_global_.c_str(), base_frame_local_.c_str());
//     }
//   }

//   void onGlobalOdom(const nav_msgs::msg::Odometry::SharedPtr msg) 
//   {
//     if (use_msg_frame_ids_) 
//     {
//       const auto& f = msg->header.frame_id;
//       if (!f.empty()) 
//       {
//         if (f == odom_frame_) 
//         {
//           RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
//             "Ignoring global frame_id '%s' because it equals current odom_frame '%s'.",
//             f.c_str(), odom_frame_.c_str());
//         } 
//         else 
//         {
//           map_frame_ = f;
//         }
//       }
      
//       if (!msg->child_frame_id.empty()) 
//         base_frame_global_ = msg->child_frame_id;
//     }

//     global_pose_.t = tf2::Vector3(msg->pose.pose.position.x,
//                                   msg->pose.pose.position.y,
//                                   msg->pose.pose.position.z);
//     global_pose_.q = tf2::Quaternion(msg->pose.pose.orientation.x,
//                                      msg->pose.pose.orientation.y,
//                                      msg->pose.pose.orientation.z,
//                                      msg->pose.pose.orientation.w);
//     global_pose_.valid = true;
//     global_pose_.stamp = msg->header.stamp;

//     maybeWarnBaseFrameMismatch();
//   }

//   void onLocalOdom(const nav_msgs::msg::Odometry::SharedPtr msg) 
//   {
//     if (use_msg_frame_ids_) 
//     {
//       const auto& f = msg->header.frame_id;
//       if (!f.empty()) 
//       {
//         if (f == map_frame_) 
//         {
//           RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
//             "Ignoring local frame_id '%s' because it equals current map_frame '%s'.",
//             f.c_str(), map_frame_.c_str());
//         } 
//         else 
//         {
//           odom_frame_ = f;
//         }
//       }
//       if (!msg->child_frame_id.empty()) 
//         base_frame_local_ = msg->child_frame_id;
//     }

//     local_pose_.t = tf2::Vector3(msg->pose.pose.position.x,
//                                  msg->pose.pose.position.y,
//                                  msg->pose.pose.position.z);
//     local_pose_.q = tf2::Quaternion(msg->pose.pose.orientation.x,
//                                     msg->pose.pose.orientation.y,
//                                     msg->pose.pose.orientation.z,
//                                     msg->pose.pose.orientation.w);
//     local_pose_.valid = true;
//     local_pose_.stamp = msg->header.stamp;

//     maybeWarnBaseFrameMismatch();
//   }

//   rclcpp::Time commonStamp() const {
//     // Use the older of the two stamps to avoid future-dated TFs wrt either source
//     if (!global_pose_.valid && !local_pose_.valid) return now();
//     if (!global_pose_.valid) return local_pose_.stamp;
//     if (!local_pose_.valid)  return global_pose_.stamp;
//     return (global_pose_.stamp <= local_pose_.stamp) ? global_pose_.stamp : local_pose_.stamp;
//   }

//   void publishIdentityIfAllowed(const rclcpp::Time& stamp) {
//     if (map_frame_ == odom_frame_) {
//       RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
//         "Refusing to publish identity TF because map_frame ('%s') == odom_frame ('%s').",
//         map_frame_.c_str(), odom_frame_.c_str());
//       return;
//     }

//     geometry_msgs::msg::TransformStamped tf_msg;
//     tf_msg.header.stamp = stamp;
//     tf_msg.header.frame_id = map_frame_;
//     tf_msg.child_frame_id  = odom_frame_;
//     tf_msg.transform.translation.x = 0.0;
//     tf_msg.transform.translation.y = 0.0;
//     tf_msg.transform.translation.z = 0.0;
//     tf_msg.transform.rotation.w = 1.0;
//     tf_msg.transform.rotation.x = 0.0;
//     tf_msg.transform.rotation.y = 0.0;
//     tf_msg.transform.rotation.z = 0.0;
//     tf_broadcaster_->sendTransform(tf_msg);
//   }

//   void publishTf() {
//     // Never publish a self-TF
//     if (map_frame_ == odom_frame_) {
//       RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
//         "Refusing to publish TF: map_frame ('%s') equals odom_frame ('%s').",
//         map_frame_.c_str(), odom_frame_.c_str());
//       return;
//     }

//     // If one stream is missing and identity requested, publish identity
//     if (!global_pose_.valid || !local_pose_.valid) {
//       if (identity_if_missing_ && (global_pose_.valid || local_pose_.valid)) {
//         publishIdentityIfAllowed(commonStamp());
//       }
//       return;
//     }

//     // Compute T_map_odom = T_map_base * inv(T_odom_base)
//     tf2::Transform T_map_base  = toTf(global_pose_);
//     tf2::Transform T_odom_base = toTf(local_pose_);
//     tf2::Transform T_map_odom  = T_map_base * T_odom_base.inverse();

//     geometry_msgs::msg::TransformStamped tf_msg;
//     tf_msg.header.stamp = commonStamp();
//     tf_msg.header.frame_id = map_frame_;
//     tf_msg.child_frame_id  = odom_frame_;
//     tf_msg.transform = tf2::toMsg(T_map_odom);

//     tf_broadcaster_->sendTransform(tf_msg);
//   }

//   // Params
//   std::string global_odom_topic_, local_odom_topic_;
//   std::string map_frame_, odom_frame_, base_frame_hint_, base_frame_out_ ;
//   std::string base_frame_global_{"base_link"}, base_frame_local_{"base_link"};
//   double publish_rate_hz_{50.0};
//   bool use_msg_frame_ids_{true};
//   bool identity_if_missing_{false};

//   // State
//   PoseSE3 global_pose_; // base in map
//   PoseSE3 local_pose_;  // base in odom
//   PoseSE3 localfootprint_pose; // base footprint in odom

//   // ROS
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_global_, sub_local_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MapOdomBroadcaster>());
//   rclcpp::shutdown();
//   return 0;
// }

/*
    This code publish Map->Odom and Odom->Base Foot print

*/

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

struct PoseSE3 {
  tf2::Vector3 t{0,0,0};
  tf2::Quaternion q{0,0,0,1};
  rclcpp::Time stamp;
  bool valid{false};
};

class MapOdomBroadcaster : public rclcpp::Node {
public:
  MapOdomBroadcaster() : rclcpp::Node("map_odom_broadcaster")
  {
    // Parameters
    global_odom_topic_   = declare_parameter<std::string>("global_odom_topic", "/state_estimation"); // base in map
    local_odom_topic_    = declare_parameter<std::string>("local_odom_topic",  "/laser_odometry");   // base in odom
    map_frame_           = declare_parameter<std::string>("map_frame",  "map");
    odom_frame_          = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_hint_     = declare_parameter<std::string>("base_frame_hint", "base_link");
    base_frame_out_      = declare_parameter<std::string>("base_frame_out", "base_footprint"); // TF child for odom→base
    publish_rate_hz_     = declare_parameter<double>("publish_rate_hz", 50.0);
    use_msg_frame_ids_   = declare_parameter<bool>("use_msg_frame_ids", true);
    identity_if_missing_ = declare_parameter<bool>("identity_if_missing", false);
    // odom → base_footprint shaping
    project_to_2d_       = declare_parameter<bool>("project_to_2d", true); // zero roll/pitch
    zero_z_              = declare_parameter<bool>("zero_z", true);        // z=0 (plus offset)
    base_z_offset_       = declare_parameter<double>("base_z_offset", 0.0);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscriptions
    sub_global_ = create_subscription<nav_msgs::msg::Odometry>(
      global_odom_topic_, rclcpp::QoS(10),
      std::bind(&MapOdomBroadcaster::onGlobalOdom, this, std::placeholders::_1));

    sub_local_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_, rclcpp::QoS(100),
      std::bind(&MapOdomBroadcaster::onLocalOdom, this, std::placeholders::_1));

    // Timer
    const double hz = std::max(1.0, publish_rate_hz_);
    const auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MapOdomBroadcaster::publishBoth, this));

    RCLCPP_INFO(get_logger(),
      "global='%s', local='%s', map='%s', odom='%s', base_out='%s', use_msg_frame_ids=%s",
      global_odom_topic_.c_str(), local_odom_topic_.c_str(),
      map_frame_.c_str(), odom_frame_.c_str(), base_frame_out_.c_str(),
      use_msg_frame_ids_ ? "true" : "false");
  }

private:
  // --- helpers ---
  static tf2::Transform toTf(const PoseSE3& p) {
    tf2::Transform T;
    T.setOrigin(p.t);
    tf2::Quaternion q = p.q;
    if (q.length2() > 0.0) q.normalize();
    T.setRotation(q);
    return T;
  }

  rclcpp::Time commonStamp() const {
    if (!global_pose_.valid && !local_pose_.valid) return now();
    if (!global_pose_.valid) return local_pose_.stamp;
    if (!local_pose_.valid)  return global_pose_.stamp;
    return (global_pose_.stamp <= local_pose_.stamp) ? global_pose_.stamp : local_pose_.stamp;
  }

  void maybeWarnBaseFrameMismatch() {
    if (global_pose_.valid && local_pose_.valid && base_frame_global_ != base_frame_local_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Global child_frame_id ('%s') != Local child_frame_id ('%s'); assuming same physical base.",
        base_frame_global_.c_str(), base_frame_local_.c_str());
    }
  }

  // --- callbacks ---
  void onGlobalOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (use_msg_frame_ids_) {
      const auto& f = msg->header.frame_id;
      if (!f.empty()) {
        if (f == odom_frame_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Ignoring global frame_id '%s' because it equals current odom_frame '%s'.",
            f.c_str(), odom_frame_.c_str());
        } else {
          map_frame_ = f;
        }
      }
      if (!msg->child_frame_id.empty())
        base_frame_global_ = msg->child_frame_id;
    }

    global_pose_.t = {msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z};
    global_pose_.q = {msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w};
    global_pose_.stamp = msg->header.stamp;
    global_pose_.valid = true;

    maybeWarnBaseFrameMismatch();
  }

  void onLocalOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (use_msg_frame_ids_) {
      const auto& f = msg->header.frame_id;
      if (!f.empty()) {
        if (f == map_frame_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Ignoring local frame_id '%s' because it equals current map_frame '%s'.",
            f.c_str(), map_frame_.c_str());
        } else {
          odom_frame_ = f;
        }
      }
      if (!msg->child_frame_id.empty())
        base_frame_local_ = msg->child_frame_id;
    }

    local_pose_.t = {msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z};
    local_pose_.q = {msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w};
    if (local_pose_.q.length2() > 0.0) local_pose_.q.normalize();
    local_pose_.stamp = msg->header.stamp;
    local_pose_.valid = true;

    maybeWarnBaseFrameMismatch();
  }

  // --- publishers ---
  void publishMapToOdom() {
    // self-TF guard
    if (map_frame_ == odom_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Refusing to publish map→odom: map_frame('%s') == odom_frame('%s').",
        map_frame_.c_str(), odom_frame_.c_str());
      return;
    }

    if (!global_pose_.valid || !local_pose_.valid) {
      if (identity_if_missing_ && (global_pose_.valid || local_pose_.valid)) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = commonStamp();
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id  = odom_frame_;
        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_broadcaster_->sendTransform(tf_msg);
      }
      return;
    }

    // T_map_odom = T_map_base * inv(T_odom_base)
    const tf2::Transform T_map_base  = toTf(global_pose_);
    const tf2::Transform T_odom_base = toTf(local_pose_);
    const tf2::Transform T_map_odom  = T_map_base * T_odom_base.inverse();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = commonStamp();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id  = odom_frame_;
    tf_msg.transform = tf2::toMsg(T_map_odom);
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishOdomToBase() {
    if (!local_pose_.valid) return;

    // self-TF guard
    if (odom_frame_ == base_frame_out_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Refusing to publish odom→base: odom_frame('%s') == base_frame_out('%s').",
        odom_frame_.c_str(), base_frame_out_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = local_pose_.stamp;      // sync with odom source
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id  = base_frame_out_;

    // position
    tf_msg.transform.translation.x = local_pose_.t.x();
    tf_msg.transform.translation.y = local_pose_.t.y();
    tf_msg.transform.translation.z = zero_z_ ? base_z_offset_ : (local_pose_.t.z() + base_z_offset_);

    // orientation
    tf2::Quaternion q_out;
    if (project_to_2d_) {
      const double yaw = tf2::getYaw(local_pose_.q);
      q_out.setRPY(0.0, 0.0, yaw);
    } else {
      q_out = local_pose_.q;
    }
    q_out.normalize();
    tf_msg.transform.rotation = tf2::toMsg(q_out);

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishBoth() {
    publishMapToOdom();
    publishOdomToBase();
  }

  // --- params/state ---
  std::string global_odom_topic_, local_odom_topic_;
  std::string map_frame_, odom_frame_, base_frame_hint_, base_frame_out_;
  std::string base_frame_global_{"base_link"}, base_frame_local_{"base_link"};
  double publish_rate_hz_{50.0};
  bool use_msg_frame_ids_{true};
  bool identity_if_missing_{false};
  bool project_to_2d_{true};
  bool zero_z_{true};
  double base_z_offset_{0.0};

  PoseSE3 global_pose_; // base in map
  PoseSE3 local_pose_;  // base in odom

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_global_, sub_local_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomBroadcaster>());
  rclcpp::shutdown();
  return 0;
}

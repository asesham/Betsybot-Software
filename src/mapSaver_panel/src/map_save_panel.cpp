#include "mapSaver_panel/map_save_panel.hpp"

#include <QHBoxLayout>
#include <QMessageBox>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace betsybot;

MapSavePanel::MapSavePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  // UI
  auto * layout = new QVBoxLayout(this);
  start_button_ = new QPushButton("Start");
  stop_button_  = new QPushButton("Stop");
  reset_button_ = new QPushButton("Reset");
  layout->addWidget(start_button_);
  layout->addWidget(stop_button_);
  layout->addWidget(reset_button_);
  layout->addStretch();

  connect(start_button_, &QPushButton::released, this, &MapSavePanel::onStartButton);
  connect(stop_button_,  &QPushButton::released, this, &MapSavePanel::onStopButton);
  connect(reset_button_, &QPushButton::released, this, &MapSavePanel::onResetButton);
}

void MapSavePanel::onInitialize()
{
  ros_node_abstraction_ = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction_) {
    // Can't log with ros_node_ yet (not available), use fprintf as fallback
    fprintf(stderr, "[MapSavePanel] Failed to lock RosNodeAbstraction\n");
    return;
  }
  ros_node_ = ros_node_abstraction_->get_raw_node();
}

void MapSavePanel::resetBuffer()
{
  has_accum_ = false;
  accum_grid_ = nav_msgs::msg::OccupancyGrid{};
  resolution_ = 0.0;
}

void MapSavePanel::onStartButton()
{
  if (is_recording_) return;

  // Fresh session
  resetBuffer();

  try {
    map_sub_ = ros_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/grid_map", rclcpp::QoS(10),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
      {
        // Record EVERY message into the accumulated canvas
        if (!has_accum_) {
          accum_grid_ = *msg;
          resolution_ = msg->info.resolution;
          has_accum_ = true;
        } else {
          if (std::abs(msg->info.resolution - resolution_) > 1e-9) {
            RCLCPP_WARN(ros_node_->get_logger(),
              "Incoming resolution %.6f != accum resolution %.6f; ignoring this message.",
              msg->info.resolution, resolution_);
            return;
          }
          mergeIntoAccum(*msg);
        }
      }
    );
    is_recording_ = true;
    RCLCPP_INFO(ros_node_->get_logger(), "Recording started.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Failed to subscribe to /grid_map: %s", e.what());
  }
}

void MapSavePanel::onStopButton()
{
  if (!is_recording_) return;

  map_sub_.reset();
  is_recording_ = false;

  if (!has_accum_) {
    RCLCPP_WARN(ros_node_->get_logger(), "No buffered map to save.");
    return;
  }
  RCLCPP_INFO(ros_node_->get_logger(), "Recording stopped. Saving accumulated map...");
  if (saveCurrentMap()) {
    RCLCPP_INFO(ros_node_->get_logger(), "Saved map.png and map.yaml successfully.");
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "Failed to save map.");
  }
}

void MapSavePanel::onResetButton()
{
  if (is_recording_) {
    map_sub_.reset();
    is_recording_ = false;
    RCLCPP_INFO(ros_node_->get_logger(), "Recording aborted.");
  }
  resetBuffer();
  RCLCPP_INFO(ros_node_->get_logger(), "Map buffer cleared.");
}

// Enlarge canvas if needed so it covers both current accum and incoming msg bounds
void MapSavePanel::ensureCanvasFor(const nav_msgs::msg::OccupancyGrid & msg)
{
  const double res = resolution_;
  const auto & A = accum_grid_;
  const auto & B = msg;

  const double A_left   = A.info.origin.position.x;
  const double A_bottom = A.info.origin.position.y;
  const double A_right  = A_left   + A.info.width  * res;
  const double A_top    = A_bottom + A.info.height * res;

  const double B_left   = B.info.origin.position.x;
  const double B_bottom = B.info.origin.position.y;
  const double B_right  = B_left   + B.info.width  * res;
  const double B_top    = B_bottom + B.info.height * res;

  const double left   = std::min(A_left,   B_left);
  const double bottom = std::min(A_bottom, B_bottom);
  const double right  = std::max(A_right,  B_right);
  const double top    = std::max(A_top,    B_top);

  const uint32_t new_w = static_cast<uint32_t>(std::ceil((right - left) / res));
  const uint32_t new_h = static_cast<uint32_t>(std::ceil((top   - bottom) / res));

  const bool needs_resize =
    (new_w != A.info.width) || (new_h != A.info.height) ||
    std::abs(left   - A_left)   > 1e-12 ||
    std::abs(bottom - A_bottom) > 1e-12;

  if (!needs_resize) return;

  std::vector<int8_t> new_data(new_w * new_h, -1);

  const int off_x_old = static_cast<int>(std::llround((A_left   - left)   / res));
  const int off_y_old = static_cast<int>(std::llround((A_bottom - bottom) / res));

  for (uint32_t y = 0; y < A.info.height; ++y) {
    for (uint32_t x = 0; x < A.info.width; ++x) {
      const int8_t v = A.data[y * A.info.width + x];
      if (v < 0) continue;
      const uint32_t nx = static_cast<uint32_t>(x + off_x_old);
      const uint32_t ny = static_cast<uint32_t>(y + off_y_old);
      new_data[ny * new_w + nx] = v;
    }
  }

  accum_grid_.info.origin.position.x = left;
  accum_grid_.info.origin.position.y = bottom;
  accum_grid_.info.width  = new_w;
  accum_grid_.info.height = new_h;
  accum_grid_.data.swap(new_data);
}

// Merge B into accumulated A (A is already resized to fit)
void MapSavePanel::mergeIntoAccum(const nav_msgs::msg::OccupancyGrid & msg)
{
  // Warn if rotated (we assume axis-aligned / yaw ~ 0)
  tf2::Quaternion q(
    msg.info.origin.orientation.x,
    msg.info.origin.orientation.y,
    msg.info.origin.orientation.z,
    msg.info.origin.orientation.w
  );
  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);
  if (std::abs(y) > 1e-3) {
    static bool warned = false;
    if (!warned) {
      RCLCPP_WARN(ros_node_->get_logger(),
        "Incoming map has non-zero yaw (%.4f rad). Rotation is not handled; results may be misaligned.",
        y);
      warned = true;
    }
  }

  ensureCanvasFor(msg);

  const double res = resolution_;
  const int off_x = static_cast<int>(std::llround(
    (msg.info.origin.position.x - accum_grid_.info.origin.position.x) / res));
  const int off_y = static_cast<int>(std::llround(
    (msg.info.origin.position.y - accum_grid_.info.origin.position.y) / res));

  for (uint32_t yb = 0; yb < msg.info.height; ++yb) {
    for (uint32_t xb = 0; xb < msg.info.width; ++xb) {
      const int8_t val = msg.data[yb * msg.info.width + xb];
      if (val < 0) continue; // unknown -> ignore

      const uint32_t xa = static_cast<uint32_t>(xb + off_x);
      const uint32_t ya = static_cast<uint32_t>(yb + off_y);
      int8_t & dst = accum_grid_.data[ya * accum_grid_.info.width + xa];

      if (dst < 0) {
        dst = val;
      } else {
        // Merge rule: occupied wins; free if both free; else unknown-ish
        if (val >= occ_thresh_ || dst >= occ_thresh_) {
          dst = 100;
        } else if (val <= free_thresh_ && dst <= free_thresh_) {
          dst = 0;
        } else {
          // keep the "more occupied" tendency
          dst = std::max(dst, val);
        }
      }
    }
  }
}

bool MapSavePanel::saveCurrentMap()
{
  const std::string base_dir = "/home/betsybot/Betsybot-Software/src/betsybot/maps";
  std::error_code ec;
  std::filesystem::create_directories(base_dir, ec);

  const std::string png_path  = base_dir + "/map.png";
  const std::string yaml_path = base_dir + "/map.yaml";

  // Overwrite any existing files
  std::filesystem::remove(png_path,  ec);
  std::filesystem::remove(yaml_path, ec);

  const auto & map = accum_grid_;
  const uint32_t w = map.info.width;
  const uint32_t h = map.info.height;
  const double res = map.info.resolution;

  // Convert occupancy -> grayscale (0 black occ, 254 white free, 205 unknown)
  cv::Mat img(h, w, CV_8UC1);
  for (uint32_t y = 0; y < h; ++y) {
    for (uint32_t x = 0; x < w; ++x) {
      const int8_t v = map.data[(h - 1 - y) * w + x]; // flip y so origin is bottom-left
      uint8_t px = 205;
      if (v < 0) {
        px = 205;
      } else if (v >= occ_thresh_) {
        px = 0;
      } else if (v <= free_thresh_) {
        px = 254;
      } else {
        px = 205;
      }
      img.at<uint8_t>(y, x) = px;
    }
  }

  if (!cv::imwrite(png_path, img)) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Failed to write PNG: %s", png_path.c_str());
    return false;
  }

  // Extract yaw from origin
  tf2::Quaternion q(
    map.info.origin.orientation.x,
    map.info.origin.orientation.y,
    map.info.origin.orientation.z,
    map.info.origin.orientation.w
  );
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // YAML
  std::ofstream yaml(yaml_path);
  if (!yaml.is_open()) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Failed to write YAML: %s", yaml_path.c_str());
    return false;
  }
  yaml << "image: map.png\n";
  yaml << "mode: trinary\n";
  yaml << "resolution: " << std::fixed << std::setprecision(6) << res << "\n";
  yaml << "origin: [" 
       << std::fixed << std::setprecision(6)
       << map.info.origin.position.x << ", "
       << map.info.origin.position.y << ", "
       << yaw << "]\n";
  yaml << "negate: 0\n";
  yaml << "occupied_thresh: 0.65\n";
  yaml << "free_thresh: 0.196\n";
  yaml.close();

  return true;
}

// Export plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(betsybot::MapSavePanel, rviz_common::Panel)

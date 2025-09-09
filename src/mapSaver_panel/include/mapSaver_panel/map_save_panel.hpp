#ifndef MAPSAVER_PANEL__MAP_SAVE_PANEL_HPP_
#define MAPSAVER_PANEL__MAP_SAVE_PANEL_HPP_

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace betsybot
{

class MapSavePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MapSavePanel(QWidget * parent = nullptr) ;
  ~MapSavePanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onStartButton();
  void onStopButton();
  void onResetButton();

private:
  // Accumulation helpers
  void resetBuffer();
  void mergeIntoAccum(const nav_msgs::msg::OccupancyGrid & msg);
  void ensureCanvasFor(const nav_msgs::msg::OccupancyGrid & msg);

  // Save helpers
  bool saveCurrentMap();

  // RViz/ROS
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> ros_node_abstraction_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  // Buffer: accumulated full map
  nav_msgs::msg::OccupancyGrid accum_grid_;
  bool has_accum_{false};

  // UI
  QPushButton * start_button_{nullptr};
  QPushButton * stop_button_{nullptr};
  QPushButton * reset_button_{nullptr};

  // State / params
  bool is_recording_{false};
  double resolution_{0.0};
  int occ_thresh_{65};   // same thresholds used for PNG/YAML
  int free_thresh_{20};
};

}  // namespace betsybot

#endif  // MAPSAVER_PANEL__MAP_SAVE_PANEL_HPP_

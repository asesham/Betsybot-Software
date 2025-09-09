#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointsQoSRelay : public rclcpp::Node {
public:
  PointsQoSRelay() : Node("points_qos_relay") {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/lidar_points");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/lidar_points_reliable");
    int depth     = this->declare_parameter<int>("depth", 10);

    // Subscribe as "sensor data" (typically BestEffort for LiDAR)
    auto sub_qos = rclcpp::SensorDataQoS(); // BestEffort, KeepLast(5) by default
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sub_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
        pub_->publish(*msg);
      });

    // Publish as Reliable so strict subscribers can connect
    rclcpp::QoS pub_qos(depth);
    pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable)
           .durability(rclcpp::DurabilityPolicy::Volatile)
           .history(rclcpp::HistoryPolicy::KeepLast);

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);

    RCLCPP_INFO(get_logger(), "Relaying %s  ->  %s (Reliable)",
                input_topic_.c_str(), output_topic_.c_str());
  }

private:
  std::string input_topic_, output_topic_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointsQoSRelay>());
  rclcpp::shutdown();
  return 0;
}


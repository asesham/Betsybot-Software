#include <chrono>
#include <functional>
#include <string>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam(const rclcpp::NodeOptions & options) : Node("minimal_param_node", options)
  {
    //this->declare_parameter("my_parameter", "world");
    this->declare_parameter("yaw", 0.0);
    
    yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalParam::timer_callback, this));
    oorike_subscriber_ = this->create_subscription<std_msgs::msg::String>
     ("/color_detector/Trigger", 10, std::bind(&MinimalParam::oorike_callback, this, std::placeholders::_1));
  }

  void oorike_callback (const std_msgs::msg::String::SharedPtr msg)
  {}
  void timer_callback()
  {    
    std_msgs::msg::Float64 msg;
    msg.data = this->get_parameter("yaw").as_double();
    //std::string my_param = this->get_parameter("my_parameter").as_string();

    //RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    //std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    //this->set_parameters(all_new_parameters);

    yaw_publisher_->publish(msg);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr oorike_subscriber_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MinimalParam)
/*
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
*/

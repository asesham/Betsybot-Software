#include <cstdio>
#include <chrono> // For using 10ms
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp" //For Imu sensor message
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp" // For Imu angular Velocity Storage
#include "geometry_msgs/msg/quaternion.hpp" // For Imu angular Velocity Storage
#include "geometry_msgs/msg/twist.hpp" // For Imu angular Velocity Storage
#include <deque> // For deque
#include <cmath> // For abs() with floating-point numbers
#include <cstdlib> // // For abs() with integer numbers
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

//#define M_PI 3.141592653589793
uint16_t cal_window_size = 1000;

double convert_radians_to_180_degrees(double rad) {
    double deg = rad * (180.0 / M_PI);
    // Normalize to the -180 to 180 range
    deg = fmod(deg, 360.0);
    if (deg > 180.0) {
        deg -= 360.0;
    } else if (deg <= -180.0) {
        deg += 360.0;
    }
    return deg;
}

class SensorFusion : public rclcpp::Node
{
    public:
    SensorFusion() : Node("sensor_fusion"), count_(0), prev_yaw_angle(0.0), calibration_done(false), cal_avg_yaw(0.0), cal_yaw_sum(0.0)
    {
        this->declare_parameter("window_size", 50);
        this->declare_parameter("yaw", 0.0);
        this->declare_parameter("lin_vel", 0.0);
        this->declare_parameter("ang_vel", 0.0);
        
        rmw_qos_profile_t qos = rmw_qos_profile_default;
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/camera/camera/gyro/sample", 5, std::bind(&SensorFusion::imu_sub_callback, this, std::placeholders::_1));
        
        //imu_filtered_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/sensor/imu_filtered", \ 
        //                                                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
        imu_filtered_publisher_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/sensor_fusion/yaw", \ 
                                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
        //cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/motion_planning/cmd_vel", \
                                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
                                                        
        timer_ = this->create_wall_timer(10ms, std::bind(&SensorFusion::imu_pub_callback, this));
    }
    
    private:
    void imu_sub_callback(sensor_msgs::msg::Imu::SharedPtr data)
    {
        window_size = this->get_parameter("window_size").as_int();
        
        int32_t time_sec_diff = data->header.stamp.sec - prev_secs;
        int32_t time_nanosec_diff = data->header.stamp.nanosec - prev_nanosecs;
        int time_diff = time_sec_diff * 1000000000 + time_nanosec_diff;
        
        prev_secs = data->header.stamp.sec;
        prev_nanosecs = data->header.stamp.nanosec;
        
        if (!calibration_done)
        {
            if ( cal_count >= cal_window_size)
            {
                calibration_done = true;
                cal_avg_yaw = (float)cal_yaw_sum/cal_window_size;
            }
            else
            {
                cal_count++;
                RCLCPP_INFO(this->get_logger(), "Cal Not Done.... cal count = %d", cal_count);
		cal_yaw_sum += data->angular_velocity.y;
                return;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Calibration Done!!! cal_avg_yaw = %f cal_yaw_sum = %f", cal_avg_yaw, cal_yaw_sum);
        sum_.x += data->angular_velocity.x;
        sum_.y += data->angular_velocity.y;
        sum_.z += data->angular_velocity.z;
        imu_data.push_back(data->angular_velocity);
        
        while (imu_data.size() > window_size)
        {
            sum_.x -= imu_data.front().x;
            sum_.y -= imu_data.front().y;
            sum_.z -= imu_data.front().z;
            
            imu_data.pop_front();
        } 
        
        imu_filtered_data.x = sum_.x/imu_data.size(); // Pitch Angular Velocity
        imu_filtered_data.z = sum_.z/imu_data.size(); // Roll Angular Velocity
        
        imu_filtered_data.y = sum_.y/imu_data.size() - cal_avg_yaw; // Yaw Angular Velocity
        
        /* Yaw angular velocity Noise attenuation */
        if (abs(imu_filtered_data.y) < 0.001)
        {
            count_++;
        }
        else
        {
            count_ = 0;
        }
        
        
        if (count_ > window_size)
        {
            imu_filtered_data.y = imu_filtered_data_prev_y_*0.95;
            if (abs(imu_filtered_data_prev_y_) < 0.0001)
                imu_filtered_data.y = 0;
        }
        imu_filtered_data_prev_y_ = imu_filtered_data.y;
        /* Yaw angular velocity Noise attenuation */
        
        /* calculate yaw angle */
        angles.y = prev_yaw_angle + (double)(imu_filtered_data.y * ((double)time_diff/1000000000));//) * ;
        
        angle_degrees.y = convert_radians_to_180_degrees(angles.y);
        RCLCPP_INFO(this->get_logger(), "Prev Yaw : %f, ang Vel: %f, Yaw: %f cal done", prev_yaw_angle, imu_filtered_data.y, angle_degrees.y);
        prev_yaw_angle = angles.y;
    }
    
    void imu_pub_callback()
    {
        //yaw_parameter = static_cast<float>(this->get_parameter("yaw").as_double());
        //lin_vel = static_cast<float>(this->get_parameter("lin_vel").as_double());
        //ang_vel = static_cast<float>(this->get_parameter("ang_vel").as_double());
        //RCLCPP_INFO(this->get_logger(), "Publishing: %f, %f, %f, %d", imu_filtered_data.x, imu_filtered_data.y, imu_filtered_data.z, imu_data.size());
        geometry_msgs::msg::Quaternion angles_test;
        angles_test.x = 0.0;
        angles_test.y = angles.y;
        angles_test.z = 0.0;
        angles_test.w = 0.0;
        imu_filtered_publisher_->publish(angles_test);
        
        //geometry_msgs::msg::Twist command;
        //command.linear.x = lin_vel;
        //command.linear.y = 0.0;
        //command.linear.z = 0.0;
        //command.angular.x = 0.0;
        //command.angular.y = 0.0;
        //command.angular.z = ang_vel;
        //cmd_vel_publisher_->publish(command);
        
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_filtered_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr imu_filtered_publisher_;
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    std::deque<geometry_msgs::msg::Vector3> imu_data;
    geometry_msgs::msg::Vector3 imu_filtered_data;
    float imu_filtered_data_prev_y_;
    geometry_msgs::msg::Vector3 angles;
    geometry_msgs::msg::Quaternion angles_test;
    geometry_msgs::msg::Vector3 angle_degrees;
    double prev_yaw_angle;
    geometry_msgs::msg::Vector3 sum_;
    int count_;
    int32_t prev_secs;
    uint32_t prev_nanosecs;
    int window_size;
    bool calibration_done;
    uint16_t cal_count;
    float cal_avg_yaw;
    float cal_yaw_sum;
    float yaw_parameter;
    float lin_vel;
    float ang_vel;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorFusion>());
  rclcpp::shutdown();
  return 0;
}


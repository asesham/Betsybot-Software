#include <cstdio>
#include <chrono> // For using 10ms
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp" //For Imu sensor message
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp" // For Imu angular Velocity Storage
#include <deque> // For deque
#include <cmath> // For abs() with floating-point numbers
#include <cstdlib> // // For abs() with integer numbers
#include <rclcpp/qos.hpp>

#define ENABLE_MOTORS 1
#if ENABLE_MOTORS
#include "ctre/phoenix6/TalonFX.hpp"
#endif
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp" // for FeedEnable
#include "differential-drive/pid.hpp"
#include "geometry_msgs/msg/twist.hpp" // For Imu angular Velocity Storage

using namespace std::chrono_literals;
#if ENABLE_MOTORS
using namespace ctre::phoenix6;
#endif

float lin_vel_kp = 1;
float lin_vel_ki = 0;
float lin_vel_kd = 0;
float ang_vel_kp = 1;
float ang_vel_ki = 0;
float ang_vel_kd = 0;
float yaw_angle_kp = 1;
float yaw_angle_ki = 0;
float yaw_angle_kd = 0;

constexpr char const *CANBUS_NAME = "can1";

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


class DifferentialDrive : public rclcpp::Node
{
public:

    DifferentialDrive() : Node("differential_drive")
    {
        // PID settings
        
        // Yaw angle PID setting     
        yaw_pid = new PID(1.0, 0.0, 0.0);
#if ENABLE_MOTORS
        // Motor Config settings
        configs::TalonFXConfiguration fx_cfg{};

        // the left motor is CCW+
        fx_cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
        leftMotor.GetConfigurator().Apply(fx_cfg);

        // the right motor is CW+
        fx_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
        rightMotor.GetConfigurator().Apply(fx_cfg);
#endif
        // Motor speed limits
        max_motor_limit = 0.9;
        min_motor_limit = -0.9;
        
        // Set Workspace Variables
        drive_straight = false;
        count_ = 0;
        
        // Subscribers -> Yaw angle, Imu, command velocity
        //   Service -> red/green signal from camera
        yaw_subscription_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
        "/sensor_fusion/yaw", 5, std::bind(&DifferentialDrive::yaw_angle_callback, this, std::placeholders::_1));
        
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/motion_planning/cmd_vel", 5, std::bind(&DifferentialDrive::cmd_vel_callback, this, std::placeholders::_1));

        // Periodic Function to Drive
        timer_ = this->create_wall_timer(10ms, std::bind(&DifferentialDrive::drive, this));
    }
    
    void setMaxMotorLimit(double limit) { max_motor_limit = limit; }
    void setMinMotorLimit(double limit) { min_motor_limit = limit; }
#if ENABLE_MOTORS
    // devices
    hardware::TalonFX leftMotor{1, CANBUS_NAME};
    hardware::TalonFX rightMotor{2, CANBUS_NAME};

    // control requests
    controls::DutyCycleOut leftOut{0};
    controls::DutyCycleOut rightOut{0};
#endif
private:

    void yaw_angle_callback(geometry_msgs::msg::Quaternion::SharedPtr data)
    {
        yaw_angle = data->y;
    }
    
    void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        lin_vel = cmd->linear.x;
        ang_vel = cmd->angular.z;
    }
    
    void setMotorSpeeds(double left_speed, double right_speed)
    {
        RCLCPP_INFO(this->get_logger(), "Left_speed = %f right_speed = %f", left_speed, right_speed);
        // Left Motor speed limiting 
        if (left_speed > max_motor_limit)
            left_speed = max_motor_limit;
        else if (left_speed < min_motor_limit)
            left_speed = min_motor_limit;
         
        // Right Motor speed limiting
        if (right_speed > max_motor_limit)
            right_speed = max_motor_limit;
        else if (right_speed < min_motor_limit)
            right_speed = min_motor_limit; 
#if ENABLE_MOTORS
        // Set Motor Speeds
        leftOut.Output = 0.04; //left_speed;
        rightOut.Output = -0.04; //right_speed;
        //leftMotor.SetControl(leftOut);
        //rightMotor.SetControl(rightOut);
#endif
    }
    
    void drive()
    {
        ctre::phoenix::unmanaged::FeedEnable(100);
        if (abs(ang_vel) < 0.0001)
        {
            // Wait for 25 counts or 250ms to get the stable yaw angle 
            // Reason for 250ms wait is the window size used for averaging in 
            // Sensor Fusion Module
            if (!drive_straight)
            {
                if (count_ < 25)
                    count_++;
                else
                {
                   drive_straight = true;
                   count_ = 0;
                   desired_yaw = yaw_angle;
                }
            }
            else // Once drive_straight is true, you have the stable yaw angle
            {
                double yaw_error = (double)(desired_yaw - yaw_angle);
                double pid_motor_correction = yaw_pid->getErrorOutput(yaw_error);
                RCLCPP_INFO(this->get_logger(), "yaw_error = %f pid_motor_correction = %f", yaw_error, pid_motor_correction);
                double left_speed = lin_vel + pid_motor_correction;
                double right_speed = lin_vel - pid_motor_correction;
                setMotorSpeeds(left_speed, right_speed);
            }
        }
        else
        {
            drive_straight = false;
            count_ = 0;
        }        
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_filtered_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr yaw_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    
    double max_motor_limit;
    double min_motor_limit;
    
    float lin_vel;
    float ang_vel;
    float yaw_angle;
    
    PID* yaw_pid;
    float desired_yaw;
    bool drive_straight;
    int count_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DifferentialDrive>());
  rclcpp::shutdown();
  return 0;
}


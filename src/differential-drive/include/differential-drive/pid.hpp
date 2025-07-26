#include <chrono>
#include <stdint.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>

class PID
{
public:
    PID(double p, double i, double d) : kp(p), ki(i), kd(d), integral_term(0.0)
    {
        //prev_time = system_clock.now();
        prev_time = std::chrono::high_resolution_clock::now();
        prev_err = 0.0;
    }
    
    double getErrorOutput(double err)
    {
        auto cur_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dt_duration = cur_time - prev_time;
        double dt = dt_duration.count();
        //RCLCPP_INFO(logger, "dt = %f", dt);
        std::cout << "dt = " << dt << std::endl;
        
        //rclcpp::Time cur_time = system_clock.now();
        //rclcpp::Duration dt_duration = cur_time - prev_time;
        //double dt = dt_duration.seconds();
        
        prev_time = cur_time;
        
        integral_term += ki * err * dt;
        
        std::cout << "integral_term = " << integral_term << std::endl;
        //RCLCPP_INFO(logger, "integral_term = %f", integral_term);
        /*
        if (integral_term > integral_limit )
            integral_term = integral_limit;
        else if (integral_term < -integral_limit)
            integral_term = -integral_limit;
        */
        
        double err_out = kp * err + integral_term + kd * (err - prev_err) * dt;
        
        std::cout << "kp = " << kp << ", ki = " << ki << ", kd = " << kd << std::endl;
        std::cout << "kp = " << kp << ", err = " << err << std::endl;
        std::cout << "kd = " << kd << ", err = " << err << ", prev_err = " << prev_err << ", dt = " << dt << std::endl;
        std::cout << "err_out = " << err_out << std::endl;
        //RCLCPP_INFO(logger, "kd = %f, err = %f, prev_err = %f, dt = %f", kp, err, prev_err, dt);
        //RCLCPP_INFO(logger, "err_out = %f", err_out);
        prev_err = err;
        
        return err_out;
    }
    
    void setIntegralLimit(double value) { integral_limit = value; }
    void setKP(double value) { kp = value; }
    void setKI(double value) { ki = value; }
    void setKD(double value) { kd = value; }
    
private:
    double kp;
    double ki;
    double kd;
    
    double integral_limit;
    
    double prev_err;
    double integral_term;
    
    //rclcpp::Clock system_clock;
    //rclcpp::Time prev_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> prev_time;
};

/* REFERENCE 
#include <chrono> // For time tracking, if using high-resolution timers

class PIDController {
public:
    // Constructor
    PIDController(double kp, double ki, double kd, double min_output, double max_output, double integral_limit)
        : Kp(kp), Ki(ki), Kd(kd), minOutput(min_output), maxOutput(max_output), integralLimit(integral_limit),
          previousError(0.0), integralTerm(0.0), previousTime(std::chrono::high_resolution_clock::now()) {}

    // Method to calculate the control output
    double calculate(double setpoint, double measurement) {
        // Calculate time difference
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dt_duration = currentTime - previousTime;
        double dt = dt_duration.count();
        previousTime = currentTime;

        // Calculate error
        double error = setpoint - measurement;

        // Proportional term
        double proportionalTerm = Kp * error;

        // Integral term with anti-windup
        integralTerm += Ki * error * dt;
        if (integralTerm > integralLimit) {
            integralTerm = integralLimit;
        } else if (integralTerm < -integralLimit) {
            integralTerm = -integralLimit;
        }

        // Derivative term (with simple derivative on error)
        double derivativeTerm = Kd * (error - previousError) / dt;

        // Calculate total output
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Apply output limits
        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < minOutput) {
            output = minOutput;
        }

        // Store current error for next iteration
        previousError = error;

        return output;
    }

    // Method to reset the controller state (e.g., for re-initialization)
    void reset() {
        previousError = 0.0;
        integralTerm = 0.0;
        previousTime = std::chrono::high_resolution_clock::now();
    }

private:
    double Kp; // Proportional gain
    double Ki; // Integral gain
    double Kd; // Derivative gain

    double minOutput;     // Minimum allowed output value
    double maxOutput;     // Maximum allowed output value
    double integralLimit; // Limit for the integral term to prevent wind-up

    double previousError; // Error from the previous calculation
    double integralTerm;  // Accumulated integral term
    std::chrono::time_point<std::chrono::high_resolution_clock> previousTime; // Time of the previous calculation
};
*/

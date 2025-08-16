import rclpy
from rclpy.node import Node
# from rclpy.exceptions import ParameterNotDeclaredException
# from rcl_interfaces.msg import ParameterType
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
#

import time
import Jetson.GPIO as GPIO 
#
DEAD_BAND = 0.0

    
class ControllerDrive(Node):

    def __init__(self):
        super().__init__('controller_pub')
        # Publish Velocities to CAN cotroller
        #self.moveControl_pub = self.create_publisher(Float32, 'linearVel', 10)
        #self.spinControl_pub = self.create_publisher(Float32, 'angularVel', 10)
        self.ControlCmd_pub = self.create_publisher(Twist, '/motion_planning/cmd_vel',10)
        
        #  ==== GPIO CONFIGURATION ====
        self.LINEAR_PIN = 16   # Board pin number (change to match your wiring)
        self.ANGULAR_PIN = 13   # Board pin number (change to match your wiring)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.LINEAR_PIN, GPIO.IN)
        GPIO.setup(self.ANGULAR_PIN, GPIO.IN)
        #
        self.declare_parameter('k_gain', 0.15) # Declare Kalman Filter Weight value
        self.kGain_ = self.get_parameter('k_gain').value

        self.prev_lin_cmd = 0.0
        self.prev_saved_lin_cmd = 0.0
        self.lin_cmd_count = 0
        self.prev_avg_lin_cmd = None
        self.prev_avg_ang_cmd = None
        self.prev_ang_cmd = 0.0
        self.prev_saved_ang_cmd = 0.0
        self.ang_cmd_count = 0
        self.lin_cmd = 0.0
        self.ang_cmd = 0.0
        self.send_publish = False
        self.lin_calibration_done = False
        self.ang_calibration_done = False
        self.LINEAR_DEAD_BAND = 0.0
        self.ANGULAR_DEAD_BAND = 0.0
        self.msg = Twist()
        # Timer to RUN callbacks
        self.timer_period = 0.005  # seconds
        self.timer = self.create_timer(self.timer_period, self.LinearPinDetect)
        self.timer = self.create_timer(self.timer_period, self.AngularPinDetect)
        
    def get_cmd_vel(self, cmd_in):
        cmd = 0.0
        sign = (int) (cmd_in/abs(cmd_in))
        #print(sign)
        #self.send_publish = True
        #return (sign * ((float)((int)(self.lin_cmd * 10))/10))
        
        if (cmd_in > 0.0 and
            cmd_in <= 0.2):
            cmd = 0.0
        elif(cmd_in < 0.0 and
            cmd_in >= -0.15):
            cmd = 0.0
        elif(cmd_in < -0.15 and
            cmd_in >= -0.2):
            cmd = -0.1
        elif (abs(cmd_in) > 0.2 and
              abs(cmd_in) <= 0.3):
            cmd = sign * 0.2
        elif (abs(cmd_in) > 0.3 and
              abs(cmd_in) <= 0.4):
            cmd = sign * 0.3
        elif (abs(cmd_in) > 0.4 and
              abs(cmd_in) <= 0.5):
            cmd = sign * 0.4
        elif (abs(cmd_in) > 0.5 and
              abs(cmd_in) <= 0.6):
            cmd = sign * 0.5
        elif (abs(cmd_in) > 0.6 and
              abs(cmd_in) <= 0.7):
            cmd = sign * 0.6
        elif (abs(cmd_in) > 0.7 and
              abs(cmd_in) <= 0.8):
            cmd = sign * 0.7
        elif (abs(cmd_in) > 0.8 and
              abs(cmd_in) <= 0.9):
            cmd = sign * 0.8
        elif (abs(cmd_in) > 0.8 and
              abs(cmd_in) <= 0.9):
            cmd = sign * 0.9
        self.send_publish = True
        return cmd

    def LinearPinDetect(self):
        #
        last_rising = None
        period = None
        cal_count = 0
        dutybuff = 0.0
        init_mean = 0.0
        new_mean = 0.0
        max_mean = None
        min_mean = None
        linear_msg = Float32()
        angular_msg = Float32()
        duty = 0.0
        try:
            if (self.prev_avg_lin_cmd is not None):
                init_mean = self.prev_avg_lin_cmd
                duty = init_mean + self.LINEAR_DEAD_BAND
            start_time = time.time()  # Record the start time
            duration_ms = 100        # Target duration in milliseconds
            target_time = start_time + (duration_ms / 1000) # Convert milliseconds to seconds

            while (not self.lin_calibration_done or (self.lin_calibration_done and time.time() < target_time)):
                # Wait for rising edge
                GPIO.wait_for_edge(self.LINEAR_PIN, GPIO.RISING)
                now = time.time()
                # Calculate period (time between rising edges)
                if last_rising is not None:
                    period = now - last_rising
                last_rising = now

                # Wait for falling edge to measure high time
                GPIO.wait_for_edge(self.LINEAR_PIN, GPIO.FALLING)
                fall_time = time.time()

                if period:
                    high_time = fall_time - last_rising
                    freq = 1.0 / period
                    duty = (high_time / period) * 100.0
                    if duty > 11.0 or duty < 4.0:
                        continue
                    dutybuff = dutybuff + duty
                    #self.get_logger().info(f"Linear movments -> Freq: {freq:8.2f} Hz | Duty: {duty:5.2f}%")
                else:
                    self.get_logger().info("Waiting for Linear stable signal...")
                    
                if not self.lin_calibration_done:
                    if cal_count >= 500:
                        self.get_logger().info(f"Linear Calibration Done!!!!")
                        self.lin_calibration_done = True
                        init_mean = dutybuff / cal_count
                        self.LINEAR_DEAD_BAND = init_mean - 7.5
                        dutybuff = 0
                    else:
                        cal_count = cal_count + 1

                if init_mean > 0.0:
                    new_mean = (init_mean + 0.1* (duty - self.LINEAR_DEAD_BAND)) / (1+ 0.1)           
                    
                
                if self.lin_calibration_done:
                    msg = Twist()
                    for_print = self.lin_cmd = 1.4/5 * (7.5 - new_mean)
                    #self.get_logger().info(f"Command Lin Velocity = {for_print:5.2f} Duty = {duty:5.2f} New Mean = {new_mean:5.2f} init_mean = {init_mean:5.2f}") 
                    cmd = self.get_cmd_vel(self.lin_cmd)
                    self.msg.linear.x = self.prev_saved_lin_cmd
                    if (self.prev_lin_cmd == cmd):
                        self.lin_cmd_count += 1
                        if self.lin_cmd_count >=5:
                            self.msg.linear.x = cmd
                            self.prev_saved_lin_cmd = cmd
                    else:
                        self.lin_cmd_count = 0
                    #self.msg.linear.x = self.get_cmd_vel(self.lin_cmd)
                    #linear_msg.data = new_mean
                    #self.moveControl_pub.publish(linear_msg)
                    self.prev_lin_cmd = cmd
                    #if(self.send_publish):     
                    self.ControlCmd_pub.publish(self.msg)
                
                init_mean = new_mean
                #if duty > 0.0:
                #    angular_msg.data = duty
                #    self.spinControl_pub.publish(angular_msg)
                    
            
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")
        except KeyboardInterrupt:
            self.get_logger().info("Node interrupted by user (Ctrl+C).")

        self.prev_avg_lin_cmd = init_mean
        
        
    def AngularPinDetect(self):
        #
        last_rising = None
        period = None
        cal_count = 0
        dutybuff = 0.0
        init_mean = 0.0
        new_mean = 0.0
        max_mean = None
        min_mean = None
        linear_msg = Float32()
        angular_msg = Float32()
        duty = 0.0
        try:
            if (self.prev_avg_ang_cmd is not None):
                init_mean = self.prev_avg_ang_cmd
                duty = init_mean + self.ANGULAR_DEAD_BAND
                
            start_time = time.time()  # Record the start time
            duration_ms = 100        # while loop duration in milliseconds
            target_time = start_time + (duration_ms / 1000) # Convert milliseconds to seconds

            while (not self.ang_calibration_done or (self.ang_calibration_done and time.time() < target_time)):
                # Wait for rising edge
                GPIO.wait_for_edge(self.ANGULAR_PIN, GPIO.RISING)
                now = time.time()
                # Calculate period (time between rising edges)
                if last_rising is not None:
                    period = now - last_rising
                last_rising = now

                # Wait for falling edge to measure high time
                GPIO.wait_for_edge(self.ANGULAR_PIN, GPIO.FALLING)
                fall_time = time.time()

                if period:
                    high_time = fall_time - last_rising
                    freq = 1.0 / period
                    duty = (high_time / period) * 100.0
                    if duty > 11.0 or duty < 4.0:
                        continue
                    dutybuff = dutybuff + duty
                    #self.get_logger().info(f"Linear movments -> Freq: {freq:8.2f} Hz | Duty: {duty:5.2f}%")
                else:
                    self.get_logger().info("Waiting for Angular stable signal...")
                    
                if not self.ang_calibration_done:
                    if cal_count >= 500:
                        self.get_logger().info(f"Angular Calibration Done!!!!")
                        self.ang_calibration_done = True
                        init_mean = dutybuff / cal_count
                        self.ANGULAR_DEAD_BAND = init_mean - 7.5
                        dutybuff = 0
                    else:
                        cal_count = cal_count + 1

                if init_mean > 0.0:
                    new_mean = (init_mean + 0.1* (duty - self.ANGULAR_DEAD_BAND)) / (1+ 0.1)           
                    
                
                if self.ang_calibration_done:
                    msg = Twist()
                    for_print = self.ang_cmd = 1.4/5 * (new_mean - 7.5)
                    #self.get_logger().info(f"Command Ang Velocity = {for_print:5.2f} Duty = {duty:5.2f} New Mean = {new_mean:5.2f} init_mean = {init_mean:5.2f}") 
                    cmd = self.get_cmd_vel(self.ang_cmd)
                    self.msg.angular.z = self.prev_saved_ang_cmd
                    if (self.prev_ang_cmd == cmd):
                        self.ang_cmd_count += 1
                        if self.ang_cmd_count >=5:
                            self.msg.angular.z = cmd
                            self.prev_saved_ang_cmd = cmd
                    else:
                        self.ang_cmd_count = 0
                    #self.msg.angular.z = self.get_cmd_vel(self.ang_cmd)
                    self.prev_ang_cmd = cmd
             
                    if(self.send_publish):     
                        self.ControlCmd_pub.publish(self.msg)
                
                init_mean = new_mean
                #if duty > 0.0:
                #    angular_msg.data = duty
                #    self.spinControl_pub.publish(angular_msg)
                    
            
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")
        except KeyboardInterrupt:
            self.get_logger().info("Node interrupted by user (Ctrl+C).")

        self.prev_avg_ang_cmd = init_mean
        
def main(args=None):
    rclpy.init(args=args)
    node = ControllerDrive()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


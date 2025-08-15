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

        # Timer to RUN callbacks
        # self.timer_period = 0.001  # seconds
        # self.timer = self.create_timer(self.timer_period, self.LinearPinDetect)
        # self.timer = self.create_timer(self.timer_period, self.AngularPinDetect)
        

    def LinearPinDetect(self):
        #
        last_rising = None
        period = None
        count = 0
        dutybuff = 0.0
        init_mean = 0.0
        new_mean = 0.0
        max_mean = None
        min_mean = None
        linear_msg = Float32()
        angular_msg = Float32()
        duty = 0.0
        calibration_done = False
        try:
            while True:
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
                    self.get_logger().info("Waiting for stable signal...")
                    
                if not calibration_done:
                    if count >= 1000:
                        self.get_logger().info(f"Calibration Done!!!!")
                        calibration_done = True
                        init_mean = dutybuff / count
                        dutybuff = 0
                    else:
                        count = count + 1

                if init_mean > 0.0:
                    new_mean = (init_mean + 0.15* (duty - (init_mean -7.5))) / (1+ 0.15)           

                #self.get_logger().info(f"New mean =  {new_mean:5.2f}% Init mean = {init_mean:5.2f}%, duty={duty:5.2f}")
                #if new_mean > 0.1:
                #    if max_mean is None or new_mean > max_mean:
                #        max_mean = new_mean
                #    if min_mean is None or new_mean < min_mean:
                #        min_mean = new_mean
                #    self.get_logger().info(f"Max mean =  {max_mean:5.2f}% Min mean = {min_mean:5.2f}%")
                init_mean = new_mean
                #
                # Publish Linear Controller Data
                msg = Twist()
                cmd = msg.linear.x = 1.4/5 * (new_mean - 7.5)
                self.get_logger().info(f"Command Lin Velocity = {cmd:5.2f}") 
                msg.angular.z = 0.0
                #linear_msg.data = new_mean
                #self.moveControl_pub.publish(linear_msg)
                self.ControlCmd_pub.publish(msg)
                
                #if duty > 0.0:
                #    angular_msg.data = duty
                #    self.spinControl_pub.publish(angular_msg)
                    

        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")
        except KeyboardInterrupt:
            self.get_logger().info("Node interrupted by user (Ctrl+C).")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerDrive()
    node.LinearPinDetect()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


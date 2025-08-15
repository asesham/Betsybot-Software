import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import Jetson.GPIO as GPIO

# === CONFIG ===
LINEAR_PIN = 13       # BOARD pin number for Linear control
#ANGULAR_PIN = 13      # BOARD pin number for Angular control
LINEAR_GAIN = 0.15
ANGULAR_GAIN = 0.15
DUTY_CENTER = 7.5     # Neutral duty cycle (%)
DUTY_MAX_ALLOWED = 11.0
VELOCITY_SCALE = 1.4 / 5.0
SAMPLE_INTERVAL = 0.0005  # 0.5 ms sampling interval

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LINEAR_PIN, GPIO.IN)
    #GPIO.setup(ANGULAR_PIN, GPIO.IN)
    last_rising = None
    period = None
    freq = 0.0
    duty = 0.0
    max_duty_cycle = None
    max_frequency = None
    min_duty_cycle = None
    min_frequency = None
    last_state = GPIO.input(LINEAR_PIN)
    high_start = None
    try:
        while True:
            current_state = GPIO.wait_for_edge(LINEAR_PIN, GPIO.RISING)
            now = time.time()

            # Detect rising edge
            #if current_state == GPIO.HIGH and last_state == GPIO.LOW:
            if last_rising is not None:
                period = now - last_rising
            last_rising = now
            high_start = now

            current_state = GPIO.wait_for_edge(LINEAR_PIN, GPIO.FALLING)
            fall_time = time.time()
            if period:
                high_time = fall_time - last_rising
                freq = 1.0 / period
                duty = (high_time / period) * 100.0
                print("Sideways pin")
                print(f"Freq: {freq:8.2f} Hz | Duty: {duty:5.2f}%")
            else:
                print("Waiting for stable signal...")
            # Detect falling edge
            #elif current_state == GPIO.LOW and last_state == GPIO.HIGH:
            #    if high_start is not None and period:
            #        high_time = now - high_start
            #        duty = (high_time / period) * 100.0
            #        freq = 1/period
                    
            print(f"Freq:{freq:5.2f}Hz Duty: {duty:5.2f}%")
            
    except KeyboardInterrupt:
        print("\nExiting...")
        GPIO.cleanup()
"""
                    if duty > DUTY_MAX_ALLOWED:
                        last_state = current_state
                        return None

                    if not calibration_done:
                        duty_sum += duty
                        count += 1
                        if count >= 500:  # Calibration period
                            init_mean = self.duty_sum / self.count
                            calibration_done = True
                            duty_sum = 0
                            print(f"{self.label} Calibration Done: {self.init_mean:.2f}%")
                        self.last_state = current_state
                        return None

class PWMReader:
    # A helper class to track PWM edges and duty cycle.
    def __init__(self, pin, gain, label):
        self.pin = pin
        self.gain = gain
        self.label = label

        self.last_state = GPIO.input(pin)
        self.last_rising = None
        self.high_start = None
        self.period = None
        self.init_mean = 0.0
        self.duty_sum = 0.0
        self.count = 0
        self.calibration_done = False

    def update(self):
        #Check the pin state, update duty cycle, and return velocity or None.
        current_state = GPIO.input(self.pin)
        now = time.time()

        # Detect rising edge
        if current_state == GPIO.HIGH and self.last_state == GPIO.LOW:
            if self.last_rising is not None:
                self.period = now - self.last_rising
            self.last_rising = now
            self.high_start = now

        # Detect falling edge
        elif current_state == GPIO.LOW and self.last_state == GPIO.HIGH:
            if self.high_start is not None and self.period:
                high_time = now - self.high_start
                duty = (high_time / self.period) * 100.0

                if duty > DUTY_MAX_ALLOWED:
                    self.last_state = current_state
                    return None

                if not self.calibration_done:
                    self.duty_sum += duty
                    self.count += 1
                    if self.count >= 500:  # Calibration period
                        self.init_mean = self.duty_sum / self.count
                        self.calibration_done = True
                        self.duty_sum = 0
                        print(f"{self.label} Calibration Done: {self.init_mean:.2f}%")
                    self.last_state = current_state
                    return None

                new_mean = (self.init_mean + self.gain * (duty - (self.init_mean - DUTY_CENTER))) / (1 + self.gain)
                self.init_mean = new_mean
                velocity = VELOCITY_SCALE * (new_mean - DUTY_CENTER)
                self.last_state = current_state
                return velocity

        self.last_state = current_state
        return None

class ControllerDrive(Node):
    def __init__(self):
        super().__init__('joystick_drive_node')

        # ROS publishers
        self.move_pub = self.create_publisher(Float32, 'linearVel', 10)
        self.spin_pub = self.create_publisher(Float32, 'angularVel', 10)

        # GPIO setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LINEAR_PIN, GPIO.IN)
        GPIO.setup(ANGULAR_PIN, GPIO.IN)

        # PWM Readers
        self.linear_reader = PWMReader(LINEAR_PIN, LINEAR_GAIN, "Linear")
        #self.angular_reader = PWMReader(ANGULAR_PIN, ANGULAR_GAIN, "Angular")

        # Timers (run at high frequency)
        self.create_timer(SAMPLE_INTERVAL, self.linear_loop)
        #self.create_timer(SAMPLE_INTERVAL, self.angular_loop)

    def linear_loop(self):
        vel = self.linear_reader.update()
        if vel is not None:
            msg = Float32()
            msg.data = vel
            self.move_pub.publish(msg)

    def angular_loop(self):
        vel = self.angular_reader.update()
        if vel is not None:
            msg = Float32()
            msg.data = vel
            self.spin_pub.publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()
"""
if __name__ == "__main__":
    main()

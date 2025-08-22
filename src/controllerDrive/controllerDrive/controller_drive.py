import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO


class ControllerDrive(Node):
    def __init__(self):
        super().__init__('controller_pub')

        # Publisher for velocity commands
        self.control_cmd_pub = self.create_publisher(Twist, '/motion_planning/cmd_vel', 10)

        # GPIO configuration
        self.LINEAR_PIN = 16   # Board pin number (adjust to wiring)
        self.ANGULAR_PIN = 13  # Board pin number (adjust to wiring)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.LINEAR_PIN, GPIO.IN)
        GPIO.setup(self.ANGULAR_PIN, GPIO.IN)

        # Parameters
        self.declare_parameter('k_gain', 0.15)
        self.k_gain = self.get_parameter('k_gain').value

        # State
        self.msg = Twist()
        self.send_publish = False

        # Linear channel state
        self.lin_state = {
            "pin": self.LINEAR_PIN,
            "calibration_done": False,
            "dead_band": 0.0,
            "prev_avg": None,
            "prev_cmd": 0.0,
            "prev_saved": 0.0,
            "cmd_count": 0,
            "current_cmd": 0.0,
            "scale": lambda mean: 1.4 / 5 * (7.5 - mean),
            "assign": lambda msg, val: setattr(msg.linear, "x", val)
        }

        # Angular channel state
        self.ang_state = {
            "pin": self.ANGULAR_PIN,
            "calibration_done": False,
            "dead_band": 0.0,
            "prev_avg": None,
            "prev_cmd": 0.0,
            "prev_saved": 0.0,
            "cmd_count": 0,
            "current_cmd": 0.0,
            "scale": lambda mean: 1.4 / 5 * (mean - 7.5),
            "assign": lambda msg, val: setattr(msg.angular, "z", val)
        }

        # Timers
        self.timer_period = 0.005
        self.create_timer(self.timer_period, lambda: self.process_channel(self.lin_state, "Linear"))
        self.create_timer(self.timer_period, lambda: self.process_channel(self.ang_state, "Angular"))

    def exit_node(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.control_cmd_pub.publish(msg)

    def get_cmd_vel(self, cmd_in: float) -> float:
        cmd = 0.0
        sign = int(cmd_in / abs(cmd_in))

        if 0.0 < cmd_in <= 0.2:
            cmd = 0.0
        elif -0.15 <= cmd_in < 0.0:
            cmd = 0.0
        elif -0.2 <= cmd_in < -0.15:
            cmd = -0.1
        elif 0.2 < abs(cmd_in) <= 0.3:
            cmd = sign * 0.2
        elif 0.3 < abs(cmd_in) <= 0.4:
            cmd = sign * 0.3
        elif 0.4 < abs(cmd_in) <= 0.5:
            cmd = sign * 0.4
        elif 0.5 < abs(cmd_in) <= 0.6:
            cmd = sign * 0.5
        elif 0.6 < abs(cmd_in) <= 0.7:
            cmd = sign * 0.6
        elif 0.7 < abs(cmd_in) <= 0.8:
            cmd = sign * 0.7
        elif 0.8 < abs(cmd_in) <= 0.9:
            cmd = sign * 0.9

        self.send_publish = True
        return cmd

    def process_channel(self, state: dict, label: str):
        """Handles calibration, duty cycle measurement, and velocity publishing for a channel."""
        last_rising = None
        period = None
        cal_count = 0
        dutybuff = 0.0
        init_mean = 0.0
        new_mean = 0.0
        duty = 0.0

        try:
            if state["prev_avg"] is not None:
                init_mean = state["prev_avg"]
                duty = init_mean + state["dead_band"]

            start_time = time.time()
            target_time = start_time + 0.1  # 100 ms window

            while (not state["calibration_done"] or
                   (state["calibration_done"] and time.time() < target_time)):

                GPIO.wait_for_edge(state["pin"], GPIO.RISING)
                now = time.time()
                if last_rising is not None:
                    period = now - last_rising
                last_rising = now

                GPIO.wait_for_edge(state["pin"], GPIO.FALLING)
                fall_time = time.time()

                if period:
                    high_time = fall_time - last_rising
                    duty = (high_time / period) * 100.0
                    if duty > 11.0 or duty < 4.0:
                        continue
                    dutybuff += duty
                else:
                    self.get_logger().info(f"Waiting for {label} stable signal...")

                if not state["calibration_done"]:
                    if cal_count >= 500:
                        self.get_logger().info(f"{label} Calibration Done")
                        state["calibration_done"] = True
                        init_mean = dutybuff / cal_count
                        state["dead_band"] = init_mean - 7.5
                        dutybuff = 0
                    else:
                        cal_count += 1

                if init_mean > 0.0:
                    new_mean = (init_mean + 0.1 * (duty - state["dead_band"])) / 1.1

                if state["calibration_done"]:
                    state["current_cmd"] = state["scale"](new_mean)
                    cmd = self.get_cmd_vel(state["current_cmd"])

                    state["assign"](self.msg, state["prev_saved"])

                    if state["prev_cmd"] == cmd:
                        state["cmd_count"] += 1
                        if state["cmd_count"] >= 5:
                            state["assign"](self.msg, cmd)
                            state["prev_saved"] = cmd
                    else:
                        state["cmd_count"] = 0

                    state["prev_cmd"] = cmd

                    if self.send_publish:
                        self.control_cmd_pub.publish(self.msg)

                init_mean = new_mean

        except Exception as e:
            self.get_logger().error(f"{label} channel error: {e}")
            exit_node()
            rclpy.shutdown()

        state["prev_avg"] = init_mean


def main(args=None):
    rclpy.init(args=args)
    node = ControllerDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterupt:
        node.exit_node()
        pass
    finally:
        node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()


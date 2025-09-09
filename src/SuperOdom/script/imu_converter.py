#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
To RUN

python3 imu_converter.py --ros-args \
  -p input_topic:=/camera/camera/imu \
  -p output_topic:=/converted/imu \
  -p target_frame:=imu_link_rep103 \
  -p orientation_semantics:=world_from_sensor

"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
                        QoSProfile,
                        QoSHistoryPolicy,
                        QoSReliabilityPolicy,
                        QoSDurabilityPolicy,
                    )
from sensor_msgs.msg import Imu


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0 or math.isnan(n):
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_mul(q1, q2):
    # Hamilton product, (x, y, z, w)
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return (x, y, z, w)


class ImuFrameConverter(Node):
    def __init__(self):
        super().__init__("imu_frame_converter")

        # Parameters
        self.input_topic = (
            self.declare_parameter("input_topic", "/imu/raw")
            .get_parameter_value()
            .string_value
        )
        self.output_topic = (
            self.declare_parameter("output_topic", "/imu/rep103")
            .get_parameter_value()
            .string_value
        )
        self.target_frame = (
            self.declare_parameter("target_frame", "imu_link_rep103")
            .get_parameter_value()
            .string_value
        )
        self.orient_sem = (
            self.declare_parameter("orientation_semantics", "world_from_sensor")
            .get_parameter_value()
            .string_value
        )
        self.output_rel = (
            self.declare_parameter("output_reliability", "reliable")  # or "best_effort"
            .get_parameter_value()
            .string_value
        )
        self.sub_depth = (
            self.declare_parameter("subscribe_depth", 5).get_parameter_value().integer_value
        )
        self.pub_depth = (
            self.declare_parameter("publish_depth", 50).get_parameter_value().integer_value
        )

        # Fixed rotation from Sensor frame (S) -> Target REP-103 (T)
        # v_T = R * v_S
        self.R = np.array(
            [
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ],
            dtype=float,
        )

        # Same rotation as unit quaternion q_TS (x,y,z,w)
        self.q_TS = (-0.5, 0.5, -0.5, 0.5)

        # --- QoS profiles ---
        sensor_qos = QoSProfile(
            depth=int(self.sub_depth),
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        pub_qos = QoSProfile(
            depth=int(self.pub_depth),
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=(
                QoSReliabilityPolicy.RELIABLE
                if self.output_rel.lower() == "reliable"
                else QoSReliabilityPolicy.BEST_EFFORT
            ),
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(Imu, self.input_topic, self.cb, sensor_qos)
        self.pub = self.create_publisher(Imu, self.output_topic, pub_qos)

        self.get_logger().info(
            f"IMU converter: '{self.input_topic}' -> '{self.output_topic}' | "
            f"target_frame='{self.target_frame}', orientation_semantics='{self.orient_sem}', "
            f"PUB reliability='{self.output_rel.upper()}', SUB QoS=BEST_EFFORT"
        )

    # ---- Helpers ----
    def _rot_vec(self, v):
        return self.R @ np.array([v.x, v.y, v.z], dtype=float)

    def _rot_cov(self, cov9):
        # Respect sentinel for "unknown": orientation covariance may have cov[0] < 0
        if len(cov9) != 9 or (cov9[0] is not None and float(cov9[0]) < 0.0):
            return cov9
        P = np.array(cov9, dtype=float).reshape(3, 3)
        P2 = self.R @ P @ self.R.T
        return P2.reshape(-1).tolist()

    # ---- Callback ----
    def cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.target_frame

        # --- Orientation ---
        q_in = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # Use if quaternion is non-zero and finite
        valid_quat = (
            not all(abs(c) == 0.0 for c in q_in)
            and all(math.isfinite(c) for c in q_in)
        )
        if valid_quat:
            if self.orient_sem == "world_from_sensor":
                # q_in = q_WS ; want q_out = q_WT = q_WS ⊗ q_ST = q_in ⊗ conj(q_TS)
                q_out = quat_mul(q_in, quat_conj(self.q_TS))
            else:
                # q_in = q_SW ; want q_out = q_TW = q_TS ⊗ q_SW
                q_out = quat_mul(self.q_TS, q_in)
            q_out = quat_normalize(q_out)
            out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q_out
        else:
            out.orientation = msg.orientation  # pass-through if invalid/unknown

        # --- Angular velocity ---
        wx, wy, wz = self._rot_vec(msg.angular_velocity)
        out.angular_velocity.x = float(wx)
        out.angular_velocity.y = float(wy)
        out.angular_velocity.z = float(wz)

        # --- Linear acceleration ---
        ax, ay, az = self._rot_vec(msg.linear_acceleration)
        out.linear_acceleration.x = float(ax)
        out.linear_acceleration.y = float(ay)
        out.linear_acceleration.z = float(az)

        # --- Covariances (rotate 3x3 blocks) ---
        out.orientation_covariance = self._rot_cov(msg.orientation_covariance)
        out.angular_velocity_covariance = self._rot_cov(msg.angular_velocity_covariance)
        out.linear_acceleration_covariance = self._rot_cov(msg.linear_acceleration_covariance)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuFrameConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
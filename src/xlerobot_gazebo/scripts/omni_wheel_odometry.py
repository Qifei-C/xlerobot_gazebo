#!/usr/bin/env python3
"""Compute odometry from omni wheel joint states.

Forward kinematics for a 3-wheel omni drive:

    vx = (2 / n) * Σ  -sin(α_i) * v_i * r
    vy = (2 / n) * Σ   cos(α_i) * v_i * r
    wz = (1 / (n * R)) * Σ  v_i * r

where α_i = wheel_offset + i * 2π/n, v_i is wheel angular velocity,
r is wheel radius, R is robot radius.
"""

import math
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from base_twist_utils import omni_wheel_speeds_to_body_twist
from base_twist_utils import yaw_to_quaternion_components


class OmniWheelOdometry(Node):
    def __init__(self) -> None:
        super().__init__("omni_wheel_odometry")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("wheel_count", 3)
        self.declare_parameter("wheel_offset", 0.0)
        self.declare_parameter("robot_radius", 0.111)
        self.declare_parameter("wheel_radius", 0.0508)
        self.declare_parameter(
            "wheel_names",
            ["wheel_0_joint", "wheel_1_joint", "wheel_2_joint"],
        )

        self.odom_frame = self.get_parameter("odom_frame_id").value
        self.base_frame = self.get_parameter("base_frame_id").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.wheel_count = int(self.get_parameter("wheel_count").value)
        self.wheel_offset = float(self.get_parameter("wheel_offset").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_names: list[str] = list(self.get_parameter("wheel_names").value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        self.lock = threading.Lock()

        odom_topic = self.get_parameter("odom_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, 10,
        )

    def _wheels_to_body(
        self, wheel_velocities: list[float],
    ) -> tuple[float, float, float]:
        return omni_wheel_speeds_to_body_twist(
            wheel_velocities,
            self.wheel_count,
            self.wheel_offset,
            self.robot_radius,
            self.wheel_radius,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        wheel_velocities: list[float] = []
        for name in self.wheel_names:
            try:
                idx = msg.name.index(name)
            except ValueError:
                return
            if idx < len(msg.velocity):
                wheel_velocities.append(msg.velocity[idx])
            else:
                return

        now = self.get_clock().now()

        with self.lock:
            if self.last_time is None:
                self.last_time = now
                return

            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0.0:
                return
            self.last_time = now

            vx_body, vy_body, wz = self._wheels_to_body(wheel_velocities)

            cos_t = math.cos(self.theta)
            sin_t = math.sin(self.theta)
            self.x += (vx_body * cos_t - vy_body * sin_t) * dt
            self.y += (vx_body * sin_t + vy_body * cos_t) * dt
            self.theta += wz * dt

        qx, qy, qz, qw = yaw_to_quaternion_components(self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = OmniWheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

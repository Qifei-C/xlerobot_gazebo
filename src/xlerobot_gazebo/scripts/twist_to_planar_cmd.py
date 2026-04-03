#!/usr/bin/env python3

import threading

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from base_twist_utils import LimitConfig
from base_twist_utils import limit_body_twist


class TwistToPlanarCmd(Node):
    def __init__(self) -> None:
        super().__init__("twist_to_planar_cmd")
        self.declare_parameter("input_topic", "/base/cmd_vel")
        self.declare_parameter(
            "output_topic", "/ideal_base_velocity_controller/commands"
        )
        self.declare_parameter("limited_topic", "/base/cmd_vel_limited")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_planar_speed", 0.3)
        self.declare_parameter("max_angular_speed", 1.57079632679)
        self.declare_parameter("max_planar_acceleration", 1.0)
        self.declare_parameter("max_angular_acceleration", 4.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        limited_topic = self.get_parameter("limited_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_planar_speed = float(self.get_parameter("max_planar_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_planar_acceleration = float(
            self.get_parameter("max_planar_acceleration").value
        )
        self.max_angular_acceleration = float(
            self.get_parameter("max_angular_acceleration").value
        )
        self.limit_config = LimitConfig(
            max_planar_speed=self.max_planar_speed,
            max_angular_speed=self.max_angular_speed,
            max_planar_acceleration=self.max_planar_acceleration,
            max_angular_acceleration=self.max_angular_acceleration,
        )

        self.publisher = self.create_publisher(Float64MultiArray, output_topic, 10)
        self.debug_publisher = self.create_publisher(TwistStamped, limited_topic, 10)
        self.subscription = self.create_subscription(
            TwistStamped,
            input_topic,
            self.handle_twist,
            10,
        )
        self.target_twist = (0.0, 0.0, 0.0)
        self.current_twist = (0.0, 0.0, 0.0)
        now = self.get_clock().now()
        self.last_command_time = now
        self.last_update_time = now
        self.state_lock = threading.Lock()
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.publish_command)

    def handle_twist(self, msg: TwistStamped) -> None:
        with self.state_lock:
            self.target_twist = (
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.angular.z,
            )
            self.last_command_time = self.get_clock().now()

    def publish_command(self) -> None:
        now = self.get_clock().now()
        with self.state_lock:
            dt = (now - self.last_update_time).nanoseconds / 1e9
            if dt <= 0.0:
                return
            self.last_update_time = now

            target_twist = self.target_twist
            stale = (now - self.last_command_time).nanoseconds / 1e9 > self.command_timeout
            if stale:
                target_twist = (0.0, 0.0, 0.0)

            limited_vx, limited_vy, limited_wz = limit_body_twist(
                self.current_twist,
                target_twist,
                dt,
                self.limit_config,
            )
            self.current_twist = (limited_vx, limited_vy, limited_wz)

        command = Float64MultiArray()
        command.data = [
            limited_vx,
            limited_vy,
            limited_wz,
        ]
        self.publisher.publish(command)

        debug_msg = TwistStamped()
        debug_msg.header.stamp = now.to_msg()
        debug_msg.twist.linear.x = limited_vx
        debug_msg.twist.linear.y = limited_vy
        debug_msg.twist.angular.z = limited_wz
        self.debug_publisher.publish(debug_msg)


def main() -> None:
    rclpy.init()
    node = TwistToPlanarCmd()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

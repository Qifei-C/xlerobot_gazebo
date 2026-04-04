#!/usr/bin/env python3
"""Convert body twist to omni wheel velocities with rate limiting.

Inverse kinematics for a 3-wheel omni drive (wheels at 0°, 120°, 240°):

    v_i = (-sin(α_i) * vx + cos(α_i) * vy + R * wz) / r

where α_i = wheel_offset + i * 2π/n.
"""

import threading

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from base_twist_utils import body_twist_to_omni_wheel_speeds
from base_twist_utils import LimitConfig
from base_twist_utils import limit_body_twist


class OmniTwistToWheels(Node):
    def __init__(self) -> None:
        super().__init__("omni_twist_to_wheels")
        self.declare_parameter("input_topic", "/base/cmd_vel")
        self.declare_parameter("output_topic", "/omni_wheel_velocity_controller/commands")
        self.declare_parameter("limited_topic", "/base/cmd_vel_limited")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_planar_speed", 0.3)
        self.declare_parameter("max_angular_speed", 1.57079632679)
        self.declare_parameter("max_planar_acceleration", 0.6)
        self.declare_parameter("max_angular_acceleration", 3.14159265359)
        self.declare_parameter("max_wheel_speed", 10.0)
        self.declare_parameter("wheel_count", 3)
        self.declare_parameter("wheel_offset", 0.0)
        self.declare_parameter("robot_radius", 0.111)
        self.declare_parameter("wheel_radius", 0.0508)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        limited_topic = self.get_parameter("limited_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.wheel_count = int(self.get_parameter("wheel_count").value)
        self.wheel_offset = float(self.get_parameter("wheel_offset").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.limit_config = LimitConfig(
            max_planar_speed=float(self.get_parameter("max_planar_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
            max_planar_acceleration=float(self.get_parameter("max_planar_acceleration").value),
            max_angular_acceleration=float(self.get_parameter("max_angular_acceleration").value),
            max_wheel_speed=float(self.get_parameter("max_wheel_speed").value),
            wheel_count=self.wheel_count,
            wheel_offset=self.wheel_offset,
            robot_radius=self.robot_radius,
            wheel_radius=self.wheel_radius,
        )

        self.wheel_pub = self.create_publisher(Float64MultiArray, output_topic, 10)
        self.debug_pub = self.create_publisher(TwistStamped, limited_topic, 10)
        self.sub = self.create_subscription(TwistStamped, input_topic, self._on_twist, 10)

        self.target_twist = (0.0, 0.0, 0.0)
        self.current_twist = (0.0, 0.0, 0.0)
        now = self.get_clock().now()
        self.last_command_time = now
        self.last_update_time = now
        self.state_lock = threading.Lock()
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._tick)

    def _on_twist(self, msg: TwistStamped) -> None:
        with self.state_lock:
            self.target_twist = (
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.angular.z,
            )
            self.last_command_time = self.get_clock().now()

    def _body_to_wheels(self, vx: float, vy: float, wz: float) -> list[float]:
        return body_twist_to_omni_wheel_speeds(
            vx,
            vy,
            wz,
            self.wheel_count,
            self.wheel_offset,
            self.robot_radius,
            self.wheel_radius,
        )

    def _tick(self) -> None:
        now = self.get_clock().now()
        with self.state_lock:
            dt = (now - self.last_update_time).nanoseconds / 1e9
            if dt <= 0.0:
                return
            self.last_update_time = now

            target = self.target_twist
            if (now - self.last_command_time).nanoseconds / 1e9 > self.command_timeout:
                target = (0.0, 0.0, 0.0)

            vx, vy, wz = limit_body_twist(
                self.current_twist, target, dt, self.limit_config,
            )
            self.current_twist = (vx, vy, wz)

        wheel_cmd = Float64MultiArray()
        wheel_cmd.data = self._body_to_wheels(vx, vy, wz)
        self.wheel_pub.publish(wheel_cmd)

        debug = TwistStamped()
        debug.header.stamp = now.to_msg()
        debug.twist.linear.x = vx
        debug.twist.linear.y = vy
        debug.twist.angular.z = wz
        self.debug_pub.publish(debug)


def main() -> None:
    rclpy.init()
    node = OmniTwistToWheels()
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

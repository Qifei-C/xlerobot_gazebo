#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class TwistStampedRelay(Node):
    def __init__(self) -> None:
        super().__init__("twist_stamped_relay")
        self.declare_parameter("input_topic", "/base/cmd_vel")
        self.declare_parameter("output_topic", "/omni_base_controller/cmd_vel")
        self.declare_parameter("limited_topic", "/base/cmd_vel_limited")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_planar_speed", 0.3)
        self.declare_parameter("max_angular_speed", 1.57079632679)
        self.declare_parameter("max_planar_acceleration", 0.6)
        self.declare_parameter("max_angular_acceleration", 3.14159265359)
        self.declare_parameter("max_wheel_speed", 10.0)
        self.declare_parameter("wheel_count", 3)
        self.declare_parameter("wheel_offset", 0.5235987756)
        self.declare_parameter("robot_radius", 0.111)
        self.declare_parameter("wheel_radius", 0.0508)

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
        self.max_wheel_speed = float(self.get_parameter("max_wheel_speed").value)
        self.wheel_count = int(self.get_parameter("wheel_count").value)
        self.wheel_offset = float(self.get_parameter("wheel_offset").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)

        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
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
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.publish_twist)

    def handle_twist(self, msg: TwistStamped) -> None:
        self.target_twist = self.clamp_body_twist(
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.angular.z,
        )
        self.last_command_time = self.get_clock().now()

    def clamp_body_twist(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        planar_speed = math.hypot(vx, vy)
        if planar_speed > self.max_planar_speed > 0.0:
            scale = self.max_planar_speed / planar_speed
            vx *= scale
            vy *= scale
        if self.max_angular_speed > 0.0:
            wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))
        return vx, vy, wz

    def apply_acceleration_limits(
        self, target_vx: float, target_vy: float, target_wz: float, dt: float
    ) -> tuple[float, float, float]:
        current_vx, current_vy, current_wz = self.current_twist
        delta_vx = target_vx - current_vx
        delta_vy = target_vy - current_vy
        planar_delta = math.hypot(delta_vx, delta_vy)
        max_planar_delta = self.max_planar_acceleration * dt
        if planar_delta > max_planar_delta > 0.0:
            scale = max_planar_delta / planar_delta
            delta_vx *= scale
            delta_vy *= scale

        delta_wz = target_wz - current_wz
        max_wz_delta = self.max_angular_acceleration * dt
        if abs(delta_wz) > max_wz_delta > 0.0:
            delta_wz = math.copysign(max_wz_delta, delta_wz)

        return current_vx + delta_vx, current_vy + delta_vy, current_wz + delta_wz

    def apply_wheel_speed_limit(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        if self.max_wheel_speed <= 0.0 or self.wheel_count < 1 or self.wheel_radius <= 0.0:
            return vx, vy, wz

        peak_speed = 0.0
        for index in range(self.wheel_count):
            alpha = self.wheel_offset + index * (2.0 * math.pi / self.wheel_count)
            wheel_speed = (
                -math.sin(alpha) * vx
                + math.cos(alpha) * vy
                - self.robot_radius * wz
            ) / self.wheel_radius
            peak_speed = max(peak_speed, abs(wheel_speed))

        if peak_speed > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak_speed
            return vx * scale, vy * scale, wz * scale
        return vx, vy, wz

    def publish_twist(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_update_time = now

        target_vx, target_vy, target_wz = self.target_twist
        stale = (now - self.last_command_time).nanoseconds / 1e9 > self.command_timeout
        if stale:
            target_vx, target_vy, target_wz = 0.0, 0.0, 0.0

        limited_vx, limited_vy, limited_wz = self.apply_acceleration_limits(
            target_vx,
            target_vy,
            target_wz,
            dt,
        )
        limited_vx, limited_vy, limited_wz = self.apply_wheel_speed_limit(
            limited_vx,
            limited_vy,
            limited_wz,
        )
        limited_vx, limited_vy, limited_wz = self.clamp_body_twist(
            limited_vx,
            limited_vy,
            limited_wz,
        )
        self.current_twist = (limited_vx, limited_vy, limited_wz)

        command = TwistStamped()
        command.header.stamp = now.to_msg()
        command.twist.linear.x = limited_vx
        command.twist.linear.y = limited_vy
        command.twist.angular.z = limited_wz
        self.publisher.publish(command)
        self.debug_publisher.publish(command)


def main() -> None:
    rclpy.init()
    node = TwistStampedRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

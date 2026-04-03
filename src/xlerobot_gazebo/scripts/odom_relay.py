#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_relay")
        self.declare_parameter("input_topic", "/omni_base_controller/odom")
        self.declare_parameter("output_topic", "/odom")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.handle_odom,
            10,
        )

    def handle_odom(self, msg: Odometry) -> None:
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

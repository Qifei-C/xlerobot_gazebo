#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    msg = Quaternion()
    msg.w = math.cos(half_yaw)
    msg.z = math.sin(half_yaw)
    return msg


class PlanarJointOdometry(Node):
    def __init__(self) -> None:
        super().__init__("planar_joint_odometry")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)

        self.joint_states_topic = self.get_parameter("joint_states_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.handle_joint_states,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

    def handle_joint_states(self, msg: JointState) -> None:
        try:
            x_index = msg.name.index("root_x_axis_joint")
            y_index = msg.name.index("root_y_axis_joint")
            yaw_index = msg.name.index("root_z_rotation_joint")
        except ValueError:
            return

        if len(msg.position) <= max(x_index, y_index, yaw_index):
            return

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        x = msg.position[x_index]
        y = msg.position[y_index]
        yaw = msg.position[yaw_index]
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)

        if len(msg.velocity) > max(x_index, y_index, yaw_index):
            odom.twist.twist.linear.x = msg.velocity[x_index]
            odom.twist.twist.linear.y = msg.velocity[y_index]
            odom.twist.twist.angular.z = msg.velocity[yaw_index]

        self.publisher.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = PlanarJointOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

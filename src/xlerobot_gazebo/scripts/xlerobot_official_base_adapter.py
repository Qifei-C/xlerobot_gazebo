#!/usr/bin/env python3

import json
import threading

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

from base_twist_utils import official_base_action_to_twist
from base_twist_utils import odom_twist_to_official_base_observation
from base_twist_utils import official_position_to_radians
from base_twist_utils import radians_to_official_position


POSITION_SPECS = {
    "left_arm_shoulder_pan.pos": {
        "joint": "Rotation",
        "group": "left_arm",
        "lower": -2.1,
        "upper": 2.1,
        "zero_to_hundred": False,
    },
    "left_arm_shoulder_lift.pos": {
        "joint": "Pitch",
        "group": "left_arm",
        "lower": -0.1,
        "upper": 3.45,
        "zero_to_hundred": False,
    },
    "left_arm_elbow_flex.pos": {
        "joint": "Elbow",
        "group": "left_arm",
        "lower": -0.2,
        "upper": 3.14159,
        "zero_to_hundred": False,
    },
    "left_arm_wrist_flex.pos": {
        "joint": "Wrist_Pitch",
        "group": "left_arm",
        "lower": -1.8,
        "upper": 1.8,
        "zero_to_hundred": False,
    },
    "left_arm_wrist_roll.pos": {
        "joint": "Wrist_Roll",
        "group": "left_arm",
        "lower": -3.14159,
        "upper": 3.14159,
        "zero_to_hundred": False,
    },
    "left_arm_gripper.pos": {
        "joint": "Jaw",
        "group": "left_gripper",
        "lower": 0.0,
        "upper": 1.7,
        "zero_to_hundred": True,
    },
    "right_arm_shoulder_pan.pos": {
        "joint": "Rotation_2",
        "group": "right_arm",
        "lower": -2.1,
        "upper": 2.1,
        "zero_to_hundred": False,
    },
    "right_arm_shoulder_lift.pos": {
        "joint": "Pitch_2",
        "group": "right_arm",
        "lower": -0.1,
        "upper": 3.45,
        "zero_to_hundred": False,
    },
    "right_arm_elbow_flex.pos": {
        "joint": "Elbow_2",
        "group": "right_arm",
        "lower": -0.2,
        "upper": 3.14159,
        "zero_to_hundred": False,
    },
    "right_arm_wrist_flex.pos": {
        "joint": "Wrist_Pitch_2",
        "group": "right_arm",
        "lower": -1.8,
        "upper": 1.8,
        "zero_to_hundred": False,
    },
    "right_arm_wrist_roll.pos": {
        "joint": "Wrist_Roll_2",
        "group": "right_arm",
        "lower": -3.14159,
        "upper": 3.14159,
        "zero_to_hundred": False,
    },
    "right_arm_gripper.pos": {
        "joint": "Jaw_2",
        "group": "right_gripper",
        "lower": 0.0,
        "upper": 1.7,
        "zero_to_hundred": True,
    },
    "head_motor_1.pos": {
        "joint": "head_pan_joint",
        "group": "head",
        "lower": -1.57,
        "upper": 1.57,
        "zero_to_hundred": False,
    },
    "head_motor_2.pos": {
        "joint": "head_tilt_joint",
        "group": "head",
        "lower": -0.76,
        "upper": 1.45,
        "zero_to_hundred": False,
    },
}

GROUP_ORDERS = {
    "left_arm": [
        "left_arm_shoulder_pan.pos",
        "left_arm_shoulder_lift.pos",
        "left_arm_elbow_flex.pos",
        "left_arm_wrist_flex.pos",
        "left_arm_wrist_roll.pos",
    ],
    "left_gripper": [
        "left_arm_gripper.pos",
    ],
    "right_arm": [
        "right_arm_shoulder_pan.pos",
        "right_arm_shoulder_lift.pos",
        "right_arm_elbow_flex.pos",
        "right_arm_wrist_flex.pos",
        "right_arm_wrist_roll.pos",
    ],
    "right_gripper": [
        "right_arm_gripper.pos",
    ],
    "head": [
        "head_motor_1.pos",
        "head_motor_2.pos",
    ],
}


class XLerobotOfficialBaseAdapter(Node):
    def __init__(self) -> None:
        super().__init__("xlerobot_official_base_adapter")
        self.declare_parameter("input_action_topic", "/xlerobot/action_dict")
        self.declare_parameter("output_cmd_topic", "/base/cmd_vel")
        self.declare_parameter("input_odom_topic", "/odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("output_observation_topic", "/xlerobot/observation_dict")
        self.declare_parameter("left_arm_command_topic", "/left_arm_position_controller/commands")
        self.declare_parameter("left_gripper_command_topic", "/left_gripper_position_controller/commands")
        self.declare_parameter("right_arm_command_topic", "/right_arm_position_controller/commands")
        self.declare_parameter("right_gripper_command_topic", "/right_gripper_position_controller/commands")
        self.declare_parameter("head_command_topic", "/head_position_controller/commands")
        self.declare_parameter("command_frame_id", "base_link")
        self.declare_parameter("use_degrees", False)
        self.declare_parameter("publish_rate_hz", 20.0)

        input_action_topic = self.get_parameter("input_action_topic").value
        output_cmd_topic = self.get_parameter("output_cmd_topic").value
        input_odom_topic = self.get_parameter("input_odom_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        output_observation_topic = self.get_parameter("output_observation_topic").value
        left_arm_command_topic = self.get_parameter("left_arm_command_topic").value
        left_gripper_command_topic = self.get_parameter("left_gripper_command_topic").value
        right_arm_command_topic = self.get_parameter("right_arm_command_topic").value
        right_gripper_command_topic = self.get_parameter("right_gripper_command_topic").value
        head_command_topic = self.get_parameter("head_command_topic").value
        self.command_frame_id = self.get_parameter("command_frame_id").value
        self.use_degrees = bool(self.get_parameter("use_degrees").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.cmd_publisher = self.create_publisher(TwistStamped, output_cmd_topic, 10)
        self.left_arm_publisher = self.create_publisher(Float64MultiArray, left_arm_command_topic, 10)
        self.left_gripper_publisher = self.create_publisher(
            Float64MultiArray,
            left_gripper_command_topic,
            10,
        )
        self.right_arm_publisher = self.create_publisher(Float64MultiArray, right_arm_command_topic, 10)
        self.right_gripper_publisher = self.create_publisher(
            Float64MultiArray,
            right_gripper_command_topic,
            10,
        )
        self.head_publisher = self.create_publisher(Float64MultiArray, head_command_topic, 10)
        self.observation_publisher = self.create_publisher(String, output_observation_topic, 10)
        self.action_subscription = self.create_subscription(
            String,
            input_action_topic,
            self.handle_action,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            input_odom_topic,
            self.handle_odom,
            10,
        )
        self.joint_states_subscription = self.create_subscription(
            JointState,
            joint_states_topic,
            self.handle_joint_states,
            10,
        )

        self.state_lock = threading.Lock()
        self.latest_joint_positions: dict[str, float] = {}
        self.latest_base_twist = (0.0, 0.0, 0.0)
        self.last_group_commands = {
            "left_arm": [0.0] * len(GROUP_ORDERS["left_arm"]),
            "right_arm": [0.0] * len(GROUP_ORDERS["right_arm"]),
            "head": [0.0] * len(GROUP_ORDERS["head"]),
        }
        self.observation_timer = self.create_timer(
            1.0 / max(publish_rate_hz, 1.0),
            self.publish_observation,
        )

    def handle_action(self, msg: String) -> None:
        try:
            action = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Failed to parse action_dict JSON: {exc}")
            return

        if not isinstance(action, dict):
            self.get_logger().warning("Ignoring action_dict payload because it is not a JSON object.")
            return

        with self.state_lock:
            current_positions = dict(self.latest_joint_positions)

        self.publish_base_command(action)
        self.publish_group_command(
            "left_arm",
            action,
            current_positions,
            self.left_arm_publisher,
        )
        self.publish_group_command(
            "left_gripper",
            action,
            current_positions,
            self.left_gripper_publisher,
        )
        self.publish_group_command(
            "right_arm",
            action,
            current_positions,
            self.right_arm_publisher,
        )
        self.publish_group_command(
            "right_gripper",
            action,
            current_positions,
            self.right_gripper_publisher,
        )
        self.publish_group_command(
            "head",
            action,
            current_positions,
            self.head_publisher,
        )

    def handle_odom(self, msg: Odometry) -> None:
        with self.state_lock:
            self.latest_base_twist = (
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z,
            )

    def handle_joint_states(self, msg: JointState) -> None:
        with self.state_lock:
            for name, position in zip(msg.name, msg.position):
                self.latest_joint_positions[name] = position

    def publish_base_command(self, action: dict[str, float]) -> None:
        vx, vy, wz = official_base_action_to_twist(action)
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self.command_frame_id
        command.twist.linear.x = vx
        command.twist.linear.y = vy
        command.twist.angular.z = wz
        self.cmd_publisher.publish(command)

    def publish_group_command(
        self,
        group: str,
        action: dict[str, float],
        current_positions: dict[str, float],
        publisher,
    ) -> None:
        group_keys = GROUP_ORDERS[group]
        if not any(key in action for key in group_keys):
            return

        command_data = []
        for index, key in enumerate(group_keys):
            spec = POSITION_SPECS[key]
            if key in action:
                position = official_position_to_radians(
                    action[key],
                    spec["lower"],
                    spec["upper"],
                    self.use_degrees,
                    spec["zero_to_hundred"],
                )
            elif spec["joint"] in current_positions:
                position = current_positions[spec["joint"]]
            else:
                position = self.last_group_commands[group][index]

            position = max(spec["lower"], min(spec["upper"], position))
            command_data.append(position)

        self.last_group_commands[group] = command_data
        publisher.publish(Float64MultiArray(data=command_data))

    def publish_observation(self) -> None:
        with self.state_lock:
            joint_positions = dict(self.latest_joint_positions)
            base_twist = self.latest_base_twist

        observation = {}
        for key, spec in POSITION_SPECS.items():
            joint_position = joint_positions.get(spec["joint"])
            if joint_position is None:
                return
            observation[key] = radians_to_official_position(
                joint_position,
                spec["lower"],
                spec["upper"],
                self.use_degrees,
                spec["zero_to_hundred"],
            )

        observation.update(
            odom_twist_to_official_base_observation(
                base_twist[0],
                base_twist[1],
                base_twist[2],
            )
        )
        self.observation_publisher.publish(
            String(data=json.dumps(observation, sort_keys=True))
        )


def main() -> None:
    rclpy.init()
    node = XLerobotOfficialBaseAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

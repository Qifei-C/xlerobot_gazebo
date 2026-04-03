#!/usr/bin/env python3
"""WASD keyboard teleop for XLeRobot base.

Controls:
    W / S  — forward / backward
    A / D  — strafe left / right
    Q / E  — rotate left / right
    Space  — stop
    Esc    — quit
"""

import sys
import termios
import tty
from select import select

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

KEYMAP: dict[str, tuple[float, float, float]] = {
    # key: (linear_x, linear_y, angular_z)
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
    " ": (0.0, 0.0, 0.0),
}

HELP = """\
WASD Keyboard Teleop
---------------------
  W/S : forward / backward
  A/D : strafe left / right
  Q/E : rotate left / right
Space : stop
  Esc : quit
"""


def read_key(timeout: float = 0.1) -> str | None:
    """Return a single keypress if available within *timeout* seconds."""
    ready, _, _ = select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__(
            "keyboard_teleop",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                ),
            ],
        )
        self.declare_parameter("cmd_topic", "/base/cmd_vel")
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        topic = self.get_parameter("cmd_topic").value
        self.linear_speed: float = self.get_parameter("linear_speed").value
        self.angular_speed: float = self.get_parameter("angular_speed").value
        rate_hz: float = self.get_parameter("publish_rate_hz").value

        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def _tick(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.angular.z = self.wz
        self.pub.publish(msg)

    def handle_key(self, key: str) -> bool:
        """Process a keypress. Returns False when the user wants to quit."""
        if key == "\x1b":
            return False
        lower = key.lower()
        if lower in KEYMAP:
            lx, ly, az = KEYMAP[lower]
            self.vx = lx * self.linear_speed
            self.vy = ly * self.linear_speed
            self.wz = az * self.angular_speed
        return True


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(HELP)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            key = read_key(timeout=0.05)
            if key is not None and not node.handle_key(key):
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.vx = node.vy = node.wz = 0.0
        node._tick()  # publish one stop message
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

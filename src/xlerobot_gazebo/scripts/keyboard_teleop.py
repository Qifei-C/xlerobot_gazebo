#!/usr/bin/env python3
"""WASD keyboard teleop for XLeRobot base.

Controls:
    W / S  — forward / backward  (vx)
    A / D  — strafe left / right (vy)
    Q / E  — rotate left / right (wz)
    Space  — stop all axes
    Esc    — quit

Keys modify only their own axis, so pressing W then A
gives forward + strafe (diagonal motion).
"""

import sys
import termios
import time
import tty
from select import select

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

# Each key maps to (axis, sign).  "stop" resets all axes.
KEYMAP: dict[str, tuple[str, float]] = {
    "w": ("vx", 1.0),
    "s": ("vx", -1.0),
    "a": ("vy", 1.0),
    "d": ("vy", -1.0),
    "q": ("wz", -1.0),
    "e": ("wz", 1.0),
    " ": ("stop", 0.0),
}

HELP = """\
WASD Keyboard Teleop
---------------------
  W/S : forward / backward
  A/D : strafe left / right
  Q/E : rotate left / right
Space : stop all axes
  Esc : quit

Keys are independent — press multiple for combined motion.
"""


def read_key(timeout: float = 0.1) -> str | None:
    """Return a single keypress if available within *timeout* seconds."""
    ready, _, _ = select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_teleop")
        self.declare_parameter("cmd_topic", "/base/cmd_vel")
        self.declare_parameter("speed", 0.2)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout", 0.5)

        topic = self.get_parameter("cmd_topic").value
        self.speed: float = self.get_parameter("speed").value
        rate_hz: float = self.get_parameter("publish_rate_hz").value
        self.command_timeout: float = self.get_parameter("command_timeout").value

        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_key_time = 0.0

    def _print_status(self) -> None:
        sys.stdout.write(
            f"\rCommand: vx={self.vx:+.2f} vy={self.vy:+.2f} wz={self.wz:+.2f}      "
        )
        sys.stdout.flush()

    def _tick(self) -> None:
        if (
            self.command_timeout > 0.0
            and (self.vx != 0.0 or self.vy != 0.0 or self.wz != 0.0)
            and (time.monotonic() - self.last_key_time) > self.command_timeout
        ):
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            self._print_status()

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
        if lower not in KEYMAP:
            return True
        axis, sign = KEYMAP[lower]
        if axis == "stop":
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
        elif axis == "vx":
            self.vx = sign * self.speed
        elif axis == "vy":
            self.vy = sign * self.speed
        elif axis == "wz":
            self.wz = sign * self.speed
        self.last_key_time = time.monotonic()
        self._tick()
        self._print_status()
        return True


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(HELP)
        node._print_status()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            key = read_key(timeout=0.05)
            if key is not None and not node.handle_key(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.vx = 0.0
        node.vy = 0.0
        node.wz = 0.0
        if rclpy.ok():
            node._tick()
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

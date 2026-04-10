"""
keyboard_teleop_node.py — Keyboard teleoperation for manual boat control.

=========================================================
KEY MAPPING
=========================================================
  Z / Arrow Up    : forward  (linear.x=1.5, angular.z=0.0)
  S / Arrow Down  : backward (linear.x=-1.0, angular.z=0.0)
  Q / Arrow Left  : turn left  (linear.x=0.3, angular.z=1.0)
  D / Arrow Right : turn right (linear.x=0.3, angular.z=-1.0)
  Space           : stop (linear.x=0.0, angular.z=0.0)
  M               : toggle MANUAL/AUTO mode
  Ctrl+C          : exit cleanly

=========================================================
TOPICS
=========================================================
Publie :
  /cmd_vel      (geometry_msgs/Twist)  — commandes (MANUAL mode only)
  /manual_mode  (std_msgs/Bool)        — True=MANUAL, False=AUTO

Souscrit :
  /current_mode (std_msgs/String)      — mode retourné par le navigateur

=========================================================
ARCHITECTURE
=========================================================
ROS2 spin runs in a daemon thread so the main thread can block
on raw keyboard input (tty/termios) without freezing the node.
"""

import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

# Arrow key escape sequences sent by most terminals
_ARROW_UP = '\x1b[A'
_ARROW_DOWN = '\x1b[B'
_ARROW_LEFT = '\x1b[D'
_ARROW_RIGHT = '\x1b[C'

# (linear.x, angular.z) for each key
KEY_COMMANDS = {
    'z': (1.5, 0.0),
    's': (-1.0, 0.0),
    'q': (0.3, 1.0),
    'd': (0.3, -1.0),
    ' ': (0.0, 0.0),
    _ARROW_UP: (1.5, 0.0),
    _ARROW_DOWN: (-1.0, 0.0),
    _ARROW_LEFT: (0.3, 1.0),
    _ARROW_RIGHT: (0.3, -1.0),
}

_HELP = (
    "Keys: Z/↑=forward  S/↓=backward  Q/←=turn left  D/→=turn right  "
    "Space=stop  M=toggle mode  Ctrl+C=quit"
)


def _read_key(fd):
    """
    Read one logical key from a raw file descriptor.

    Returns a string: a single character, an arrow-key escape sequence
    such as '\\x1b[A', or '\\x03' for Ctrl+C.
    """
    ch = os.read(fd, 1).decode('utf-8', errors='replace')
    if ch == '\x1b':
        # Check for the rest of a CSI escape sequence within 50 ms
        ready = select.select([fd], [], [], 0.05)[0]
        if ready:
            rest = os.read(fd, 2).decode('utf-8', errors='replace')
            return '\x1b' + rest
    return ch


class KeyboardTeleopNode(Node):
    """ROS2 node that publishes cmd_vel and manual_mode from keyboard input."""

    def __init__(self):
        super().__init__('keyboard_teleop')

        # Start in MANUAL mode so the node immediately takes control
        self.manual_mode = True

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_manual_mode = self.create_publisher(Bool, '/manual_mode', 10)

        self.sub_current_mode = self.create_subscription(
            String,
            '/current_mode',
            self._current_mode_callback,
            10,
        )

        # Publish initial mode so the navigator knows straight away
        self._publish_mode(self.manual_mode)

    def _current_mode_callback(self, msg):
        """Keep local state in sync with the navigator's published mode."""
        self.manual_mode = (msg.data == 'MANUAL')

    def _publish_mode(self, manual: bool):
        msg = Bool()
        msg.data = manual
        self.pub_manual_mode.publish(msg)

    def toggle_mode(self):
        """Flip MANUAL/AUTO and publish the new state."""
        self.manual_mode = not self.manual_mode
        self._publish_mode(self.manual_mode)

    def send_cmd_vel(self, linear_x: float, angular_z: float):
        """Publish a Twist command on /cmd_vel."""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.pub_cmd_vel.publish(cmd)


def _keyboard_loop(node: KeyboardTeleopNode):
    """
    Blocking keyboard loop — runs in the main thread.

    Reads raw key input and either sends velocity commands (MANUAL)
    or toggles the mode (M key).  All other keypresses are ignored
    in AUTO mode.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    print('\r\nKeyboard teleop started.')
    print(_HELP)
    print()

    try:
        tty.setraw(fd)

        while rclpy.ok():
            key = _read_key(fd)

            if key == '\x03':  # Ctrl+C
                break

            # M key: toggle mode regardless of current mode
            if key.lower() == 'm':
                node.toggle_mode()
                mode_str = 'MANUAL' if node.manual_mode else 'AUTO'
                print(f'\r[{mode_str}]   Mode toggled                                   ')
                continue

            # Look up the command (try lowercase for letter keys)
            cmd = KEY_COMMANDS.get(key) or KEY_COMMANDS.get(key.lower())

            if cmd is None:
                continue  # Unknown key — ignore

            linear_x, angular_z = cmd

            if node.manual_mode:
                node.send_cmd_vel(linear_x, angular_z)
                print(
                    f'\r[MANUAL] linear={linear_x:.1f} angular={angular_z:.1f}          '
                )
            else:
                # In AUTO mode, movement keys are ignored
                print(
                    f'\r[AUTO]   Navigator active — press M to switch to MANUAL    '
                )

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    # Spin in a daemon thread so the main thread can block on keyboard I/O
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        _keyboard_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

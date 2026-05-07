"""
sim_bridge_node.py — Glue between the 2D simulator and the Njord PID chain.

Two jobs:
  1. Synthesise /imu/data from /sim2d/odom so pid_controller can close the
     yaw-rate loop (it needs angular_velocity.z).
  2. Forward /control/effort (pid_controller output) to /cmd_vel so the
     simulator receives the final control signal instead of raw nav commands.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class SimBridgeNode(Node):

    def __init__(self):
        super().__init__('sim_bridge')

        # Synthesise IMU from odometry angular velocity
        self._pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.create_subscription(Odometry, '/sim2d/odom', self._odom_cb, 10)

        # Forward PID effort back to the simulator as /cmd_vel
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/control/effort', self._effort_cb, 10)

        self.get_logger().info('SimBridge ready — /imu/data + /control/effort → /cmd_vel')

    def _odom_cb(self, msg):
        imu = Imu()
        imu.header = msg.header
        imu.angular_velocity.z = msg.twist.twist.angular.z
        imu.orientation_covariance[0] = -1.0   # orientation not provided
        self._pub_imu.publish(imu)

    def _effort_cb(self, msg):
        self._pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

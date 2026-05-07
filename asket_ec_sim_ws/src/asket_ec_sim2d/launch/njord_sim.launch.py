"""
njord_sim.launch.py — 2D simulator with the Njord PID control chain.

The existing waypoint_navigator_node handles path planning (pure pursuit
through gate centres). The Njord chain adds closed-loop PID control on top:

  waypoint_navigator  →  /nav/cmd_vel
  nav_to_pid          reads /nav/cmd_vel → /control/setpoint  (clamps values)
  pid_controller      reads /control/setpoint + /imu/data → /control/effort
  sim_bridge          /control/effort → /cmd_vel  (back to the simulator)
                      /sim2d/odom    → /imu/data  (yaw rate for PID feedback)

Run:
  source /opt/ros/jazzy/setup.bash
  ros2 launch asket_ec_sim2d njord_sim.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg        = get_package_share_directory('asket_ec_sim2d')
    rviz_cfg   = os.path.join(pkg, 'config', 'sim2d.rviz')
    buoys_file = os.path.join(pkg, 'config', 'buoys.yaml')
    wp_file    = os.path.join(pkg, 'config', 'waypoints.yaml')

    return LaunchDescription([

        # ── Simulator ────────────────────────────────────────────────
        Node(package='asket_ec_sim2d', executable='simulator_node',
             name='sim2d_simulator', output='screen'),

        Node(package='asket_ec_sim2d', executable='buoy_simulator_node',
             name='buoy_simulator', output='screen',
             parameters=[{'buoys_file': buoys_file}]),

        # ── Path planner (pure pursuit, unchanged) ───────────────────
        # Remapped: publishes /nav/cmd_vel instead of /cmd_vel so the
        # Njord PID chain sits between the planner and the simulator.
        Node(package='asket_ec_sim2d', executable='waypoint_navigator_node',
             name='waypoint_navigator', output='screen',
             parameters=[{'waypoints_file': wp_file}],
             remappings=[('/cmd_vel', '/nav/cmd_vel')]),

        # ── Njord PID chain ──────────────────────────────────────────
        # nav_to_pid: reads /nav/cmd_vel (remapped), clamps, → /control/setpoint
        Node(package='asket_ec_sim2d', executable='nav_to_pid',
             name='nav_to_pid', output='screen',
             remappings=[('/cmd_vel', '/nav/cmd_vel')]),

        # pid_controller: reads /control/setpoint + /imu/data → /control/effort
        Node(package='asket_ec_sim2d', executable='pid_controller',
             name='pid_controller', output='screen'),

        # sim_bridge: synthesises /imu/data and forwards effort → /cmd_vel
        Node(package='asket_ec_sim2d', executable='sim_bridge_node',
             name='sim_bridge', output='screen'),

        # ── Visualisation ────────────────────────────────────────────
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', arguments=['-d', rviz_cfg]),
    ])

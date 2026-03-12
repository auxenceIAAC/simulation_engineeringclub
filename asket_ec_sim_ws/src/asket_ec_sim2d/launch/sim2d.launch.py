"""
sim2d.launch.py — Lance le simulateur 2D Python + RViz2

=========================================================
POURQUOI CE LAUNCH FILE ?
=========================================================
Ce launch file remplace l'ancienne stack Gazebo par un simulateur
2D pur Python. Avantages :
  - Démarre en < 1 seconde (vs 10-30s pour Gazebo)
  - Aucune dépendance OpenGL / GPU
  - Stable sur WSL2, VMs, CI/CD
  - Physique déterministe et reproductible

Commande :
  ros2 launch asket_ec_sim2d sim2d.launch.py

Pour tester sans RViz2 :
  ros2 run asket_ec_sim2d simulator_node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Lance simulator_node + waypoint_navigator_node + RViz2."""

    pkg_sim2d = get_package_share_directory('asket_ec_sim2d')
    rviz_config = os.path.join(pkg_sim2d, 'config', 'sim2d.rviz')
    waypoints_file = os.path.join(pkg_sim2d, 'config', 'waypoints.yaml')

    # use_sim_time=false : le simulateur Python utilise l'horloge système.
    # Pas de /clock publié → pas de "jump back in time" dans RViz2.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Utiliser le temps de simulation (false = horloge système)'
    )

    # Nœud simulateur 2D — cœur de la simulation
    simulator_node = Node(
        package='asket_ec_sim2d',
        executable='simulator_node',
        name='sim2d_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Nœud navigateur par waypoints
    # Charge waypoints.yaml, trie par distance, publie /cmd_vel
    waypoint_navigator_node = Node(
        package='asket_ec_sim2d',
        executable='waypoint_navigator_node',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'waypoints_file': waypoints_file,
        }]
    )

    # RViz2 avec la config dédiée sim2d
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        simulator_node,
        waypoint_navigator_node,
        rviz_node,
    ])

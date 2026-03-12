"""
rviz.launch.py — Lance RViz2 avec la configuration Beatnaut

=========================================================
QU'EST-CE QUE RVIZ2 ?
=========================================================
RViz2 (ROS Visualization 2) est l'outil de visualisation standard de ROS2.
Contrairement à Gazebo qui simule la physique, RViz2 affiche uniquement
les données publiées sur les topics ROS2 :

- Modèle 3D du bateau    → depuis /robot_description (URDF/SDF)
- Axes TF                → depuis /tf et /tf_static
- Trajectoire GPS        → depuis /asket_ec/path (nav_msgs/Path)
- Orientation IMU        → depuis /asket_ec/imu (sensor_msgs/Imu)

Avantage pour WSL2 : RViz2 utilise le rendu standard OpenGL, beaucoup
plus léger que le moteur de rendu 3D de Gazebo (Ogre).

Commande d'utilisation :
  ros2 launch asket_ec_gazebo rviz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Lance RViz2 avec la configuration beatnaut.rviz."""

    pkg_asket_ec_gazebo = get_package_share_directory('asket_ec_gazebo')

    # Chemin vers le fichier de configuration RViz2
    rviz_config_file = os.path.join(
        pkg_asket_ec_gazebo, 'config', 'beatnaut.rviz'
    )

    # Argument : utiliser le temps de simulation
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Synchroniser RViz2 sur le temps de simulation Gazebo'
    )

    # =========================================================
    # COMPOSANT : RViz2
    # =========================================================
    # Node() démarre RViz2 comme un nœud ROS2 standard.
    #
    # arguments=['-d', config] : charger automatiquement notre config
    # (évite de devoir reconfigurer manuellement les displays à chaque démarrage)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node,
    ])

"""
control.launch.py — Lance uniquement le nœud de contrôle (sans Gazebo)

Ce launch file est utile pour :
- Tester le contrôleur séparément
- Relancer le contrôleur sans redémarrer toute la simulation
- Débogage du comportement du contrôleur

Usage :
  ros2 launch beatnaut_control control.launch.py
  ros2 launch beatnaut_control control.launch.py max_thrust_rpm:=200.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Lance uniquement le nœud de contrôle différentiel."""

    # Arguments configurables depuis la ligne de commande
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Utiliser le temps de simulation'
    )

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.50',
        description='Écartement entre les deux propulseurs en mètres'
    )

    max_thrust_arg = DeclareLaunchArgument(
        'max_thrust_rpm',
        default_value='300.0',
        description='Vitesse angulaire max des hélices en rad/s'
    )

    # Nœud de contrôle différentiel
    control_node = Node(
        package='beatnaut_control',
        executable='differential_drive_node',
        name='differential_drive_controller',
        output='screen',
        # remappings : permet de rediriger les topics si nécessaire
        # Exemple : [(from, to)] pour renommer un topic
        remappings=[],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'max_thrust_rpm': LaunchConfiguration('max_thrust_rpm'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        wheel_separation_arg,
        max_thrust_arg,
        control_node,
    ])

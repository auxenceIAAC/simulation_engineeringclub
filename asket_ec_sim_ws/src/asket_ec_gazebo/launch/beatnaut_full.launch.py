"""
beatnaut_full.launch.py — Stack complète : Gazebo headless + RViz2 + contrôle

=========================================================
POURQUOI CE FICHIER ?
=========================================================
Ce launch file est optimisé pour les machines avec GPU limité (WSL2, Intel UHD).
Il démarre :

1. Gazebo en mode HEADLESS (-s, sans fenêtre graphique)
   → La physique tourne en arrière-plan sans rendu 3D lourd
   → Le bateau est simulé, les capteurs publient leurs données

2. RViz2 (visualisation légère)
   → Affiche le modèle du bateau, les TF, la trajectoire, l'IMU
   → Beaucoup moins gourmand en GPU que le rendu Gazebo

3. Le contrôleur de propulsion (asket_ec_control)
   → Traduit les commandes haut-niveau en vitesses d'hélices

Commande d'utilisation :
  ros2 launch asket_ec_gazebo beatnaut_full.launch.py

Arguments disponibles :
  use_sim_time:=false  → Utiliser le temps système (déconseillé)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Lance la stack complète Gazebo headless + RViz2 + contrôleur."""

    pkg_asket_ec_gazebo = get_package_share_directory('asket_ec_gazebo')
    pkg_asket_ec_control = get_package_share_directory('asket_ec_control')

    # =========================================================
    # ARGUMENT : temps de simulation
    # =========================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Synchroniser tous les nœuds sur le temps de simulation Gazebo'
    )

    # =========================================================
    # COMPOSANT 1 : Gazebo en mode headless
    # =========================================================
    # On force headless:=true ici — c'est le but de ce launch file.
    # Le moteur physique tourne, mais aucune fenêtre Gazebo ne s'ouvre.
    # Résultat : beaucoup moins de charge GPU, idéal pour Intel UHD / WSL2.
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_asket_ec_gazebo, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'headless': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # =========================================================
    # COMPOSANT 2 : RViz2
    # =========================================================
    # On attend 2 secondes que robot_state_publisher publie /robot_description
    # avant de démarrer RViz2 (sinon RobotModel affiche une erreur temporaire).
    rviz_launch = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_asket_ec_gazebo, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()
        )]
    )

    # =========================================================
    # COMPOSANT 3 : Contrôleur de propulsion
    # =========================================================
    # On attend 4 secondes (Gazebo headless + spawn robot).
    control_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='asket_ec_control',
                executable='differential_drive_node',
                name='differential_drive_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'wheel_separation': 0.50,
                    'max_thrust_rpm': 300.0,
                }]
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        simulation_launch,   # Gazebo headless (démarre immédiatement)
        rviz_launch,         # RViz2 (après 2s)
        control_node,        # Contrôleur (après 4s)
    ])

"""
beatnaut_full.launch.py — Launch file principal qui démarre TOUT

=========================================================
POURQUOI CE FICHIER ?
=========================================================
Ce launch file est le "point d'entrée" de toute la simulation.
Il lance en une seule commande :
1. La simulation Gazebo (simulation.launch.py)
2. Le nœud de contrôle différentiel (beatnaut_control)

Commande d'utilisation :
  ros2 launch beatnaut_gazebo beatnaut_full.launch.py

Arguments disponibles :
  headless:=true       → Gazebo sans interface graphique
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
    """Lance l'intégralité de la stack de simulation Beatnaut."""

    # Chemins vers les packages
    pkg_beatnaut_gazebo = get_package_share_directory('beatnaut_gazebo')
    pkg_beatnaut_control = get_package_share_directory('beatnaut_control')

    # =========================================================
    # ARGUMENTS GLOBAUX
    # =========================================================
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Lancer Gazebo sans interface graphique si True'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Synchroniser tous les nœuds sur le temps de simulation Gazebo'
    )

    # =========================================================
    # COMPOSANT 1 : Simulation Gazebo complète
    # =========================================================
    # IncludeLaunchDescription() = "inclure un autre launch file".
    # C'est comme faire un `import` en Python — on réutilise simulation.launch.py.
    #
    # launch_arguments : transmettre les arguments au launch file inclus.
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_beatnaut_gazebo, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # =========================================================
    # COMPOSANT 2 : Nœud de contrôle différentiel
    # =========================================================
    # On attend 3 secondes avant de démarrer le contrôleur pour laisser
    # le temps à Gazebo de charger complètement.
    # TimerAction(period=..., actions=[...]) = démarrer après X secondes.
    control_node = TimerAction(
        period=3.0,  # secondes d'attente
        actions=[
            Node(
                package='beatnaut_control',
                executable='differential_drive_node',
                name='differential_drive_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    # Paramètres du contrôleur (peuvent être surchargés en CLI)
                    'wheel_separation': 0.50,    # Écartement entre les moteurs (m)
                    'max_thrust_rpm': 300.0,     # Vitesse max des hélices (rad/s)
                }]
            )
        ]
    )

    # =========================================================
    # ASSEMBLAGE FINAL
    # =========================================================
    return LaunchDescription([
        # Arguments
        headless_arg,
        use_sim_time_arg,

        # Simulation Gazebo (démarre immédiatement)
        simulation_launch,

        # Contrôleur (démarre 3s après pour laisser Gazebo s'initialiser)
        control_node,
    ])

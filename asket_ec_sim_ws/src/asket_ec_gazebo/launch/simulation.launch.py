"""
simulation.launch.py — Launch file pour démarrer la simulation Gazebo

=========================================================
QU'EST-CE QU'UN LAUNCH FILE ROS2 ?
=========================================================
En ROS2, un "launch file" est un script Python qui démarre automatiquement
plusieurs nœuds et processus en même temps.

Sans launch file, tu devrais ouvrir 4 terminaux séparés pour :
1. Terminal 1 : démarrer Gazebo avec le bon monde
2. Terminal 2 : spawner le robot dans Gazebo
3. Terminal 3 : démarrer robot_state_publisher
4. Terminal 4 : démarrer le bridge ROS2-Gazebo

Le launch file automatise tout ça en une seule commande :
  ros2 launch asket_ec_gazebo simulation.launch.py

=========================================================
STRUCTURE D'UN LAUNCH FILE ROS2
=========================================================
Un launch file Python doit définir une fonction `generate_launch_description()`
qui retourne un objet `LaunchDescription`.

Cette fonction peut contenir :
- Node()          : démarrer un nœud ROS2
- ExecuteProcess(): démarrer un processus système quelconque
- IncludeLaunchDescription(): inclure un autre launch file
- DeclareLaunchArgument(): déclarer des paramètres configurables
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Fonction principale du launch file.
    ROS2 l'appelle automatiquement quand tu lances `ros2 launch`.
    Elle doit retourner un LaunchDescription avec tous les composants à démarrer.
    """

    # =========================================================
    # ÉTAPE 1 : Trouver les chemins vers les packages
    # =========================================================
    # get_package_share_directory() cherche le package dans
    # asket_ec_sim_ws/install/<package_name>/share/<package_name>/
    # (l'endroit où `colcon build` copie les fichiers)

    # Chemin vers le package asket_ec_gazebo (ce package)
    pkg_asket_ec_gazebo = get_package_share_directory('asket_ec_gazebo')

    # Chemin vers le package asket_ec_description (modèle du bateau)
    pkg_asket_ec_description = get_package_share_directory('asket_ec_description')

    # =========================================================
    # ÉTAPE 2 : Construire les chemins vers les fichiers
    # =========================================================

    # Fichier SDF du monde Gazebo
    world_file = os.path.join(pkg_asket_ec_gazebo, 'worlds', 'asket_ec_world.sdf')

    # Fichier SDF du modèle du bateau — utilisé pour spawner dans Gazebo
    robot_sdf_file = os.path.join(pkg_asket_ec_description, 'urdf', 'asket_ec.sdf')

    # Fichier URDF minimal — utilisé par robot_state_publisher pour les TF
    # (le SDF complet n'est pas compatible avec sdformat_urdf quand il contient
    # une balise <pose> au niveau <model>)
    robot_urdf_file = os.path.join(pkg_asket_ec_description, 'urdf', 'asket_ec.urdf')

    # Fichier de configuration du bridge
    bridge_config_file = os.path.join(pkg_asket_ec_gazebo, 'config', 'ros_gz_bridge.yaml')

    # =========================================================
    # ÉTAPE 3 : Déclarer les arguments du launch
    # =========================================================
    # Les arguments permettent de personnaliser le lancement depuis la CLI.
    # Exemple : ros2 launch asket_ec_gazebo simulation.launch.py headless:=true

    # Argument : mode headless (sans interface graphique, utile pour les serveurs)
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Lancer Gazebo en mode headless (sans GUI) si True'
    )

    # Argument : utiliser le temps de simulation
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Utiliser le temps de simulation Gazebo plutôt que le temps système'
    )

    # =========================================================
    # ÉTAPE 4 : Lire le contenu URDF pour robot_state_publisher
    # =========================================================
    # robot_state_publisher a besoin du contenu XML (pas juste le chemin).
    # On utilise l'URDF minimal (pas le SDF) car sdformat_urdf ne supporte pas
    # la balise <pose> au niveau <model> → crash "not currently supported".
    with open(robot_urdf_file, 'r') as f:
        robot_description_content = f.read()

    # =========================================================
    # COMPOSANT 1 : Gazebo Harmonic
    # =========================================================
    # ExecuteProcess démarre Gazebo comme un processus système.
    # `gz sim` est la commande pour Gazebo Harmonic (anciennement `ign gazebo`).
    #
    # Arguments Gazebo :
    # -r  = run immediately (démarrer la simulation au lancement, sans pause)
    # -v4 = verbosity level 4 (logs détaillés pour le débogage)
    #
    # Pour Gazebo Harmonic, la variable d'environnement GZ_VERSION=harmonic
    # peut être nécessaire selon ton installation.
    # Variables d'environnement communes aux deux modes Gazebo
    gz_env = {
        # GZ_SIM_RESOURCE_PATH : dit à Gazebo où chercher les modèles
        'GZ_SIM_RESOURCE_PATH': ':'.join(
            os.path.join(p, 'share')
            for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':')
            if p
        ),
        # Rendu logiciel (CPU) pour WSL2 (pas de GPU OpenGL)
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'MESA_GL_VERSION_OVERRIDE': '3.3',
    }

    # Mode GUI (fenêtre graphique) — lancé si headless:=false (défaut)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        additional_env=gz_env,
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    # Mode headless (serveur uniquement) — lancé si headless:=true
    # -s = server only, pas de rendu graphique. Idéal pour WSL2/CI.
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
        additional_env=gz_env,
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    # =========================================================
    # COMPOSANT 2 : robot_state_publisher
    # =========================================================
    # Ce nœud lit la description du robot (SDF/URDF) et publie les
    # transformations TF2 (Transform Frame 2) de toutes les pièces du robot.
    #
    # TF2 = système de référentiels de ROS2
    # Permet à n'importe quel nœud de savoir "où est le capteur GPS par rapport
    # au centre du bateau ?" ou "dans quelle direction pointe le bateau ?".
    #
    # Sans TF2, RViz2 ne pourrait pas afficher les données capteurs correctement.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # =========================================================
    # COMPOSANT 3 : Bridge ROS2 <-> Gazebo
    # =========================================================
    # Ce nœud fait la traduction entre les messages Gazebo et ROS2.
    # Il lit sa configuration depuis ros_gz_bridge.yaml.
    #
    # Sans ce bridge :
    # - Les données IMU de Gazebo ne seraient pas visibles dans ROS2
    # - Les commandes moteurs de ROS2 n'arriveraient pas dans Gazebo
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'qos_overrides./clock.publisher.reliability': 'best_effort',
        }]
    )

    # =========================================================
    # COMPOSANT 4 : Spawn du robot dans Gazebo
    # =========================================================
    # On attend 3 secondes que Gazebo soit prêt, puis on spawne le bateau
    # via ros_gz_sim/create — plus fiable que package:// URI dans le world SDF.
    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', robot_sdf_file,
                '-name', 'asket_ec',
                '-x', '0', '-y', '0', '-z', '0.15',
            ],
            output='screen',
        )]
    )

    # =========================================================
    # ASSEMBLAGE : Retourner tous les composants
    # =========================================================
    return LaunchDescription([
        headless_arg,
        use_sim_time_arg,
        gazebo,           # Gazebo avec GUI (si headless:=false)
        gazebo_headless,  # Gazebo sans GUI / -s (si headless:=true)
        robot_state_publisher,
        ros_gz_bridge,
        spawn_robot,
    ])

# Beatnaut Simulation — ROS2/Gazebo Harmonic

Simulation complète du bateau autonome **Beatnaut** avec ROS2 Jazzy et Gazebo Harmonic.

```
    ╔══════════════════════════════════════╗
    ║    ████████ BEATNAUT ████████        ║
    ║   ═══════════════════════════        ║
    ║   [port]  ←1.50m→  [stbd]           ║
    ║    🌊🌊🌊🌊🌊🌊🌊🌊🌊🌊               ║
    ╚══════════════════════════════════════╝
```

---

## Qu'est-ce que ce projet ?

Ce repository contient un **workspace ROS2 complet** pour simuler le Beatnaut,
un catamaran télécommandé/autonome construit par un club d'ingénierie étudiant.

**Le Beatnaut physique :**
- Catamaran de 1.50m × 0.60m, masse 10 kg
- 2 propulseurs BLDC (contrôle différentiel, comme un tank)
- GPS + IMU pour la navigation
- ESP32 comme contrôleur embarqué
- ROS2 Jazzy comme middleware robotique

**Ce que simule ce projet :**
- La physique hydrodynamique (flottaison d'Archimède + résistance de l'eau)
- Les deux propulseurs avec physique réaliste (coefficient de poussée Kt)
- Les capteurs IMU (orientation + accélération) et GPS (position géographique)
- Le contrôle différentiel gauche/droite

---

## Prérequis

### Logiciels requis

| Logiciel | Version | Installation |
|----------|---------|-------------|
| Ubuntu | 24.04 LTS | — |
| ROS2 | Jazzy Jalisco | [docs.ros.org](https://docs.ros.org/en/jazzy/Installation.html) |
| Gazebo | Harmonic (8.x) | `sudo apt install gz-harmonic` |
| ros_gz | jazzy | `sudo apt install ros-jazzy-ros-gz` |

### Installation des dépendances ROS2

```bash
# Mettre à jour les sources
sudo apt update

# Installer ROS2 Jazzy (desktop complet)
sudo apt install ros-jazzy-desktop

# Installer Gazebo Harmonic
sudo apt install gz-harmonic

# Installer le bridge ROS2-Gazebo
sudo apt install ros-jazzy-ros-gz

# Installer les outils de build
sudo apt install python3-colcon-common-extensions python3-rosdep

# Initialiser rosdep (une seule fois)
sudo rosdep init
rosdep update
```

---

## Structure du projet

```
beatnaut_sim_ws/
├── src/
│   │
│   ├── beatnaut_description/        # Package : MODÈLE du bateau
│   │   ├── package.xml              # Dépendances ROS2 du package
│   │   ├── CMakeLists.txt           # Instructions de build
│   │   └── urdf/
│   │       └── beatnaut.sdf         # ← FICHIER PRINCIPAL : modèle 3D + physique
│   │                                #   Définit géométrie, masse, inertie,
│   │                                #   capteurs (IMU/GPS), plugins physique
│   │
│   ├── beatnaut_gazebo/             # Package : SIMULATION Gazebo
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── worlds/
│   │   │   └── beatnaut_world.sdf   # Monde Gazebo : eau, soleil, bateau inclus
│   │   ├── config/
│   │   │   └── ros_gz_bridge.yaml   # Topics bridgés entre Gazebo et ROS2
│   │   └── launch/
│   │       ├── simulation.launch.py # Lance Gazebo + bridge
│   │       └── beatnaut_full.launch.py  # ← POINT D'ENTRÉE PRINCIPAL
│   │
│   └── beatnaut_control/            # Package : CONTRÔLE des moteurs
│       ├── package.xml
│       ├── setup.py
│       ├── beatnaut_control/
│       │   └── differential_drive_node.py  # Nœud : /cmd_vel → propulseurs
│       └── launch/
│           └── control.launch.py    # Lance uniquement le contrôleur
│
├── build.sh                         # Script de compilation automatique
└── README.md                        # Ce fichier
```

---

## Installation et build

### 1. Cloner le repository

```bash
git clone https://github.com/auxenceIAAC/simulation_engineeringclub.git
cd simulation_engineeringclub
```

### 2. Compiler le workspace

```bash
# Méthode rapide (script automatique)
bash beatnaut_sim_ws/build.sh

# OU méthode manuelle
source /opt/ros/jazzy/setup.bash
cd beatnaut_sim_ws
colcon build --symlink-install
```

### 3. Sourcer l'installation

> **Important** : cette commande doit être exécutée dans **chaque nouveau terminal**
> qui veut utiliser ce workspace.

```bash
source beatnaut_sim_ws/install/setup.bash
```

**Astuce** : Ajoute cette ligne à ton `~/.bashrc` pour ne pas avoir à la retaper :
```bash
echo "source ~/simulation_engineeringclub/beatnaut_sim_ws/install/setup.bash" >> ~/.bashrc
```

---

## Lancer la simulation

### Simulation complète (Gazebo + contrôleur)

```bash
ros2 launch beatnaut_gazebo beatnaut_full.launch.py
```

Cela démarre :
1. **Gazebo Harmonic** avec le monde beatnaut_world.sdf
2. **robot_state_publisher** (transformations TF2)
3. **ros_gz_bridge** (bridge ROS2 <-> Gazebo)
4. **differential_drive_controller** (après 3s de délai)

### Simulation Gazebo seulement (sans contrôleur)

```bash
ros2 launch beatnaut_gazebo simulation.launch.py
```

### Contrôleur seulement (si Gazebo tourne déjà)

```bash
ros2 launch beatnaut_control control.launch.py
```

### Mode headless (sans interface graphique, pour serveurs)

```bash
ros2 launch beatnaut_gazebo beatnaut_full.launch.py headless:=true
```

---

## Contrôler le bateau

### Commandes de base

```bash
# Faire avancer le bateau à 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Faire reculer
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.3}}" --once

# Tourner à gauche (angular.z positif)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.5}}" --once

# Tourner à droite (angular.z négatif)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: -0.5}}" --once

# Rotation sur place à gauche (v=0, w>0)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 1.0}}" --once

# ARRET D'URGENCE
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

### Contrôle direct des propulseurs

```bash
# Propulseur gauche (port) à 150 rad/s
ros2 topic pub /beatnaut/thruster/port/cmd std_msgs/msg/Float64 "{data: 150.0}" --once

# Propulseur droit (starboard) à 150 rad/s
ros2 topic pub /beatnaut/thruster/starboard/cmd std_msgs/msg/Float64 "{data: 150.0}" --once
```

---

## Lire les données capteurs

```bash
# Données GPS en temps réel (position lat/lon/alt)
ros2 topic echo /beatnaut/navsat

# Données IMU (orientation + accélération)
ros2 topic echo /beatnaut/imu

# Position et orientation du bateau dans le monde
ros2 topic echo /beatnaut/pose

# Odométrie estimée
ros2 topic echo /beatnaut/odometry

# Voir tous les topics actifs
ros2 topic list

# Voir la fréquence d'un topic
ros2 topic hz /beatnaut/imu
ros2 topic hz /beatnaut/navsat
```

---

## Architecture logicielle

```
+------------------------------------------------------------------+
|                    WORKSPACE ROS2 BEATNAUT                        |
|                                                                   |
|  +-----------------+         +-----------------------------+     |
|  |  Teleop         |         |    Gazebo Harmonic           |     |
|  |  (clavier/joy)  |         |                             |     |
|  +--------+--------+         |  +---------------------+   |     |
|           |  /cmd_vel        |  |  Modele Beatnaut     |   |     |
|           v                  |  |  - BuoyancyPlugin   |   |     |
|  +-----------------+         |  |  - HydroPlugin      |   |     |
|  |  Differential   |         |  |  - ThrusterPlugin   |   |     |
|  |  Drive Node     |         |  |  - IMU Sensor       |   |     |
|  +--+----------+---+         |  |  - GPS Sensor       |   |     |
|     |          |             |  +---------------------+   |     |
|     | port/cmd | stbd/cmd    +-------------+---------------+     |
|     v          v                           |                     |
|  +----------------------+  <--------------+                     |
|  |   ros_gz_bridge      |  /beatnaut/imu                        |
|  |  (ROS2 <-> Gazebo)   |  /beatnaut/navsat                     |
|  +----------------------+  /beatnaut/thruster/*/cmd             |
|           |                                                      |
|           v                                                      |
|  +----------------------+                                       |
|  | robot_state_pub      | -> /tf, /tf_static                    |
|  +----------------------+                                       |
+------------------------------------------------------------------+
```

---

## Topics ROS2

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Entree | Commande de vitesse du bateau |
| `/beatnaut/thruster/port/cmd` | `std_msgs/Float64` | Sortie -> Gazebo | Commande propulseur gauche (rad/s) |
| `/beatnaut/thruster/starboard/cmd` | `std_msgs/Float64` | Sortie -> Gazebo | Commande propulseur droit (rad/s) |
| `/beatnaut/imu` | `sensor_msgs/Imu` | Gazebo -> ROS2 | Donnees IMU (orientation + accel.) |
| `/beatnaut/navsat` | `sensor_msgs/NavSatFix` | Gazebo -> ROS2 | Position GPS (lat/lon/alt) |
| `/beatnaut/pose` | `geometry_msgs/PoseStamped` | Gazebo -> ROS2 | Position precise dans le monde |
| `/beatnaut/odometry` | `nav_msgs/Odometry` | Gazebo -> ROS2 | Odometrie estimee |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo -> ROS2 | Horloge de simulation |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_pub | Transformations entre les liens |

---

## Physique de la simulation

### Flottabilite (BuoyancyPlugin)
```
F_flottaison = rho_eau x V_immerge x g
             = 1000 x V x 9.81
```
Le bateau flotte quand `F_flottaison = F_gravite = m x g`.

### Resistance hydrodynamique (HydrodynamicsPlugin)
```
F_resistance = -xDotU x v  -  xUU x v x |v|
                 lineaire         quadratique
```

### Poussee des helices (ThrusterPlugin)
```
F_poussee = Kt x rho x n^2 x D^4
           = 0.004 x 1000 x n^2 x 0.08^4
```
Ou `n` est la vitesse angulaire de l'helice en rad/s.

### Controle differentiel
```
v_port      = v + (L/2) x omega    # Moteur gauche
v_starboard = v - (L/2) x omega    # Moteur droit
```
Ou `L = 0.50 m` (ecartement des moteurs).

---

## Depannage

### "gz: command not found"
```bash
# Verifier l'installation de Gazebo Harmonic
which gz
# Si absent : sudo apt install gz-harmonic
```

### "Package 'beatnaut_gazebo' not found"
```bash
# Tu as oublie de sourcer l'installation
source beatnaut_sim_ws/install/setup.bash
```

### Le bridge ne demarre pas
```bash
# Verifier que ros_gz_bridge est installe
ros2 pkg list | grep ros_gz_bridge
# Si absent : sudo apt install ros-jazzy-ros-gz
```

### La simulation est tres lente (< 0.5x real time)
- Reduire la resolution graphique dans Gazebo
- Utiliser le mode headless : `headless:=true`
- Augmenter `max_step_size` dans `beatnaut_world.sdf` (reduit la precision)

---

## Pour aller plus loin

- **Ajouter la navigation autonome** : integrer Nav2 avec une carte de la zone
- **Telemetrie WiFi** : bridge entre ESP32 et ROS2 via micro-ROS
- **Simulation de vagues** : plugin `gz-waves` de VRX (Virtual RobotX)
- **Camera** : ajouter un capteur camera dans le SDF pour la vision
- **Lidar** : simuler un lidar 2D pour la detection d'obstacles

---

## References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [ros_gz GitHub](https://github.com/gazebosim/ros_gz)
- [VRX (Virtual RobotX) — simulation de bateaux](https://github.com/osrf/vrx)
- [SDF Format Specification](http://sdformat.org/spec)

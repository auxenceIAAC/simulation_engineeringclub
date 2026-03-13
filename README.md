# Simulation Asket EC — Club d'Ingénierie

## Vue d'ensemble

Simulateur 2D ROS2 du bateau Asket EC. Pas de Gazebo —
simulateur Python pur à 50Hz, affiché dans RViz2.
Architecture : simulateur physique + navigateur waypoints GPS.

---

## Prérequis

- Ubuntu 24.04 (ou WSL2)
- ROS2 Jazzy installé
- Python 3.12

---

## Installation

### 1. Cloner le repo

```bash
git clone https://github.com/auxenceIAAC/simulation_engineeringclub
cd simulation_engineeringclub
```

### 2. Builder le workspace

```bash
cd asket_ec_sim_ws
colcon build --packages-select asket_ec_sim2d
source install/setup.bash
```

### 3. Installer les nœuds Python

```bash
cd src/asket_ec_sim2d
pip install -e . --break-system-packages
cd ../../..
```

---

## Lancement

Le projet nécessite 3 terminaux. Dans chaque terminal,
sourcer ROS2 d'abord :

```bash
source /opt/ros/jazzy/setup.bash
```

### Terminal 1 — Simulateur physique

```bash
~/.local/bin/simulator_node
```

Publie à 50Hz :
- `/sim2d/pose` — position et cap du bateau (`PoseStamped`)
- `/sim2d/path` — trajectoire complète (`Path`)
- `/sim2d/odom` — odométrie (`Odometry`)
- `/sim2d/navsat` — GPS simulé ancré sur Barcelone (`NavSatFix`)

Souscrit à :
- `/cmd_vel` — commandes de vitesse (`Twist`)

### Terminal 2 — Navigateur waypoints (mode automatique)

```bash
~/.local/bin/waypoint_navigator_node
```

Lit les waypoints depuis :
```
asket_ec_sim_ws/src/asket_ec_sim2d/config/waypoints.yaml
```

Comportement :
- Trie les waypoints du plus proche au plus éloigné
- Dirige le bateau vers chaque waypoint via pure pursuit
- Passe au waypoint suivant quand le bateau est à moins de 2m
- Publie `/cmd_vel` automatiquement

Format `waypoints.yaml` :

```yaml
waypoints:
  - {lat: 41.3851, lon: 2.1734}
  - {lat: 41.3860, lon: 2.1750}
```

### Terminal 3 — Visualisation RViz2

```bash
rviz2 -d ~/simulation_engineeringclub/asket_ec_sim_ws/src/asket_ec_sim2d/config/sim2d.rviz
```

Affiche :
- Trajectoire réelle du bateau en vert (`/sim2d/path`)
- Plan de route waypoints en rouge (`/waypoints/path`)
- Pose actuelle du bateau (`/sim2d/pose`)
- Grille de référence (Fixed Frame : `odom`)

---

## Contrôle manuel

Pour contrôler manuellement le bateau (remplace le Terminal 2) :

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

Paramètres :
- `linear.x` : avance (positif) / recule (négatif), max ≈ 1.0
- `angular.z` : tourne à gauche (positif) / droite (négatif), max ≈ 1.0

---

## Architecture technique

### Physique du bateau

- Propulsion différentielle (2 thrusters)
- Écartement moteurs : 0.5m
- Drag coefficient : 3.0 (résistance eau)
- Fréquence simulation : 50Hz
- Origine GPS : 41.3851°N, 2.1734°E (Barcelone)

### Algorithme de navigation (pure pursuit simplifié)

```
angle_error = angle_vers_waypoint - cap_actuel
angular.z = 2.0 * sin(angle_error)
linear.x  = 0.5 * cos(angle_error)
```

### Structure du repo

```
simulation_engineeringclub/
└── asket_ec_sim_ws/
    └── src/
        └── asket_ec_sim2d/
            ├── asket_ec_sim2d/
            │   ├── simulator_node.py           # physique 50Hz
            │   └── waypoint_navigator_node.py  # navigation GPS
            ├── config/
            │   ├── waypoints.yaml              # points GPS cibles
            │   └── sim2d.rviz                  # config RViz2
            ├── launch/
            │   └── sim2d.launch.py
            ├── setup.cfg
            └── package.xml
```

---

## Prochaines étapes

- [ ] Mode manuel / automatique avec commutation via topic
- [ ] Intégration QGroundControl via MAVLink + MAVROS
- [ ] Modèle physique plus réaliste (vent, courant)

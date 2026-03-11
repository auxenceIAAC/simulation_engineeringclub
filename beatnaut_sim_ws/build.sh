#!/bin/bash
# =============================================================================
# build.sh — Script de compilation du workspace ROS2 Beatnaut
#
# CE SCRIPT FAIT QUOI ?
# Ce script automatise toute la procédure de compilation du workspace ROS2.
# Normalement, il faut faire plusieurs étapes manuelles :
#   1. Sourcer l'environnement ROS2
#   2. Se placer dans le bon dossier
#   3. Lancer colcon build
#   4. Sourcer l'installation
#
# Ce script fait tout ça en une seule commande.
#
# UTILISATION :
#   cd beatnaut_sim_ws
#   bash build.sh
#
# OU depuis la racine du repo :
#   bash beatnaut_sim_ws/build.sh
#
# PRÉREQUIS :
#   - ROS2 Jazzy installé dans /opt/ros/jazzy/
#   - Gazebo Harmonic installé (gz-sim >= 8.0)
#   - ros_gz installé : sudo apt install ros-jazzy-ros-gz
# =============================================================================

set -e  # Arrêter le script si une commande échoue (fail-fast)

# Couleurs pour l'affichage (rend les messages plus lisibles)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color (réinitialise la couleur)

echo -e "${BLUE}=======================================${NC}"
echo -e "${BLUE}  Build du workspace Beatnaut Sim     ${NC}"
echo -e "${BLUE}=======================================${NC}"

# =============================================================================
# ÉTAPE 1 : Vérifier que ROS2 est installé
# =============================================================================
echo -e "\n${YELLOW}[1/4] Vérification de l'installation ROS2...${NC}"

if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo -e "${RED}ERREUR : ROS2 Jazzy non trouvé dans /opt/ros/jazzy/${NC}"
    echo "Installer ROS2 Jazzy : https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 Jazzy trouvé${NC}"

# =============================================================================
# ÉTAPE 2 : Sourcer l'environnement ROS2
# =============================================================================
# "source" = charger les variables d'environnement ROS2 dans le shell actuel.
# Sans cette étape, les commandes ros2, colcon, etc. ne sont pas trouvées.
echo -e "\n${YELLOW}[2/4] Chargement de l'environnement ROS2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash
echo -e "${GREEN}✓ Environnement ROS2 chargé${NC}"

# =============================================================================
# ÉTAPE 3 : Se placer dans le workspace
# =============================================================================
# Le script peut être appelé depuis n'importe où, donc on trouve
# son emplacement pour toujours compiler le bon workspace.
echo -e "\n${YELLOW}[3/4] Navigation vers le workspace...${NC}"

# Trouver le dossier du script (même s'il est appelé depuis ailleurs)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${GREEN}✓ Workspace : $(pwd)${NC}"

# Vérifier que le dossier src/ existe
if [ ! -d "src" ]; then
    echo -e "${RED}ERREUR : Dossier src/ non trouvé dans $(pwd)${NC}"
    echo "Vérifier que tu es dans le bon dossier workspace."
    exit 1
fi

# =============================================================================
# ÉTAPE 4 : Compiler avec colcon
# =============================================================================
# colcon = outil de build de ROS2 (remplace catkin de ROS1)
#
# OPTIONS UTILISÉES :
# --symlink-install : au lieu de copier les fichiers, crée des liens symboliques.
#                     Avantage : modifier un launch file .py ne nécessite pas
#                     de recompiler — les changements sont visibles immédiatement !
# --event-handlers console_direct+ : affiche les logs en temps réel
# --cmake-args -DCMAKE_BUILD_TYPE=Release : optimisations C++ (si applicable)
echo -e "\n${YELLOW}[4/4] Compilation du workspace avec colcon...${NC}"
echo -e "${BLUE}(Cela peut prendre 1-2 minutes la première fois)${NC}\n"

colcon build \
    --symlink-install \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# =============================================================================
# SUCCÈS !
# =============================================================================
echo -e "\n${GREEN}=======================================${NC}"
echo -e "${GREEN}  ✓ Build terminé avec succès !        ${NC}"
echo -e "${GREEN}=======================================${NC}"

echo -e "\n${YELLOW}Pour utiliser le workspace, exécute DANS TON TERMINAL :${NC}"
echo -e "  ${BLUE}source install/setup.bash${NC}"
echo -e "\n${YELLOW}Puis lance la simulation :${NC}"
echo -e "  ${BLUE}ros2 launch beatnaut_gazebo beatnaut_full.launch.py${NC}"
echo -e "\n${YELLOW}Ou juste la simulation Gazebo :${NC}"
echo -e "  ${BLUE}ros2 launch beatnaut_gazebo simulation.launch.py${NC}"
echo -e "\n${YELLOW}Dans un autre terminal (après avoir sourcé) :${NC}"
echo -e "  ${BLUE}ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.5}}\" --once${NC}"

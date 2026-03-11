cmake_minimum_required(VERSION 3.8)
project(beatnaut_description)

# Cherche ament_cmake, l'outil de build ROS2 basé sur CMake
find_package(ament_cmake REQUIRED)

#
# install() — dit à colcon "copie ces dossiers dans l'installation finale"
#
# Quand tu lances `colcon build`, ROS2 copie les fichiers dans
# beatnaut_sim_ws/install/beatnaut_description/share/beatnaut_description/
# C'est là que les autres packages iront chercher les fichiers SDF, configs, etc.
#
install(
  DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(beatnaut_gazebo)

find_package(ament_cmake REQUIRED)

#
# Installe tous les fichiers nécessaires dans le répertoire d'installation ROS2.
# Sans ces install(), les launch files et fichiers de config ne seraient pas
# trouvés par `ros2 launch`.
#
install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

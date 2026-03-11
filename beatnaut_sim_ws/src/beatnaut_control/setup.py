"""
setup.py — Configuration du package Python ROS2

Ce fichier dit à Python (et à colcon) comment installer ce package.
Pour les packages ROS2 Python, c'est l'équivalent du CMakeLists.txt.

POINTS IMPORTANTS :
- entry_points : dit à ROS2 quels scripts Python peuvent être lancés
  avec `ros2 run <package> <executable>`
  Format : 'nom_executable = module.python:fonction_main'
- data_files : liste des fichiers non-Python à installer (launch files, configs)
"""

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'beatnaut_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Fichier obligatoire : enregistre le package dans l'index ROS2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Manifeste du package
        ('share/' + package_name, ['package.xml']),
        # Fichiers de lancement (launch files)
        # glob('launch/*.py') trouve automatiquement tous les .py dans launch/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Fichiers de configuration (si ajoutés plus tard)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Beatnaut Engineering Club',
    maintainer_email='beatnaut@engineering.club',
    description='Contrôleur différentiel pour le bateau Beatnaut',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # console_scripts : liste des exécutables ROS2 créés par ce package
        # Chaque ligne crée une commande : ros2 run beatnaut_control <nom>
        'console_scripts': [
            # 'differential_drive_node' = nom de la commande ros2 run
            # 'beatnaut_control.differential_drive_node' = module Python
            # ':main' = fonction à appeler dans ce module
            'differential_drive_node = beatnaut_control.differential_drive_node:main',
        ],
    },
)

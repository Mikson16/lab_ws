from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Retornamos el archivo de tipo launch
   return LaunchDescription([
        Node(
            package='lab_1_pkg',
            executable='pose_loader.py',
            name='pose_loader'),
        Node(
            package='lab_1_pkg',
            executable='obstacle_detector.py',
            name='obstacle_detector'
        ),
        Node(
            package='lab_1_pkg',
            executable='dead_reckoning_nav_obstacle.py',
            name='dead_reckoning_nav_obstacle'
        )#Levantar para testear#
        ])
#        Node(
#           package='lab_1_pkg',
#            executable='dead_reckoning_nav.py',
#            name='dead_reckoning_nav'
#        ),
#Agregar en caso de ser necesario

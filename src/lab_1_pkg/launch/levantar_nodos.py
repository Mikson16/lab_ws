from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Retornamos el archivo de tipo launch
   return LaunchDescription([
        Node(
            package='simulation_connection',
            executable='pose_loader.py',
            name='pose_loader'),
        Node(
           package='simulation_connection',
            executable='dead_reckoning_nav.py',
            name='dead_reckoning_nav'
        )     
        ])

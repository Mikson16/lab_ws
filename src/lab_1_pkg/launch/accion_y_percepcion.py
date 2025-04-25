from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription



def generate_launch_description():
    # Cargamos la ruta para levantar el launch de la simulacion

    simulacion_launch = os.path.join(
                        get_package_share_directory('very_simple_robot_simulator'),
                                                     'launch/run_all.xml')
    
    # Cargamos nuestros nodos
    pose_loader_nodo = Node(package='lab_1_pkg',
                               executable='pose_loader.py',
                               name='pose_loader')
    dead_reckoning_nodo = Node(
           package='lab_1_pkg',
            executable='dead_reckoning_nav_obstacle.py',
            name='dead_reckoning_nav'
        )  

    kinect_nodo = Node(
           package='lab_1_pkg',
            executable= 'obstacle_detector.py',
            name='obstacle'
        )    
    # Retornamos el archivo de tipo launch
    launch = LaunchDescription([
        IncludeLaunchDescription(XMLLaunchDescriptionSource(simulacion_launch)),
        dead_reckoning_nodo, 
        kinect_nodo,
        pose_loader_nodo])
    return launch

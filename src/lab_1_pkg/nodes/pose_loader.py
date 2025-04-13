#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler
import os

class PoseLoader(Node):

    def __init__(self):
        super().__init__('pose_loader') 
        #Envio de lista de poses goal list

        self.publisher_ = self.create_publisher(PoseArray, 'goal_list', 10)
        #Tiempo de envio entre mensajes
        self.timer = self.create_timer(1.0, self.publish_goals)
        self.get_logger().info('Llegue aki pose loader')

    def publish_goals(self):
        #Mensaje
        poses = PoseArray()
        #Marco de coordenadas
        poses.header.frame_id = "map"
        self.get_logger().info('Map listo')

        #Leer el archivo
        #self.get_logger().info(f"Ruta completa:, {os.path.abspath(__file__)}")
        #valor = os.path.join('coordenadas.txt')
        #self.get_logger().info(f"{valor}")
        #self.get_logger().info(f"Imprimiendo {dir(os.path)}")
        #ruta = os.path('/home/maxipis/ros2_ws/src/simulation_connection/nodes/coordenadas.txt')

        try:
            # ! Areglar la ruta a relativa
            with open('/home/maxipis/ros2_ws/src/simulation_connection/nodes/coordenadas.txt', 'r') as file:
                self.get_logger().info('Lei bien el archivo')
                for line in file:
                    x, y, theta = map(float, line.strip().split(','))

                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y

                    # ✅ Convertimos theta (en radianes) a quaternion
                    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
                    pose.orientation.x = qx
                    pose.orientation.y = qy
                    pose.orientation.z = qz
                    pose.orientation.w = qw

                    poses.poses.append(pose)
            self.publisher_.publish(poses)
            self.get_logger().info('Lista de poses publicada en /goal_list.')

        except FileNotFoundError:

            self.get_logger().error(f'Archivo no encontrado: {self.pose_file}')
        except Exception as e:
            self.get_logger().error(f'Error al leer el archivo: {e}')

def main(args=None):
    
    rclpy.init(args=args)  
    node = PoseLoader()  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown() 

if __name__ == '__main__':
     main()

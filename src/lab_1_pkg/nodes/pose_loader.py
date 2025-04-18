#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler
import os
import numpy as np

def convertir_formato_valido(lista):
    """
    input: ["1.0", "0.2", "pi/2"]
    output: [1.0, 0.2, 1.57...]
    """
    x = float(lista[0])
    y = float(lista[1])
    if 'pi' in lista[2]:
        # Eliminamos los primeros 3 caracteres: p, i, /
        # Nos quedamos con el divisor
        theta = np.pi/float(lista[2][3:])
    else:
        theta = float(lista[2])
    return x, y, theta

class PoseLoader(Node):

    def __init__(self):
        super().__init__('pose_loader')
        #Envio de lista de poses goal list

        self.publisher_ = self.create_publisher(PoseArray, 'goal_list', 10)
        #Tiempo de envio entre mensajes
        self.timer = self.create_timer(1.0, self.publish_goals)
        self.get_logger().info('1/3: Creado el publisher')
        # Como buscamos solo enviar un mensaje, emitimos una vez
        self.contador_msg = 0

    def publish_goals(self):
        if self.contador_msg == 0:
            #Mensaje
            poses = PoseArray()
            #Marco de coordenadas
            poses.header.frame_id = "map"

            try:
                #! Siempre agregar el archivo de input a la ruta <workspace>/install/lib/<package>#
                # Lo de arriba parece que no es necesario
                # Solucion: Anadir al CMakeList el archivo .txt desde nodes para que luego al hacer symlink (una vez)
                # se genere en el instaa/lib. Si realizamos cambios al coordenadas.txt debe ser al de src/lab...
                dir_route = os.path.dirname(os.path.abspath(__file__)) #Consigue la ruta del directorio actual
                coordenadas_file_path = os.path.join(dir_route, 'coordenadas.txt') #Hace join del directorio actual con el nombre del archivo
                with open(coordenadas_file_path, 'r') as file:
                    self.get_logger().info('2/3 Archivo leido')
                    for line in file:
                        # Convertimos en el formato correcto
                        line = line.replace(" ", "")
                        x, y, theta = convertir_formato_valido(line.strip().split(','))
                        self.get_logger().info(f'{x}, {y}, {theta}')
                        self.get_logger().info(f' ')
                        
                        pose = Pose()
                        pose.position.x = x
                        pose.position.y = y

                        #  Convertimos theta (en radianes) a quaternion
                        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
                        pose.orientation.x = qx
                        pose.orientation.y = qy
                        pose.orientation.z = qz
                        pose.orientation.w = qw

                        poses.poses.append(pose)
                self.publisher_.publish(poses)
                self.get_logger().info('3/3 Lista de poses publicada en /goal_list.')
                self.contador_msg = 1

            except FileNotFoundError:

                self.get_logger().error(f'Archivo no encontrado: {self.pose_file}')
            except Exception as e:
                self.get_logger().error(f'Error al leer el archivo: {e}')
        else:
            pass
def main(args=None):

    rclpy.init(args=args)
    node = PoseLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()

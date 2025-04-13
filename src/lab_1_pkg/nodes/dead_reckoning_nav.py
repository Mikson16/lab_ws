#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
# Importamos el mensaje segun el tipo que recibimos
from geometry_msgs.msg import PoseArray
# Importamos el mensaje para leer la odometria
from nav_msgs.msg import Odometry
# Ocupamos euler from quaternion para convertir el mensaje enviado
from tf_transformations import euler_from_quaternion


class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('nodito')
        # Nos suscribimos al topico
        self.subscription_array = self.create_subscription(
            PoseArray,              # Tipo de mensaje
            'goal_list',         # Nombre del tópico
            self.listener_callback,
            10                   # QoS
        )

        # self.subscription_odom = self.create_subscription(
        #     Odometry,
        #     '/odom',
        #     self.read_odometry,
        #     10
        # )
        self.odometria_actual = ""
        self.get_logger().info('Realizado ')


    def listener_callback(self, msg):
        # Recibimos el mensaje del topico
        self.get_logger().info(f'Recibido: "{msg}"')


        for i, pose in enumerate(msg.poses):
            self.get_logger().info(f"{i}")
            pos = pose.position
            ori = pose.orientation
            self.get_logger().info(
                f'Pose {i}: '
                f'Pos(x={pos.x}, y={pos.y}, z={pos.z}), '
                f'Ori(x={ori.x}, y={ori.y}, z={ori.z}, w={ori.w})'
            )
            self.get_logger().info("")
            # Almacenamos en un array [x,y,teta]
            #array_pose = [pos.x, pos.y, pos.w]
            
            # Enviamos cada a la funcion mover robot
            #self.mover_robot_a_destino(array_pose)

    def read_odometry(self, odom ):
        # Metodo encargado de leer la odometria 
        # y reenviar para obtener la velocidad a enviar
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                                                odom.pose.pose.orientation.y,
                                                                                odom.pose.pose.orientation.z,
                                                                                odom.pose.pose.orientation.w ) )
        #self.get_logger().info( 'Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) 
        #! Probar si es que se puede retornar directamente en el otro metodo
        return x, y, roll

    def mover_robot_a_destino(self, array_pose):
        pass
        


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()

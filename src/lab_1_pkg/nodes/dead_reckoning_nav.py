#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
# Importamos el mensaje segun el tipo que recibimos
from geometry_msgs.msg import PoseArray, Twist
# Importamos el mensaje para leer la odometria
from nav_msgs.msg import Odometry
# Ocupamos euler from quaternion para convertir el mensaje enviado
from tf_transformations import euler_from_quaternion
import math
import time

class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav')

        #Suscripción a Pose_Loader
        self.subscription_array = self.create_subscription(PoseArray,'goal_list',self.listener_callback,10)

        #Suscripción a la odometría
        self.subscription_odom = self.create_subscription(Odometry,'/odom',self.read_odometry_callback,10)

        #Publicador de velocidades
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel_mux/input/navigation',10)

        self.odometria_actual = ""
        self.get_logger().info('Nodo dead_reckoning_nav iniciado.')

    def listener_callback(self, msg):
        #Recibe la lista de poses objetivo
        self.get_logger().info(f'Recibidas {len(msg.poses)} poses.')

        for i, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            q = pose.orientation
            _, _, theta = euler_from_quaternion((q.x, q.y, q.z, q.w))

            goal = [x, y, theta]
            self.get_logger().info(f"Moviendo al objetivo {i}: {goal}")
            self.mover_robot_a_destino(goal)

        self.get_logger().info("Todos los objetivos fueron alcanzados.")

    def read_odometry_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))

        self.odometria_actual = (x, y, yaw)

    def mover_robot_a_destino(self, goal):
        #Esperar hasta que tengamos odometría válida
        while self.odometria_actual == "":
            rclpy.spin_once(self)

        x_goal, y_goal, theta_goal = goal

        while rclpy.ok():
            x, y, theta = self.odometria_actual

            #Calcular distancia y ángulo al objetivo
            dx = x_goal - x
            dy = y_goal - y
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_goal = math.atan2(dy, dx)

            #Error angular al objetivo
            angle_error = angle_to_goal - theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # normalizado

            twist = Twist()

            if distance > 0.05:
                if abs(angle_error) > 0.1:
                    twist.angular.z = 0.5 * angle_error
                else:
                    twist.linear.x = 0.2
            else:
                #Orientación
                final_error = theta_goal - theta
                final_error = math.atan2(math.sin(final_error), math.cos(final_error))
                if abs(final_error) > 0.1:
                    twist.angular.z = 0.3 * final_error
                else:
                    break

            self.publisher_cmd_vel.publish(twist)
            rclpy.spin_once(self)
            time.sleep(0.1)

        #Detener el robot
        self.publisher_cmd_vel.publish(Twist())
        time.sleep(0.5)
    
    def aplicar_velocidad(self, lista_triplets):
        """
        El argumento corresponde a una lista de triplets, donde cada uno contendra la velocidad lineal, angular y el tiempo de aplicacion de dichas velocidades, esta funcion no se encarga de calcular como llegar a la pose objetivo, solo aplica las velocidades en una serie de tiempo"""

        twist = Twist()


        for triplet in lista_triplets:
            v_lineal, w_angular, tiempo = triplet
            
            # Aplicar velocidades lineales
            twist.linear.x = v_lineal
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            # Aplicar velocidades angulares
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = w_angular

            # Publicar el mensaje de velocidad
            self.publisher_cmd_vel.publish(twist)
            self.get_logger().info(f"Aplicando velocidades: {v_lineal}, {w_angular} durante {tiempo} segundos")
            # Esperar el tiempo especificado
            time.sleep(tiempo)
        
        # Detener el robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info("Robot detenido.")


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

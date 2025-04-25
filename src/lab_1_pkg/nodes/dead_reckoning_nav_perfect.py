#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
# Importamos el mensaje segun el tipo que recibimos
from geometry_msgs.msg import PoseArray, Twist
# Ocupamos euler from quaternion para convertir el mensaje enviado
from tf_transformations import euler_from_quaternion
import math
import time
from parametros import VELOCIDAD_LINEAL, VELOCIDAD_ANGULAR
import threading

class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav')
        #Suscripción a Pose_Loader
        self.subscription_array = self.create_subscription(PoseArray,'goal_list',self.accion_mover,10)
        #Publicador de velocidades
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel_mux/input/navigation',10)
        # Variable para guardar la pose anterior
        self.ultima_pose = None
        self.get_logger().info('Nodo dead_reckoning_nav iniciado.')
    
    def accion_mover(self, msg):
        """
        Input: msg del topico goal_list recibe una lista de posiciones
        Output: llamar al metodo para actualizar la posicion
        """
        #Recibe la lista de poses objetivo
        self.get_logger().info(f'Recibidas {len(msg.poses)} poses.')

        for i, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            q = pose.orientation
            _, _, theta = euler_from_quaternion((q.x, q.y, q.z, q.w))

            goal = [x, y, theta]
            self.get_logger().info(f" ")

            # self.get_logger().info(f"Moviendo al objetivo {i}: {goal}")
            self.mover_robot_a_destino(goal)

        self.get_logger().info("Todos los objetivos fueron alcanzados.")

    def mover_robot_a_destino(self, goal):
        """
        Input: posicion destino, odometria (enviada por el topico)
        Output: enviar la velocidad deseada y el tiempo de aplicacion al metodo aplicar_velocidad 
        """
        if self.ultima_pose == None:
            self.ultima_pose = [0,0,0]
        x_anterior, y_anterior, theta_anterior = self.ultima_pose
        x_goal, y_goal, theta_goal = goal
        
        if theta_goal < 0: #! Cambiar
            # self.get_logger().info("Cambiamos el signo")
            # self.get_logger().info(f"    La tupla ultima fue {self.ultima_pose}")
            theta_goal = abs(theta_goal)

        tiempo_lineal_x = abs((x_goal - x_anterior)/VELOCIDAD_LINEAL )
        tiempo_lineal_y = abs((y_goal - y_anterior)/VELOCIDAD_LINEAL )
        tiempo_angular = abs((theta_goal - theta_anterior)/VELOCIDAD_ANGULAR) 
        
        # Actualizamos la última pose recibida
        self.ultima_pose = [x_goal, y_goal, abs(theta_goal)] #! Cambiar

        if tiempo_angular > (tiempo_lineal_y or tiempo_lineal_x):
            # self.get_logger().info("Moviendonos angularmente")
            # Significa que debemos enviar un mensaje de solo velocidad angular
            msg_to_send = [0, VELOCIDAD_ANGULAR, tiempo_angular]
        else:
            # self.get_logger().info("Moviendonos linealmente")

            if tiempo_lineal_x > tiempo_lineal_y:
                msg_to_send = [VELOCIDAD_LINEAL, 0, tiempo_lineal_x]
            else:
                msg_to_send = [VELOCIDAD_LINEAL, 0, tiempo_lineal_y]
        # Ya hasta aqui estamos en condiciones de indicarle al metodo para enviar la velocidad    
        # self.get_logger().info(f"Velocidad enviada con el tiempo {msg_to_send}")
        self.aplicar_velocidad(msg_to_send)

    def aplicar_velocidad(self, msg_to_send):
        """
        input: [v, 0, t] o [0,w,t] 
        Output: Debemos enviar el msg twist con cierta velocidad durante t segundos.
        """
        v_lineal, w_angular, tiempo = msg_to_send

        twist = Twist()
        # Aplicar velocidades lineales
        twist.linear.x = float(v_lineal)
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        # Aplicar velocidades angulares
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(w_angular)

        # Buscando el valor del tiempo de giro
        if twist.angular.z > 0.0:
            tiempo = 1.73 # se cae a la derehca
            tiempo = 1.74 # se cae a la izquierda
            tiempo = 1.735 # se cae a la izquierda
            tiempo = 1.7325 # se cae a la derecha
            tiempo = 1.734 # se cae a la izquierda
            tiempo = 1.7336 # se cae a la izquierda
            tiempo = 1.73355 # se cae a la izquierda
            tiempo = 1.7335 # se cae a la derecha
            tiempo = 1.73352 # se cae a la izquierda
            tiempo = 1.73351 # se cae a la derecha
            tiempo = 1.733515 # se cae a la izquierda
            tiempo = 1.733513 # se cae a la izquierda
            tiempo = 1.7335125 # se cae a la izquierda
            tiempo = 1.7335122 # se cae a la izquierda
            tiempo = 1.733512 # se cae a la derecha
            tiempo = 1.7335121 # se cae a la izquierda
            tiempo = 1.73351205 # se cae a la derecha
            tiempo = 1.73351209 # se cae a la izquierda
            tiempo = 1.73351205 # se cae a la derecha
            tiempo = 1.73351206 # se cae a la derecha
            tiempo = 1.73351207 # se cae a la izquierda
            tiempo = 1.733512065 # se cae a la derecha 
            tiempo = 1.733512067 # se cae a la izquierda
            tiempo = 1.733512066 # se cae a la derecha 
            tiempo = 1.7335120665 # se cae a la izquierda
            tiempo = 1.7335120662 # se cae a la izquierda
            tiempo = 1.73351206615 # se cae a la izquierda
            tiempo = 1.73351206613 # se cae a la izquierda
            tiempo = 1.73351206612 # se cae a la izquierda
            tiempo = 1.7335120661 # se cae a la derecha 
            tiempo = 1.733512066115 # se cae a la izquierda
            tiempo = 1.733512066112 # se cae a la izquierda
            tiempo = 1.733512066111 # se cae a la izquierda
            tiempo = 1.733512066110 # se cae a la derecha 
            tiempo = 1.7335120661105 # se cae a la izquierda
            tiempo = 1.7335120661102 # se cae a la izquierda
            tiempo = 1.7335120661101 # se cae a la izquierda
            tiempo = 1.73351206611005 # se cae a la izquierda
            tiempo = 1.73351206611001 # se cae a la izquierda


        # Comenzamos a contar el tiempo transcurrido
        tiempo_inicio = time.time()
        while (time.time() - tiempo_inicio) < tiempo:
            # Publicar el mensaje de velocidad
            # if twist.angular.z > 0.0:
                #self.get_logger().info(f"Tiempo {time.time() - tiempo_inicio}")

            self.publisher_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

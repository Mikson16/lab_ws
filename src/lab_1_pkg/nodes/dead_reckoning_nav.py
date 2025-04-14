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
from parametros import VELOCIDAD_LINEAL, VELOCIDAD_ANGULAR
import threading

class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav')

        #Suscripción a Pose_Loader
        self.subscription_array = self.create_subscription(PoseArray,'goal_list',self.accion_mover,10)


        #Suscripción a la odometría
        self.subscription_odom = self.create_subscription(Odometry,'/odom',self.read_odometry_callback,10)
        self.odometria_actual = None
        # Creamos un lock que permite escribir y leer una sola vez
        self.lock = threading.Lock() 
        self.timer_odom = self.create_timer(0.05, self.escribir_odometria)
        self.odometria_lectura = None
        
        #Publicador de velocidades
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel_mux/input/navigation',10)

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
            self.get_logger().info(f"Moviendo al objetivo {i}: {goal}")
            self.mover_robot_a_destino(goal)

        self.get_logger().info("Todos los objetivos fueron alcanzados.")

    def read_odometry_callback(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                    odom.pose.pose.orientation.y,
                                                    odom.pose.pose.orientation.z,
                                                    odom.pose.pose.orientation.w ) )
        self.odometria_lectura = (x, y, yaw)
        self.get_logger().info(f"La odometria es {self.odometria_lectura}.")

    def escribir_odometria(self):
        with self.lock:
                # Tomamos el lock para poder escribir
                self.odometria_actual = self.odometria_lectura
            

    def obtener_odometria(self):
        while self.odometria_actual != None:
            with self.lock:
                # Tomamos el lock para poder escribir
                return self.odometria_actual

    def mover_robot_a_destino(self, goal):
        """
        Input: posicion destino, odometria (enviada por el topico)
        Output: enviar la velocidad deseada y el tiempo de aplicacion al metodo aplicar_velocidad 
        """
        #Esperar hasta que tengamos odometría válida
        #! Para leer la odometria, descomentar esta linea
        # while self.odometria_actual == "":
        #     # Escucha el evento una vez
        #    rclpy.spin_once(self)
        # Separamos nuestras coordenadas
        x_goal, y_goal, theta_goal = goal
        self.get_logger().info(f"{goal}, vel_lin: {VELOCIDAD_LINEAL} vel_ang: {VELOCIDAD_ANGULAR}")

        # while rclpy.ok():
        
        
        if True:
            #print(self.obtener_odometria())
            x_odom, y_odom, theta_odom = self.obtener_odometria()
            # self.get_logger().info("Entre al while de la odometri")
            self.get_logger().info(f"La odometria indica {x_odom, y_odom, theta_odom}")


            tiempo_lineal_x = (x_goal - x_odom)/VELOCIDAD_LINEAL 
            tiempo_lineal_y = (y_goal - y_odom)/VELOCIDAD_LINEAL 
            
            tiempo_angular = (theta_goal - theta_odom)/VELOCIDAD_ANGULAR

            if tiempo_angular > (tiempo_lineal_y or tiempo_lineal_x):
                # Significa que debemos enviar un mensaje de solo velocidad angular
                msg_to_send = [0, VELOCIDAD_ANGULAR, tiempo_angular]
            else:
                if tiempo_lineal_x > tiempo_lineal_y:
                    msg_to_send = [VELOCIDAD_LINEAL, 0, tiempo_lineal_x]
                else:
                    msg_to_send = [VELOCIDAD_LINEAL, 0, tiempo_lineal_y]
            # Ya hasta aqui estamos en condiciones de indicarle al metodo para enviar la velocidad    
            self.get_logger().info(f"Se envio {msg_to_send}")
            self.aplicar_velocidad(msg_to_send)
            # break
            

            #self.publisher_cmd_vel.publish(twist)
            #rclpy.spin_once(self)
            #time.sleep(0.1)

        #Detener el robot
        #self.publisher_cmd_vel.publish(Twist())
        #time.sleep(0.5)

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

        tiempo_inicio = time.time()
        while (time.time() - tiempo_inicio) < tiempo:
            #self.get_logger().info(f"{time.time() - tiempo_inicio}")
            # Publicar el mensaje de velocidad
            self.publisher_cmd_vel.publish(twist)
            #self.get_logger().info(f"Aplicando velocidades: {v_lineal}, {w_angular} durante {tiempo} segundos")
        # Esperar el tiempo especificado

        # Detener el robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info(f"Robot detenido")


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

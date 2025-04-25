#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
# Importamos el mensaje segun el tipo que recibimos
from geometry_msgs.msg import PoseArray, Twist, Vector3
# Importamos el mensaje para leer la odometria
from nav_msgs.msg import Odometry
# Ocupamos euler from quaternion para convertir el mensaje enviado
from tf_transformations import euler_from_quaternion
import math
import time
from parametros import VELOCIDAD_LINEAL, VELOCIDAD_ANGULAR
import threading
import queue



class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav_obstacles')
        #Suscripción a Pose_Loader
        self.subscription_array = self.create_subscription(PoseArray,'goal_list',self.recibir_posiciones,10)
        # Cola de las posiciones FIFO
        self.cola_posiciones = queue.Queue()
        # Creamos un thread que sera el responsable de revisar las posiciones y ejecutar las velocidades designadsas
        self.hilo = threading.Thread(target=self.accion_mover)

        # Bandera para indicar si hay un obstaculo o no
        self.bandera_obstaculo = None
        
        #Publicador de velocidades
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel_mux/input/navigation',10)
        # Variable para guardar la pose anterior
        self.ultima_pose = None
        self.get_logger().info('Nodo dead_reckoning_nav_obstacle iniciado.')

        # Visualizacion de obstaculos por medio del mensaje publicado por obstacle detector
        self.subscription_obstacle = self.create_subscription(Vector3,'/ocupancy_state',self.accion_obstaculo,10)
        self.get_logger().info('Suscrito al topico de obstaculos.')
        
        #! Esto de abajho esta bien
        # Ejecutamos el hilo
        self.hilo.start()


    def recibir_posiciones(self, msg):
        """
        Este método recibe la lista de posiciones que envia pose_loader
        y las agrega a una queue FIFO
        """
        # self.get_logger().info(f"Recibidas las posiciones")
        
        for i, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            q = pose.orientation
            _, _, theta = euler_from_quaternion((q.x, q.y, q.z, q.w))

            goal = [x, y, theta]
            self.cola_posiciones.put(goal)
        
        

    def accion_mover(self):
        """
        Metodo ejecutado en el hilo paralelo al main. 
        Se encarga de revisar si hay posiciones a las cuales llegar 
        Llama a mover_robot_a_destino para 
        """
        while rclpy.ok():
            #self.get_logger().info(f"Estoy en el hilo paralelo")
            # self.get_logger().info(f"Estado bandera {self.bandera_obstaculo}")
            
            if not self.cola_posiciones.empty():
                goal = self.cola_posiciones.get()
                # Pasamos la posicion para que sea convertida en velocidad
                self.mover_robot_a_destino(goal)

    def accion_obstaculo(self, msg):
        """
        Input: msg del topico /ocupancy_state
        Output: Cambiar el estado de la bander e imprimir 
        """
        #self.get_logger().info(f"\nRecibido el mensaje de ocupacion: {msg}")
        if msg.x > 0.0 or msg.y > 0.0 or msg.z > 0.0:
            #self.get_logger().info(f"Estado de los obstaculos: Izquierda: {msg.x}, Centro: {msg.y}, Derecha: {msg.z}")
            if msg.x > 0.0:
                self.get_logger().info(f"Hay un obstaculo a la izquierda")
            if msg.y > 0.0:
                self.get_logger().info(f"Hay un obstaculo al centro papu")
            if msg.z > 0.0:
                self.get_logger().info(f"Hay un obstaculo a la derecha")
            self.publisher_cmd_vel.publish(Twist())
            
            self.bandera_obstaculo = True
            

        elif msg.x == 0.0 and msg.y == 0.0 and msg.z == 0.0:
            #self.get_logger().info("No hay un obstaculo")
            self.bandera_obstaculo = False
        


    def mover_robot_a_destino(self, goal):
        """
        Input: posicion destino, odometria (enviada por el topico)
        Output: enviar la velocidad deseada y el tiempo de aplicacion al metodo aplicar_velocidad 
        """
        if self.ultima_pose == None:
            self.ultima_pose = [0,0,0]
        x_anterior, y_anterior, theta_anterior = self.ultima_pose
        x_goal, y_goal, theta_goal = goal
        
        if theta_goal < 0: 
            # self.get_logger().info("Cambiamos el signo")
            # self.get_logger().info(f"    La tupla ultima fue {self.ultima_pose}")
            theta_goal = abs(theta_goal)

        tiempo_lineal_x = abs((x_goal - x_anterior)/VELOCIDAD_LINEAL )
        tiempo_lineal_y = abs((y_goal - y_anterior)/VELOCIDAD_LINEAL )
        tiempo_angular = abs((theta_goal - theta_anterior)/VELOCIDAD_ANGULAR) 
        
        # Actualizamos la última pose recibida
        self.ultima_pose = [x_goal, y_goal, abs(theta_goal)] 

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

        
        bandera_seguir = True
        ultimo_estado = False
        twist_blanco = Twist()
        while bandera_seguir:
            # self.get_logger().info(f"Estado bandera {self.bandera_obstaculo}")

            if self.bandera_obstaculo == False:
                # Si no hay obstaculos, podemos enviar la velocidad
                self.publisher_cmd_vel.publish(twist)
                if ultimo_estado == False:
                    # Marcamos el tiempo de inicio
                    tiempo_inicio = time.time()
                    ultimo_estado = True
                if time.time() - tiempo_inicio > tiempo:
                    bandera_seguir = False
            else:
                # En el caso en que existan obstaculos, enviamos velocidad de 0
                self.publisher_cmd_vel.publish(twist_blanco)
                if ultimo_estado == True:
                    # Esto solo lo ejecutamos cuando pasa de no haber un obst. a haber un obst.
                    ultimo_estado = False
                    tiempo_transcurrido = time.time() - tiempo_inicio
                    tiempo -= tiempo_transcurrido        

        
        

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

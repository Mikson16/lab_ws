#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
# Importamos el mensaje segun el tipo que recibimos
from geometry_msgs.msg import PoseArray, Twist
# Importamos el mensaje para leer la odometria
from nav_msgs.msg import Odometry
# Ocupamos euler from quaternion para convertir el mensaje enviado
from tf_transformations import euler_from_quaternion
# Creamos una cola para almacenar los msg
import queue
import time
from parametros import VELOCIDAD_LINEAL, VELOCIDAD_ANGULAR

class DeadReckoningNav(Node):

    def __init__(self):
        super().__init__('dead_reckoning_nav')
        #Suscripción a Pose_Loader
        self.subscription_array = self.create_subscription(PoseArray,'goal_list',self.accion_mover,10)
        #Suscripción a la odometría
        self.subscription_odom = self.create_subscription(Odometry,'/odom',self.lectura_odometria_callback,10)
        self.odometria_actual = None
        #Publicador de velocidades
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel_mux/input/navigation',10)
        # Cola de mensajes del tipo (x,y, tetha)
        self.cola_mensajes = queue.Queue() #FIFO
        self.get_logger().info('Nodo dead_reckoning_nav iniciado.')

    def accion_mover(self, msg):
        """
        Input: msg del topico goal_list recibe una lista de posiciones
        Output: append a la cola 
        """
        for i, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            q = pose.orientation
            _, _, theta = euler_from_quaternion((q.x, q.y, q.z, q.w))
            # Almacenamos la informacion importante
            goal = [x, y, theta]
            self.get_logger().info(f"Guardando el mensaje {i}: {goal}")
            # Lo guardamos en la cola
            self.cola_mensajes .put(goal)
        self.get_logger().info("Todos los objetivos fueron guardados.")


    def lectura_odometria_callback(self, odom):
        """
        Input: lectura de la odometria
        Output: enviar a convertir el msg en terminos de la velocidad
        """
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                    odom.pose.pose.orientation.y,
                                                    odom.pose.pose.orientation.z,
                                                    odom.pose.pose.orientation.w ) )
        # Almacenamos la odometria actual
        self.odometria_actual = [x, y, yaw]
        if self.cola_mensajes.empty() == False: # La cola tiene msg por revisar
            # self.get_logger().info(f"(roll, pitch, yaw): {roll, pitch, yaw}.")
            goal = self.cola_mensajes.get()
            self.get_logger().info(f"(x,y,tetha): {goal}.")
            # Enviamos a procesar el mensaje
            self.mover_robot_a_destino(goal)

                
    def mover_robot_a_destino(self, goal):
        """
        Input: posicion destino, odometria (almacenada en la variable)
        Output: enviar la velocidad deseada y el tiempo de aplicacion al metodo aplicar_velocidad 
        """
        x_goal, y_goal, theta_goal = goal
        x_odom, y_odom, theta_odom = self.odometria_actual
        self.get_logger().info(f"La odometria indica {x_odom, y_odom, theta_odom}")

        # Realizamos la conversion para la velocidad
        tiempo_lineal_x = (x_goal - x_odom)/VELOCIDAD_LINEAL 
        tiempo_lineal_y = (y_goal - y_odom)/VELOCIDAD_LINEAL 
        tiempo_angular = (theta_goal - theta_odom)/VELOCIDAD_ANGULAR
        # if tiempo_angular > (tiempo_lineal_y or tiempo_lineal_x):
        if (theta_goal - theta_odom) > ( (y_goal - y_odom) or ( x_goal - x_odom)) :
            # Significa que debemos enviar un mensaje de solo velocidad angular
            msg_to_send = [0, VELOCIDAD_ANGULAR, tiempo_angular]
        else:
            if tiempo_lineal_x > tiempo_lineal_y:
                msg_to_send = [VELOCIDAD_LINEAL, 0, tiempo_lineal_x]
            else:
                msg_to_send = [VELOCIDAD_LINEAL, 0, tiempo_lineal_y]
        # Ya hasta aqui estamos en condiciones de indicarle al metodo para enviar la velocidad por el topico  
        self.get_logger().info(f"(v,w,t): {msg_to_send}")
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

        # Esperamos el tiempo dado por el mensaje
        tiempo_inicio = time.time()
        while (time.time() - tiempo_inicio) < tiempo:
            self.publisher_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

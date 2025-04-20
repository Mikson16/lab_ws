#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#Agregue Twist para poder enviar velocidades al robot
from geometry_msgs.msg import Twist
#Agregue los eventos de parachoques para detectar colisiones
from kobuki_ros_interfaces.msg import BumperEvent
# sudo apt install ros-humble-kobuki-ros-interfaces


class NodoPublicador(Node):
    def __init__(self):
        super().__init__("nodo_publicador")

        # Nos suscribimos al topico para leer la tecla y la informacion del parachoque
        self.subscriber_key = self.create_subscription(String, 'tecla_presionada', self.recibir_tecla, 10)
        # Suscripción para detectar choques frontales
        self.create_subscription(BumperEvent, '/events/bumper', self.recibir_choque, 10)
        # Publicador para enviar mensajes de velocidad en formato Twist: Robot real
        #self.publisher_vel = self.create_publisher(Twist, '/commands/velocity', 10)
        # Publicador para enviar mensajes de velocidad en formato Twist: Simulacion
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel_mux/input/navigation', 10)

        # Variables de estado
        self.obstaculo = False 
        self.movimiento_actual = Twist() 
        # Timer para enviar_velocidad periódicamente (Elimine el metodo aplicar_velocidad(self))
        self.timer = self.create_timer(0.1, self.enviar_velocidad)

        self.get_logger().info("Nodo que realiza el movimiento inicializado")

    def recibir_tecla(self, msg):
        # Pasar todos los mensajes a minuscula xsi las moscas
        tecla = msg.data.lower()

        # Condición de que si existe un obstaculo no se pueda mover
        if self.obstaculo:
            self.get_logger().info("Aiuda, no me puedo mover, he chocao chaval :c")
            return

        # Eliminamos el mensaje anterior para evitar que tenga valores viejos
        twist = Twist()

        # Condicionales para saber que tecla es la recibida
        if tecla == "i":
            twist.linear.x = 0.2  # Adelane
        elif tecla == "j":
            twist.linear.x = -0.2  # Atrashhh
        elif tecla == "a":
            twist.angular.z = 1.0  # Giro izq
        elif tecla == "s":
            twist.angular.z = -1.0  # Giro der
        elif tecla == "q":
            twist.linear.x = 0.2
            twist.angular.z = 1.0  # Avanza y gira izq
        elif tecla == "w":
            twist.linear.x = 0.2
            twist.angular.z = -1.0  # Avanza y gira der
        elif tecla == "o":
            twist.linear.x = 0.0
            twist.angular.z = 0.0  # Avanza y gira der
        else:
            self.get_logger().info(f"Tecla no valida: {tecla}")
            return
        # Definimos el movimiento actual pa despues enviarlo
        self.movimiento_actual = twist 

    def recibir_choque(self, msg):
        # Ay choco el robot bobo (se presiono el bumper frontal)
        if msg.state == 1:
            self.get_logger().info("El bobo de Julian Alberto ha chocado")
            self.obstaculo = True
            # Reiniciamosel Twits
            self.movimiento_actual = Twist()
        # No hay obstaculos (el robot ya no está chocado)
        elif msg.state == 0:
            self.get_logger().info("Julian Alberto puede moverse libremente otra vez")
            self.obstaculo = False

    def enviar_velocidad(self):
        #self.get_logger().info(f'Estamos enviando: {self.movimiento_actual}')
        # Publicamo el movimiento B)
        self.publisher_vel.publish(self.movimiento_actual)

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoPublicador()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

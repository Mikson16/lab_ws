#!/usr/bin/env python3

from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import threading

class NodoPublicador(Node):

    def __init__(self):
        super().__init__("nodo_publicador")
        # Nos suscribimos al topico para leer la tecla y la informacion del parachoque
        self.subscriber_key = self.create_subscription(String, 'tecla_presionada', self.recibir_tecla, 10)
        
        # Variable que se actualiza segun si hay o no un obstaculo
        self.obstaculo = False
       
        self.movimiento = None

    def recibir_tecla(self, data): # Callback
        # Accedemos al atributo de la EDD
        tecla = data.data
        self.get_logger().info(f"La tecla recibida fue: {tecla}")
        self.manejador_eventos(tecla)
        
    
    def recibir_choque(self, data):
        # Metodo que recibe la informacion del choque y actualiza self.obstaculo
        pass

    def manejador_eventos(self, tecla):
        """
        Este metodo es activado al recibir una tecla 
        """
        if self.obstaculo == False:
            # Traducimos la tecla en la estructura valida
            if tecla == "w":
                self.movimiento = "Adelante"
            elif tecla == "s":
                self.movimiento = "atrash"
            # Creamos un thread para el envio de velocidad
            self.aplicar_velocidad()

    def aplicar_velocidad(self):
        """
        Este metodo crea el timer para enviar la velocidad
        """
        # Metodos 
        tiempo_periodo = 0.1 #segundos
        self.timer = self.create_timer(tiempo_periodo, self.enviar_velocidad)
        

    def enviar_velocidad(self):
        self.get_logger().info(f'Estamos enviando: {self.movimiento}')
        

def main(args=None):
    rclpy.init(args=args)
    node = NodoPublicador()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

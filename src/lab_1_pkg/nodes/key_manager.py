#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios
# Parte del codigo recuperado de:
# https://docs.python.org/es/3.13/library/termios.html

class KeyManager(Node):
    def __init__(self):
        super().__init__('key_manager')
        self.publisher_ = self.create_publisher(String, 'tecla_presionada', 10)
        self.get_logger().info('Presionar p para terminar')


    def leer_tecla(self):
        """
        Leemos la tecla presionada desde la terminal
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        """
        Entramos en bucle que ejecuta el metodo y
        publica por el topico la tecla presionada
        """
        while rclpy.ok():
            key = self.leer_tecla()
            msg = String()
            msg.data = key
            # Publicamos desde el topico 
            self.publisher_.publish(msg)
            self.get_logger().info(f'Tecla presionada: {key}')
            if key == 'p':
                self.get_logger().info('Terminando...')
                break

def main(args=None):
    rclpy.init(args=args)
    node = KeyManager()
    # Ejecutamos el metodo loop desde el main
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
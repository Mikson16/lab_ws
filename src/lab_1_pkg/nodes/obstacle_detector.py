#! /usr/bin/env python3

from turtle import width
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import cv2
from cv_bridge import CvBridge
import numpy as np
from parametros import UMBRAL_DE_DETECCION
from functions import get_distance


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        #suscripciones
        self.depth_image_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_image_callback, 10)

        #publicaciones
        """Luego verificar bien que es lo que voy a enviar"""
        self.obstacle_pub = self.create_publisher(Vector3, '/ocupancy_state', 10)


        #cv_bridge
        self.bridge = CvBridge()
        self.current_cv_depth_image = None

    def depth_image_callback(self, msg):
        try:
            self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') # Para mantener el tipo de datos, luego saber si vienen en milimetros o metros, esto retorna una matriz, si es en profundidad, dara una matriz 2D con la profundidad en la codificacion del sensor, donde cada elemento de la matriz es un pixel de la imagen#

            if msg.encoding == '16UC1':
                self.current_cv_depth_image = self.current_cv_depth_image.astype(np.float32) / 1000.0  # Convertir a metros, en caso de que venga en milimetros

            # Separar la matriz en regiones
            height, width = self.current_cv_depth_image.shape
            
            izq = self.current_cv_depth_image[:, :width//3]  # Izquierda
            centro = self.current_cv_depth_image[:, width//3:2*width//3]  # Centro
            der = self.current_cv_depth_image[:, 2*width//3:]

            #Recorrer todas las matrices y si tan solo una celda rompe el limite entonces hay un obstaculo en esa region
            obstacle_state_izq = float(get_distance(izq, UMBRAL_DE_DETECCION))
            obstacle_state_centro = float(get_distance(centro, UMBRAL_DE_DETECCION))
            obstacle_state_der = float(get_distance(der, UMBRAL_DE_DETECCION))

            # Crear un mensaje de ocupación
            occupancy_state = Vector3()
            occupancy_state.x = obstacle_state_izq
            occupancy_state.y = obstacle_state_centro
            occupancy_state.z = obstacle_state_der
            
            # Publicar el mensaje de ocupación
            # self.get_logger().info(f'{occupancy_state}')

            self.obstacle_pub.publish(occupancy_state)


        except Exception as e:
            self.get_logger().error(f'Error al obtener profundidad u obtener los datos: {e}')


def main(args=None):
    rclpy.init(args=args)

    obstacle_detector = ObstacleDetector()

    rclpy.spin(obstacle_detector)

    # Destroy the node explicitly
    obstacle_detector.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import cv2
from cv_bridge import CvBridge
import numpy as np


class ObstacleDetector(Node):
    def __init__(self):
        super()._init__('obstacle_detector')

        #suscripciones
        self.depth_image_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_image_callback, 10)

        #publicaciones
        """Luego verificar bien que es lo que voy a enviar"""
        self.obstacle_pub = self.create_publisher(Image, '/ocupancy_state', 10)


        #cv_bridge
        self.bridge = CvBridge()
        self.current_cv_depth_image = None

    def depth_image_callback(self, msg):
        try:
            self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') # Para mantener el tipo de datos, luego saber si vienen en milimetros o metros, esto retorna una matriz, si es en profundidad, dara una matriz 2D con la profundidad en la codificacion del sensor, donde cada elemento de la matriz es un pixel de la imagen#

            if msg.encoding == '16UC1':
                self.current_cv_depth_image = self.current_cv_depth_image.astype(np.float32) / 1000.0  # Convertir a metros, en caso de que venga en milimetros

            #!TODO Separar la imagen y medir la distancia entre el robot y el objeto

        except Exception as e:
            self.get_logger().error(f'Error al obtener profundidad u obtener los datos: {e}')



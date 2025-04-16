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
        pass

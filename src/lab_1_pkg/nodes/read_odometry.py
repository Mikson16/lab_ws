#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

class OdometryReader( Node ):
    def __init__( self ):
        super().__init__( 'odom_reader_node' )
        self.odom_sub = self.create_subscription( Odometry, '/odom', self.odometry_cb, 10 )

    def odometry_cb( self, odom ):
        print("Leyendo")
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                                                odom.pose.pose.orientation.y,
                                                                                odom.pose.pose.orientation.z,
                                                                                odom.pose.pose.orientation.w ) )
        self.get_logger().info( 'Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw)) 

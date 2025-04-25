#! /usr/bin/env python3
# Importamos el mensaje para leer la odometria
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

class OdometryReader( Node ):
    def __init__( self ):
        super().__init__( 'odom_reader_node' )
        self.odom_sub = self.create_subscription( Odometry, '/odom', self.odometry_cb, 10 )
        self.lista_poses = []
        self.ultima_pose = None
        self.contador = 0
        self.bandera_ploteado = False


    
    def odometry_cb( self, odom ):
        # self.get_logger().info("Leyendo")
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        self.contador +=1
        # self.get_logger().info(f"Contando {self.contador}")

        info = [x,y]
        # self.get_logger().info(f'Pose actual {info}') 
        if self.contador > 150 and self.bandera_ploteado == False:
            self.bandera_ploteado = True
            # Si registramos la misma pose, terminamos de avanzar
            self.graficar() 
            pass
        else:
            self.ultima_pose = info
            self.lista_poses.append(info)
            

    def graficar(self):
        x = []
        y = []

        for i in range(len(self.lista_poses)):
            x.append(round(self.lista_poses[i][0], 3))
            y.append(round(self.lista_poses[i][1], 3))
         
        plt.plot(x, y)

        # Agregar etiquetas y título
        plt.xlabel("Eje X")
        plt.ylabel("Eje Y")
        plt.title(" Grafico de la odometria")

        # Mostrar el gráfico
        plt.show()
            
def main(args=None):
    rclpy.init(args=args)
    node = OdometryReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

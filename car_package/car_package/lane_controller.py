import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')
        
        # Suscriptor a la cámara de carretera
        self.subscription = self.create_subscription(
            Image,
            '/road_camera',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Lane Controller iniciado')
    
    def image_callback(self, msg):
        try:
            # Convertir imagen ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Procesar según el algoritmo de la práctica
            # 1. Imagen 512x16x3
            # 2. Media de cada columna para los 3 canales
            column_means = np.mean(cv_image, axis=(0, 2))  # Vector de 512 valores
            
            # 3. Encontrar máximo (centro de la carretera)
            road_center = np.argmax(column_means)
            image_center = 256  # Centro de imagen de 512px
            
            # Calcular error para control
            error = image_center - road_center
            
            self.get_logger().info(f'Centro carretera: {road_center}, Error: {error}')
            
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
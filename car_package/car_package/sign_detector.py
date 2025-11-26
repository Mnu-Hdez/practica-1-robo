import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')
        
        # Suscriptor a la cámara frontal
        self.subscription = self.create_subscription(
            Image,
            '/car_camera',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.threshold = 0.7
        
        self.get_logger().info('Sign Detector iniciado')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Aquí irá la lógica de detección con matchTemplate
            # Por ahora solo log para verificar que recibe imágenes
            self.get_logger().info('Imagen recibida de car_camera')
            
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
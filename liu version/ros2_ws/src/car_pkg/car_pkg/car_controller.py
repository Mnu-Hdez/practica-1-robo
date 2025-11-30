#!/usr/bin/env python3
"""
Nodo controlador que actúa como puente entre Webots (usando controller API)
y ROS 2 (publicando las cámaras y recibiendo comandos de motor).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import numpy as np

# Importar la API de Webots
try:
    from controller import Robot
    WEBOTS_AVAILABLE = True
except ImportError:
    print("ERROR: No se pudo importar 'controller' de Webots")
    WEBOTS_AVAILABLE = False


class CarControllerNode(Node):
    """
    Nodo que conecta Webots con ROS 2.
    - Publica las imágenes de las cámaras
    - Se suscribe a comandos de velocidad para los motores
    """
    
    def __init__(self):
        super().__init__('car_controller_node')
        
        if not WEBOTS_AVAILABLE:
            self.get_logger().error('No se pudo importar el módulo Robot de Webots')
            return
        
        # Inicializar el robot de Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Parámetros del vehículo
        self.MAX_SPEED = 20.0  # rad/s
        
        # Obtener los motores
        self.left_motor = self.robot.getDevice('left_rear_wheel')
        self.right_motor = self.robot.getDevice('right_rear_wheel')
        
        if self.left_motor is None or self.right_motor is None:
            self.get_logger().error('ERROR: No se encontraron los motores')
            return
        
        # Configurar motores en modo velocidad
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        self.get_logger().info('✓ Motores inicializados correctamente')
        
        # Obtener cámaras
        self.car_camera = self.robot.getDevice('car_camera')
        self.road_camera = self.robot.getDevice('road_camera')
        
        if self.car_camera:
            self.car_camera.enable(self.timestep)
            self.get_logger().info('✓ car_camera habilitada')
        
        if self.road_camera:
            self.road_camera.enable(self.timestep)
            self.get_logger().info('✓ road_camera habilitada')
        
        # Bridge de OpenCV
        self.bridge = CvBridge()
        
        # Publicadores de las cámaras
        self.car_camera_pub = self.create_publisher(
            Image,
            '/car/car_camera/image_raw',
            10
        )
        
        self.road_camera_pub = self.create_publisher(
            Image,
            '/car/road_camera/image_raw',
            10
        )
        
        # Suscriptores a comandos de motor
        self.left_wheel_sub = self.create_subscription(
            Float64,
            '/car/left_wheel',
            self.left_wheel_callback,
            10
        )
        
        self.right_wheel_sub = self.create_subscription(
            Float64,
            '/car/right_wheel',
            self.right_wheel_callback,
            10
        )
        
        # Timer para step del robot y publicación de imágenes
        self.create_timer(self.timestep / 1000.0, self.step_callback)
        
        self.get_logger().info('Car Controller listo.')
    
    def left_wheel_callback(self, msg):
        """Recibe comando de velocidad para rueda izquierda."""
        if self.left_motor is None:
            return
        velocity = max(-self.MAX_SPEED, min(self.MAX_SPEED, msg.data))
        self.left_motor.setVelocity(velocity)
    
    def right_wheel_callback(self, msg):
        """Recibe comando de velocidad para rueda derecha."""
        if self.right_motor is None:
            return
        velocity = max(-self.MAX_SPEED, min(self.MAX_SPEED, msg.data))
        self.right_motor.setVelocity(velocity)
    
    def publish_camera_image(self, camera, publisher, camera_name):
        """Publica la imagen de una cámara."""
        if camera is None:
            return
        
        # Obtener imagen de Webots
        image_data = camera.getImage()
        if image_data is None:
            return
        
        # Convertir a numpy array
        width = camera.getWidth()
        height = camera.getHeight()
        
        # La imagen viene en formato BGRA
        image_array = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
        
        # Convertir BGRA a BGR (eliminar canal alpha)
        image_bgr = image_array[:, :, :3]
        
        try:
            # Convertir a mensaje ROS
            msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = camera_name
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publicando imagen de {camera_name}: {e}')
    
    def step_callback(self):
        """Ejecuta un paso de simulación y publica las imágenes."""
        if self.robot.step(self.timestep) == -1:
            self.get_logger().warn('Simulación terminada')
            rclpy.shutdown()
            return
        
        # Publicar imágenes de las cámaras
        self.publish_camera_image(self.car_camera, self.car_camera_pub, 'car_camera')
        self.publish_camera_image(self.road_camera, self.road_camera_pub, 'road_camera')


def main(args=None):
    rclpy.init(args=args)
    
    if not WEBOTS_AVAILABLE:
        print("ERROR: No se puede ejecutar sin la API de Webots")
        return
    
    try:
        controller = CarControllerNode()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
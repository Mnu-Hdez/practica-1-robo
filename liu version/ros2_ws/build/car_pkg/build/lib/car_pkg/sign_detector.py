#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from ament_index_python.packages import get_package_share_directory

class SignDetector(Node):

    def __init__(self):
        super().__init__('sign_detector_node')
        
        # --- Parámetros ---
        self.bridge = CvBridge()
        self.TEMPLATE_THRESHOLD = 0.7  # Umbral de confianza
        self.current_signal = None
        self.last_signal_time = self.get_clock().now()
        
        # Velocidad máxima actual (para calcular mitad en YIELD)
        self.current_max_speed = 65.0  # Velocidad por defecto

        # --- Publicador ---
        self.publisher_ = self.create_publisher(Float64, '/car/max_speed', 10)

        # --- Suscriptor a la cámara principal ---
        self.subscription = self.create_subscription(
            Image,
            '/car/car_camera/image_raw',
            self.camera_callback,
            10)
        
        # --- Cargar plantillas ---
        try:
            pkg_share = get_package_share_directory('car_pkg')
            resource_path = os.path.join(pkg_share, 'resource')
            
            stop_path = os.path.join(resource_path, 'stop.png')
            yield_path = os.path.join(resource_path, 'yield.png')
            speed_55_path = os.path.join(resource_path, 'speed_55.jpg')
            speed_65_path = os.path.join(resource_path, 'speed_65.png')

            self.template_stop = cv2.imread(stop_path, cv2.IMREAD_COLOR)
            self.template_yield = cv2.imread(yield_path, cv2.IMREAD_COLOR)
            self.template_speed_55 = cv2.imread(speed_55_path, cv2.IMREAD_COLOR)
            self.template_speed_65 = cv2.imread(speed_65_path, cv2.IMREAD_COLOR)

            if self.template_stop is None:
                self.get_logger().error(f'No se pudo cargar: {stop_path}')
            
            self.templates = {
                "SPEED_55": self.template_speed_55,
                "SPEED_65": self.template_speed_65,
                "STOP": self.template_stop,
                "YIELD": self.template_yield
            }
            self.get_logger().info('Plantillas de señales cargadas correctamente.')

        except Exception as e:
            self.get_logger().error(f'Error al cargar plantillas: {e}')
            self.templates = {}


    def camera_callback(self, msg):
        """Callback que procesa la imagen de la cámara frontal."""
        if not self.templates:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')
            return

        detected_signal = None
        
        for name, template in self.templates.items():
            if template is None:
                continue
                
            if template.shape[0] > cv_image.shape[0] or template.shape[1] > cv_image.shape[1]:
                continue

            res = cv2.matchTemplate(cv_image, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(res)
            
            if max_val > self.TEMPLATE_THRESHOLD:
                detected_signal = name
                break

        if detected_signal:
            now = self.get_clock().now()
            if detected_signal != self.current_signal or (now - self.last_signal_time).nanoseconds > 5e9:
                self.current_signal = detected_signal
                self.last_signal_time = now
                self.process_signal(detected_signal)

    def process_signal(self, signal_name):
        """Publica la acción correspondiente a la señal detectada."""
        msg = Float64()
        
        if signal_name == "STOP":
            self.get_logger().info(f'Acción: {signal_name}')
            msg.data = 0.0  # STOP
            
        elif signal_name == "YIELD":
            # CORRECCIÓN: Mitad de la velocidad máxima ACTUAL
            new_speed = self.current_max_speed / 2.0
            self.get_logger().info(f'Acción: {signal_name} - Reduciendo a {new_speed} km/h (mitad de {self.current_max_speed})')
            msg.data = new_speed
            
        elif signal_name == "SPEED_55":
            self.get_logger().info(f'Acción: {signal_name} (55 km/h)')
            msg.data = 55.0
            self.current_max_speed = 55.0  # Actualizar velocidad máxima

        elif signal_name == "SPEED_65":
            self.get_logger().info(f'Acción: {signal_name} (65 km/h)')
            msg.data = 65.0
            self.current_max_speed = 65.0  # Actualizar velocidad máxima

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    sign_detector = SignDetector()
    rclpy.spin(sign_detector)
    
    sign_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
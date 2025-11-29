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
        self.TEMPLATE_THRESHOLD = 0.7  # Umbral de confianza (¡Ajustar!)
        self.current_signal = None
        self.last_signal_time = self.get_clock().now()

        # --- Publicador ---
        # Publica la velocidad máxima que el 'lane_controller' debe usar
        self.publisher_ = self.create_publisher(Float64, '/car/max_speed', 10)

        # --- Suscriptor ---
        # Suscriptor a la cámara principal (car_camera)
        self.subscription = self.create_subscription(
            Image,
            '/car/car_camera/image_raw',
            self.camera_callback,
            10)
        
        # --- Cargar plantillas (Templates) ---
        try:
            pkg_share = get_package_share_directory('car_pkg')
            resource_path = os.path.join(pkg_share, 'resource')
            
            # Cargar las plantillas que tengas en la carpeta 'resource'
            stop_path = os.path.join(resource_path, 'stop.png')
            yield_path = os.path.join(resource_path, 'yield.png')
            
            # <-- ¡CAMBIO! Cargamos las dos plantillas de velocidad
            speed_55_path = os.path.join(resource_path, 'speed_55.jpg')
            speed_65_path = os.path.join(resource_path, 'speed_65.png')


            self.template_stop = cv2.imread(stop_path, cv2.IMREAD_COLOR)
            self.template_yield = cv2.imread(yield_path, cv2.IMREAD_COLOR)
            
            # <-- ¡CAMBIO! Leemos las nuevas plantillas
            self.template_speed_55 = cv2.imread(speed_55_path, cv2.IMREAD_COLOR)
            self.template_speed_65 = cv2.imread(speed_65_path, cv2.IMREAD_COLOR)

            if self.template_stop is None:
                self.get_logger().error(f'No se pudo cargar: {stop_path}')
            
            # <-- ¡CAMBIO! Diccionario de plantillas actualizado
            # El orden puede ser importante si una plantilla está contenida en otra.
            # Aquí asumimos que son lo suficientemente distintas.
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
            return # No hacer nada si no hay plantillas
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')
            return

        # Bucle por todas las plantillas para ver si hay coincidencia
        detected_signal = None
        
        # NOTA: matchTemplate es sensible al tamaño.
        # Para un sistema real, se re-escala la imagen o la plantilla.
        # Aquí, por simplicidad, solo usamos el tamaño original.
        
        for name, template in self.templates.items():
            if template is None:
                continue
                
            # Comprobar si la plantilla cabe en la imagen
            if template.shape[0] > cv_image.shape[0] or template.shape[1] > cv_image.shape[1]:
                continue

            # Búsqueda por plantilla
            res = cv2.matchTemplate(cv_image, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(res)
            
            if max_val > self.TEMPLATE_THRESHOLD:
                # ¡Señal detectada!
                detected_signal = name
                break # Encontramos una, dejamos de buscar

        if detected_signal:
            # Evitar publicar la misma señal una y otra vez
            # Solo publicamos si es una señal nueva O si han pasado 5s
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
            msg.data = 0.0  # El lane_controller interpretará 0.0 como STOP
            
        elif signal_name == "YIELD":
            self.get_logger().info(f'Acción: {signal_name}')
            # (Asumimos 65 / 2 = 32.5) -> El PDF dice "mitad de la vel. máxima"
            # Esto es ambiguo, así que elegimos un valor fijo (ej. 30)
            msg.data = 30.0
            
        # <-- ¡CAMBIO! Manejamos las dos señales de velocidad
        elif signal_name == "SPEED_55":
            self.get_logger().info(f'Acción: {signal_name} (55)')
            msg.data = 55.0

        elif signal_name == "SPEED_65":
            self.get_logger().info(f'Acción: {signal_name} (65)')
            msg.data = 65.0 # Asumiendo que es la señal de 65

        # Publicar el comando de velocidad
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    sign_detector = SignDetector()
    rclpy.spin(sign_detector)
    
    sign_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
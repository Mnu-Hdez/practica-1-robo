#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class LaneController(Node):

    def __init__(self):
        super().__init__('lane_controller_node')
        
        # --- Parámetros del Vehículo ---
        self.WHEEL_RADIUS = 0.331  # metros
        self.WHEEL_DISTANCE = 1.628  # metros
        self.MAX_SPEED = 20.0  # rad/s
        
        # --- Parámetros del Controlador (AJUSTADOS) ---
        self.BASE_SPEED = 12.0  # rad/s - Velocidad base (reducida para mejor control)
        self.KP = 0.045  # Ganancia proporcional (AUMENTADA para curvas cerradas)
        self.MAX_TURN_DIFF = 11.0  # Máxima diferencia de velocidad entre ruedas
        
        self.TARGET_SPEED_KMH = 30.0
        self.current_max_speed_kmh = self.TARGET_SPEED_KMH
        self.stop_mode = False
        self.stop_timer = None
        
        # Variables para suavizado
        self.prev_error = 0.0
        self.error_smoothing = 0.65  # Factor de suavizado (aumentado para estabilidad)
        
        self.frame_count = 0

        # --- Herramientas ---
        self.bridge = CvBridge()

        # --- Publicadores ---
        self.left_motor_pub = self.create_publisher(Float64, '/car/left_wheel', 10)
        self.right_motor_pub = self.create_publisher(Float64, '/car/right_wheel', 10)

        # --- Suscriptores ---
        self.subscription_camera = self.create_subscription(
            Image,
            '/car/road_camera/image_raw',
            self.camera_callback,
            10)
        
        self.subscription_speed = self.create_subscription(
            Float64,
            '/car/max_speed',
            self.speed_callback,
            10)
        
        self.get_logger().info('Nodo "lane_controller" iniciado.')

    def speed_callback(self, msg):
        """Callback para cambios de velocidad por señales."""
        new_speed = msg.data
        
        if new_speed == 0.0:
            if not self.stop_mode:
                self.get_logger().info('¡Recibida orden de STOP!')
                self.current_max_speed_kmh = self.TARGET_SPEED_KMH
                self.TARGET_SPEED_KMH = 0.0
                self.stop_mode = True
                
                if self.stop_timer:
                    self.stop_timer.cancel()
                self.stop_timer = self.create_timer(1.0, self.resume_from_stop)
        elif self.stop_mode:
            pass
        else:
            self.get_logger().info(f'Nueva velocidad máxima: {new_speed} km/h')
            self.TARGET_SPEED_KMH = new_speed
            self.current_max_speed_kmh = new_speed

    def resume_from_stop(self):
        """Reanudar después de STOP."""
        self.get_logger().info('Reanudando marcha...')
        self.TARGET_SPEED_KMH = self.current_max_speed_kmh
        self.stop_mode = False
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

    def camera_callback(self, msg):
        """Callback principal de procesamiento."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')
            return

        # 1. Procesar imagen
        mean_vector = np.mean(cv_image, axis=0)
        line_vector = np.mean(mean_vector, axis=1)
        
        max_value = np.max(line_vector)
        
        if max_value < 50:
            # No detecta línea - mantener dirección recta
            error_raw = 0.0
            line_detected = False
        else:
            centro_detectado = np.argmax(line_vector)
            centro_ideal = msg.width / 2
            error_raw = centro_ideal - centro_detectado
            line_detected = True

        # 2. Suavizar el error (filtro exponencial)
        error = self.error_smoothing * self.prev_error + (1 - self.error_smoothing) * error_raw
        self.prev_error = error

        # 3. Calcular ajuste de velocidad (proporcional al error)
        turn_adjustment = self.KP * error
        
        # Limitar el ajuste máximo
        turn_adjustment = max(-self.MAX_TURN_DIFF, min(self.MAX_TURN_DIFF, turn_adjustment))

        # 4. Calcular velocidad base según TARGET_SPEED
        velocity_factor = self.TARGET_SPEED_KMH / 30.0
        base_speed = self.BASE_SPEED * velocity_factor
        
        # 5. Aplicar diferencial a las ruedas
        # La rueda interior gira más lento, la exterior más rápido
        left_speed = base_speed + turn_adjustment
        right_speed = base_speed - turn_adjustment
        
        # 6. Limitar velocidades
        left_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, left_speed))
        right_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, right_speed))
        
        # DEBUG
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Max:{max_value:.0f} | Err_raw:{error_raw:.0f} Err_suav:{error:.1f} | '
                f'Ajuste:{turn_adjustment:.2f} | L:{left_speed:.1f} R:{right_speed:.1f}'
            )
        
        # 7. Publicar
        left_msg = Float64()
        left_msg.data = left_speed
        self.left_motor_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = right_speed
        self.right_motor_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    lane_controller = LaneController()
    rclpy.spin(lane_controller)
    
    lane_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
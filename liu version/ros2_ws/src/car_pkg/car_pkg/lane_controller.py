#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import time
from collections import deque

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller_node')

        # --- Parámetros vehículo ---
        self.MAX_SPEED = 20.0          # rad/s máximo físico
        self.BASE_SPEED = 10.0         # velocidad base
        self.KP = 0.5                  # ganancia proporcional
        self.KD = 0.1                  # ganancia derivada
        self.MAX_TURN_DIFF = 8.0       # diferencia máxima entre ruedas

        # suavizado de salidas
        self.left_buffer = deque(maxlen=5)
        self.right_buffer = deque(maxlen=5)

        # variables internas PD
        self.prev_error = 0.0
        self.prev_time = time.time()

        # herramienta OpenCV
        self.bridge = CvBridge()

        # publishers a ruedas
        self.left_motor_pub = self.create_publisher(Float64, '/car/left_wheel', 10)
        self.right_motor_pub = self.create_publisher(Float64, '/car/right_wheel', 10)

        # subscriber a cámara
        self.subscription_camera = self.create_subscription(
            Image,
            '/car/road_camera/image_raw',
            self.camera_callback,
            10)

        self.get_logger().info('Nodo "lane_controller" inicializado y listo.')

    def camera_callback(self, msg):
        """Procesamiento de imagen y cálculo de velocidades."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')
            return

        # imagen completa, convertir a gris
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # media por columna -> vector 1D
        col_mean = gray.mean(axis=0)
        col_mean = cv2.GaussianBlur(col_mean.reshape(1, -1), (1, 5), 0).reshape(-1)

        # buscar máximo
        max_val = np.max(col_mean)
        if max_val < 10:  # línea débil o no visible
            line_pos = None
        else:
            line_pos = int(np.argmax(col_mean))

        center_ideal = w / 2.0

        # calcular error normalizado [-1,1]
        if line_pos is None:
            error_norm = 0.0
        else:
            error_norm = (line_pos - center_ideal) / (w / 2.0)

        # PD
        now = time.time()
        dt = max(1e-3, now - self.prev_time)
        derivative = (error_norm - self.prev_error) / dt
        self.prev_error = error_norm
        self.prev_time = now

        turn = self.KP * error_norm + self.KD * derivative
        turn = np.clip(turn, -1.0, 1.0)

        # ajustar velocidad según error lateral
        speed_factor = max(0.3, 1.0 - abs(error_norm))
        base_speed = self.BASE_SPEED * speed_factor

        left_speed = base_speed + turn * self.MAX_TURN_DIFF
        right_speed = base_speed - turn * self.MAX_TURN_DIFF

        # limitar velocidades
        left_speed = np.clip(left_speed, -self.MAX_SPEED, self.MAX_SPEED)
        right_speed = np.clip(right_speed, -self.MAX_SPEED, self.MAX_SPEED)

        # suavizado
        self.left_buffer.append(left_speed)
        self.right_buffer.append(right_speed)
        out_left = np.mean(self.left_buffer)
        out_right = np.mean(self.right_buffer)

        # publicar
        self.left_motor_pub.publish(Float64(data=float(out_left)))
        self.right_motor_pub.publish(Float64(data=float(out_right)))

        # debug cada 10 frames
        if np.random.rand() < 0.1:
            self.get_logger().info(f'Err:{error_norm:.2f} | L:{out_left:.2f} R:{out_right:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


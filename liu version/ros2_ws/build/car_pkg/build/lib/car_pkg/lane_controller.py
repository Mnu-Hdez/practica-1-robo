
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
        self.MAX_SPEED = 44.0
        self.BASE_SPEED = 20.0
        self.KD = 9.0
        self.MAX_TURN_DIFF = 40.0

        # KP dinámico
        self.KP_BASE = 0.45
        self.KP_MAX = 1.1

        # Buffers para suavizar velocidades
        self.left_buffer = deque(maxlen=5)
        self.right_buffer = deque(maxlen=5)

        # Buffers para suavizar error
        self.error_buffer = deque(maxlen=3)
        self.prev_time = time.time()

        self.bridge = CvBridge()

        # Publicadores
        self.left_motor_pub = self.create_publisher(Float64, '/car/left_wheel', 10)
        self.right_motor_pub = self.create_publisher(Float64, '/car/right_wheel', 10)

        # Subscriptor cámara
        self.create_subscription(Image, '/car/road_camera/image_raw', self.camera_callback, 10)

        self.get_logger().info("Lane controller optimizado ACTIVADO.")

    # --------------------------------------------------------------
    # Función para obtener la posición de la línea
    # --------------------------------------------------------------
    def get_line_pos(self, gray, top, bottom):
        """Devuelve x del centro de la línea más brillante en la región [top:bottom]."""
        region = gray[top:bottom, :]
        col_mean = region.mean(axis=0)
        col_mean = cv2.GaussianBlur(col_mean.reshape(1, -1), (1, 11), 0).reshape(-1)

        # Filtro de confianza: ignorar picos muy bajos o cortos (paso de cebra)
        max_val = np.max(col_mean)
        if max_val < 20:  # línea débil → ignorar
            return None, 0.0

        # confianza: normalizada 0..1
        confidence = max_val / 255.0
        line_x = int(np.argmax(col_mean))
        return line_x, confidence

    # --------------------------------------------------------------
    # Callback principal
    # --------------------------------------------------------------
    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        center = w / 2

        # ---------------------------
        # 1. Línea cercana (reacción inmediata)
        # ---------------------------
        near_top = int(h * 0.60)
        near_bottom = h
        line_near, conf_near = self.get_line_pos(gray, near_top, near_bottom)
        error_near = ((line_near - center) / (w / 2)) if line_near is not None else 0.0

        # ---------------------------
        # 2. Línea lejana (predicción curva)
        # ---------------------------
        far_top = int(h * 0.30)
        far_bottom = int(h * 0.55)
        line_far, conf_far = self.get_line_pos(gray, far_top, far_bottom)
        error_far = ((line_far - center) / (w / 2)) if line_far is not None else 0.0

        # ---------------------------
        # 3. Suavizado de error
        # ---------------------------
        self.error_buffer.append(error_near)
        smooth_error = np.mean(self.error_buffer)

        # ---------------------------
        # 4. Predicción de curva
        # ---------------------------
        curvature = error_far - error_near if line_far is not None else 0.0

        # ---------------------------
        # 5. KP dinámico según confianza y curva
        # ---------------------------
        KP = self.KP_BASE + abs(curvature) * 1.5
        # Ajuste según confianza de la línea cercana
        KP *= conf_near
        KP = np.clip(KP, self.KP_BASE, self.KP_MAX)

        # ---------------------------
        # 6. Control PD
        # ---------------------------
        dt = max(1e-3, time.time() - self.prev_time)
        derivative = (smooth_error - getattr(self, 'prev_error', 0.0)) / dt
        self.prev_error = smooth_error
        self.prev_time = time.time()

        turn = KP * smooth_error + self.KD * derivative
        turn = np.clip(turn, -1.0, 1.0)

        # ---------------------------
        # 7. Ajuste de velocidad según curva
        # ---------------------------
        curve_strength = min(1.0, abs(curvature) * 2.0)
        speed_factor = 1.0 - curve_strength * 0.5
        base_speed = self.BASE_SPEED * speed_factor

        left_speed = base_speed + turn * self.MAX_TURN_DIFF
        right_speed = base_speed - turn * self.MAX_TURN_DIFF

        # Limitar velocidades
        left_speed = np.clip(left_speed, -self.MAX_SPEED, self.MAX_SPEED)
        right_speed = np.clip(right_speed, -self.MAX_SPEED, self.MAX_SPEED)

        # Suavizado
        self.left_buffer.append(left_speed)
        self.right_buffer.append(right_speed)
        out_left = float(np.mean(self.left_buffer))
        out_right = float(np.mean(self.right_buffer))

        # Publicar
        self.left_motor_pub.publish(Float64(data=out_left))
        self.right_motor_pub.publish(Float64(data=out_right))

        # Debug
        if np.random.rand() < 0.1:
            self.get_logger().info(
                f"Near:{error_near:.2f} Far:{error_far:.2f} Curv:{curvature:.2f} "
                f"KP:{KP:.2f} Conf:{conf_near:.2f} L:{out_left:.2f} R:{out_right:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()


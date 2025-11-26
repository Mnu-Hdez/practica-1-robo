import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class CarDriver(Node):
    def __init__(self):
        super().__init__('car_driver')
        
        # Publicadores para controlar las ruedas
        self.left_motor_pub = self.create_publisher(Float64, '/left_front_motor', 10)
        self.right_motor_pub = self.create_publisher(Float64, '/right_front_motor', 10)
        
        self.get_logger().info('Car Driver iniciado')
        
        # Control básico inicial
        self.control_timer = self.create_timer(0.1, self.control_loop)
    
    def control_loop(self):
        # Ejemplo básico: mover hacia adelante
        speed_msg = Float64()
        speed_msg.data = 1.0  # Velocidad inicial
        
        self.left_motor_pub.publish(speed_msg)
        self.right_motor_publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(Float32, 'battery_status', 10)
        self.timer = self.create_timer(1.0, self.publish_voltage)
        self.voltage = 25.2

    def publish_voltage(self):
        msg = Float32()
        msg.data = self.voltage
        self.publisher_.publish(msg)
        self.get_logger().info(f'Pil gerilimi: {self.voltage:.2f}V')

def main():
    rclpy.init()
    node = BatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

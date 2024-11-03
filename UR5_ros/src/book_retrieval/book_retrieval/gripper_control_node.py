import rclpy
from rclpy.node import Node
from book_retrieval.msg import BookInfo
import serial
import time

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        self.command_subscriber = self.create_subscription(BookInfo, 'book_info', self.command_callback, 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)  # Update serial port as needed
        time.sleep(2)

    def command_callback(self, msg):
        width = msg.width
        self.get_logger().info(f"Received width for gripping: {width}")
        self.send_grip_command(width)

    def send_grip_command(self, width):
        width_value = f'G{int(width)}'  # Adjust command format as needed
        self.ser.write(width_value.encode())
        self.get_logger().info(f"Sent GRIP command with width: {width}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


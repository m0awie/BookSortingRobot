import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_serial')
        self.subscription = self.create_subscription(Float32, 'arduinoCommand', self.command_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyACM1', 9600)  # Update the baud rate & port as required

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        self.serial_port.write(str(command).encode())

def main(args=None):
    rclpy.init(args=args)
    util_arduino_serial = ArduinoNode()
    rclpy.spin(util_arduino_serial)
    util_arduino_serial.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
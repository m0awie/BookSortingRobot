import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from interfaces.srv import ArmCommand
import math

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Subscribe to book centroids and titles
        self.book_sub = self.create_subscription(String, 'book_centroids', self.book_callback, 10)

        # Initialize client to control the arm
        self.arm_client = self.create_client(ArmCommand, 'arm')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Arm service to become available...')

    def book_callback(self, msg):
        # Parse the message for book title and position
        data = msg.data.split(', ')
        book_title = data[0].split(': ')[1]
        position = {
            'x': float(data[1].split('=')[1]),
            'y': float(data[2].split('=')[1]),
            'z': float(data[3].split('=')[1])
        }

        self.get_logger().info(f"Moving to book titled '{book_title}' at position: x={position['x']}, y={position['y']}, z={position['z']}")

        # Move to the book position
        self.move_to_position(position['x'], position['y'], position['z'] + GRIPPER_REACH_DEPTH)

        # Send a command to the gripper to grasp the book (handled by the Gripper Node)
        self.send_gripper_command('grasp', book_title)

    def move_to_position(self, x, y, z):
        request = ArmCommand.Request()
        request.x = x
        request.y = y
        request.z = z
        request.roll = 0.0
        request.pitch = math.radians(-90)  # Adjust orientation if needed
        request.yaw = 0.0

        self.get_logger().info(f"Sending request to move arm to x={x}, y={y}, z={z}")
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Reached target position.")
        else:
            self.get_logger().warn("Failed to reach target position.")

    def send_gripper_command(self, command, title):
        # Implement this to send commands to the gripper (e.g., open, close)
        self.get_logger().info(f"Sending command to gripper: {command} for book '{title}'")

def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotNode()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

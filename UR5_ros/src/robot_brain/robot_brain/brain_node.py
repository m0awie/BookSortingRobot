import rclpy
from rclpy.node import Node
from interfaces.srv import ArmCommand
from std_msgs.msg import String, Float32
import re
import time

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        # Create a service client to interact with the robotic arm service
        self.action_client = self.create_client(ArmCommand, 'arm')

        while not self.action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robotic arm service...')

        self.gripper_publisher = self.create_publisher(Float32, 'arduinoCommand', 10) 
        self.create_subscription(String, 'book_centroids', self.book_centroid_callback, 10)

        
        
    def book_centroid_callback(self, msg):
        # Extract book title and coordinates from the message
        position_data = self.parse_position_data(msg.data)

        if position_data:
            title, x, y, z = position_data
            self.get_logger().info(f"Received book '{title}' at x={x:.2f}, y={y:.2f}, z={z:.2f}")

            # Set the arm command using the detected centroid position
            self.command = ArmCommand.Request()
            self.command.x = x
            self.command.y = y
            self.command.z = z + 0.149  # Adjust Z for tool offset
            self.command.qx, self.command.qy, self.command.qz, self.command.qw = 0.0, 0.7071, 0.0, 0.7071  # Orientation (adjust if needed)

            # Send command to the arm service
            self.get_logger().info(f"Sending arm command to pick up '{title}' at x={x}, y={y}, z={self.command.z}")
            self.send_action_command(self.command)

    def parse_position_data(self, data):
        # Parse the message to extract title and coordinates
        match = re.search(r"Book Title: (.*), Position: x=([-\d.]+), y=([-\d.]+), z=([-\d.]+)", data)
        if match:
            title = match.group(1)
            x = float(match.group(2))
            y = float(match.group(3))
            z = float(match.group(4))
            return title, x, y, z
        else:
            self.get_logger().error("Failed to parse book position data.")
            return None
        
    def send_action_command(self, command):
        future = self.action_client.call_async(command)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            self.handle_response(future)
        else:
            self.get_logger().error("Service call timed out.")

    def handle_response(self, future):
        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info("Command has been successfully received and executed.")
            elif result and not result.success:
                self.get_logger().error("Command execution failed.")
            else:
                self.get_logger().error("No response received from the service.")
        except Exception as e:
            self.get_logger().error(f"An exception occurred while processing the response: {e}")

    def activate_gripper(self, command_value):
        """Publishes a command to the gripper at the specified value."""
        command_msg = Float32()
        command_msg.data = command_value
        self.get_logger().info(f"Publishing gripper command: {command_value}")
        
        # Timer-based loop to publish at 0.5-second intervals
        for _ in range():  # Publish three times as an example; adjust based on your needs
            self.gripper_publisher.publish(command_msg)
            rclpy.spin_once(self, timeout_sec=0.5)  # Wait for 0.5 seconds

def main(args=None):
    # Initialize ROS client library and start the node
    rclpy.init(args=args)
    node = BrainNode()

    try:
        while rclpy.ok():
            # Process any outstanding callbacks or responses
            rclpy.spin_once(node, timeout_sec=0.5)  # Adjust timeout as needed

            # Optional: Check if the service call is complete and break the loop
            if hasattr(node, 'future') and node.future.done():
                break

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        
        # Create a subscription to receive gripper commands
        self.gripper_sub = self.create_subscription(
            String,
            'gripper_commands',  # The topic that will receive gripper commands
            self.gripper_callback,
            10
        )

    def gripper_callback(self, msg):
        # The message will contain the command, such as 'grasp', 'release', 'open', 'close'
        command = msg.data
        
        if command == 'grasp':
            self.get_logger().info("Grasping the book.")
            self.grasp_book()
        elif command == 'release':
            self.get_logger().info("Releasing the book.")
            self.release_book()
        elif command == 'open':
            self.get_logger().info("Opening the gripper.")
            self.open_gripper()
        elif command == 'close':
            self.get_logger().info("Closing the gripper.")
            self.close_gripper()
        else:
            self.get_logger().warn(f"Unknown command: {command}")
    
    def grasp_book(self):
        # Here we should add the actual code to close the gripper and grasp the book
        # This could be a hardware control command depending on your setup
        self.get_logger().info("Gripper is closing to grasp the book...")

    def release_book(self):
        # Release the book by opening the gripper
        self.get_logger().info("Gripper is opening to release the book...")

    def open_gripper(self):
        # Open the gripper (if it is hardware-controlled, add that functionality)
        self.get_logger().info("Gripper is now open.")

    def close_gripper(self):
        # Close the gripper to grasp the book
        self.get_logger().info("Gripper is now closed.")

def main(args=None):
    rclpy.init(args=args)
    gripper_node = GripperNode()
    rclpy.spin(gripper_node)
    gripper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

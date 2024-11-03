import rclpy
from rclpy.node import Node
from book_retrieval.msg import BookInfo
from moveit2 import MoveIt2
from moveit2.ros2 import MoveIt2Interface

class BookMoveNode(Node):
    def __init__(self):
        super().__init__('book_move_node')
        self.book_subscriber = self.create_subscription(BookInfo, 'book_info', self.book_callback, 10)
        self.moveit2 = MoveIt2Interface(node=self)
        self.moveit2.init_move_group('ur5_arm')  # Replace 'ur5_arm' as needed

    def book_callback(self, msg):
        # Get the position of the book
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        self.get_logger().info(f"Received book position at ({x}, {y}, {z})")

        # Move to the book's position
        self.move_to_position(x, y, z)
        
        # Send a signal to the gripper node if needed

    def move_to_position(self, x, y, z):
        self.moveit2.set_pose_goal([x, y, z, 0.0, 0.0, 0.0, 1.0])  # Adjust orientation
        self.moveit2.move()
        self.get_logger().info(f"Moved to position ({x}, {y}, {z})")

def main(args=None):
    rclpy.init(args=args)
    node = BookMoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


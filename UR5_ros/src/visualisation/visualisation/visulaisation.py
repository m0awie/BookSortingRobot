# used chat gpt to make this fast

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class GripperVisualisation(Node):
    def __init__(self):
        super().__init__('gripper_viz')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.jaw_width_sub = self.create_subscription(Float32, '/arduinoCommand', self.jaw_width_callback, 10)
        self.finger_pos = 0.0

    def jaw_width_callback(self, msg):
        jaw_width = msg.data
        # Each finger moves half the jaw width
        self.finger_position = min(jaw_width / 2, 0.05)  # Ensure the movement doesn't exceed joint limit

        # Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["left_joint", "right_joint"]
        joint_state_msg.position = [self.finger_position, self.finger_position]
        self.joint_state_pub.publish(joint_state_msg)
def main(args=None):
    rclpy.init(args=args)
    gripper_viz = GripperVisualisation()
    rclpy.spin(gripper_viz)
    gripper_viz.destroy_node()
    rclpy.shutdown()

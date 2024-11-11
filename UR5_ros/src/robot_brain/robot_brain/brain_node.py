import rclpy
from rclpy.node import Node
from interfaces.srv import ArmCommand

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        # Create a service client to interact with the robotic arm service
        self.action_client = self.create_client(ArmCommand, 'arm')

        while not self.action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robotic arm service...')

        self.command = ArmCommand.Request()

        goal_position = [0.588, 0.132, 0.222]
        quaternion = [0.0, 0.7071, 0.0, 0.7071]

        self.command.x = goal_position[0]
        self.command.y = goal_position[1]
        self.command.z = goal_position[2] + 0.149 # TCP Offset is 149 [MAKE SURE Z COORDS ARE +149]
        self.command.qx = quaternion[0]
        self.command.qy = quaternion[1]
        self.command.qz = quaternion[2]
        self.command.qw = quaternion[3]

        # Send command request to the service

        self.get_logger().info(f"Sending command with x: {self.command.x}, y: {self.command.y}, z: {self.command.z}")
        self.get_logger().info(f"Quaternion: [{self.command.qx}, {self.command.qy}, {self.command.qz}, {self.command.qw}]")

        self.send_action_command(self.command)


    # def send_action_command(self, command):
    #     # Asynchronously call the service and wait for a response
    #     future = self.action_client.call_async(command)
    #     future.add_done_callback(self.handle_response)

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
# Python example: This script creates a ROS 2 node that publishes 
# a HOLD command on the /control_command topic to keep the robot 
# in a safe hold state after a short startup delay.





# --------------------------- IMPORTS ---------------------------
# rclpy is the ROS2 client library for Python, which allows us to create nodes, publishers, and subscribers.
import rclpy
from rclpy.node import Node
# px_mc_msgs.msg is a custom message package that contains the ControlCommand message type
from px_mc_msgs.msg import ControlCommand



#--------------------------- PUBLISHER NODE CLASS ---------------------------
# We create a class that inherits from the Node class from the rlcpy library.
# This class will represent our publisher node which will send control commands to the robot.
class ControlCommandPublisher(Node):
    def __init__(self):
        # Initialize the parent class (Node) and set the node name to 'control_command_publisher'
        # The node name must be unique in the ROS network and is used to identify this publisher node
        super().__init__('control_command_publisher')
        self.publisher = self.create_publisher(ControlCommand, '/control_command', 10)
        # ControlCommand is the type of msg (custom) 
        # /control_command is the topic name where the message will be published
        # 10 is the queue size for the publisher

    def send_hold_command(self):
        # Create a new empty ControlCommand message
        msg = ControlCommand()
        # Initialize the command field of the message to the constant "HOLD" from the 
        # definition of the CommandControl message type. In this case it's probably HOLD=2
        msg.command = ControlCommand.HOLD
        # Call the publish method of the publisher to send the message to the /control_command topic
        self.publisher.publish(msg)
        # Log an info message to the console 
        self.get_logger().info('Published HOLD command')



#--------------------------- MAIN FUNCTION ---------------------------
def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    # Create an instance of the ControlCommandPublisher node
    node = ControlCommandPublisher()

    try:
        import time
        # Wait for 1 second to ensure the node is fully initialized before sending the command
        time.sleep(1)
        # Send the HOLD command to the robot
        node.send_hold_command()
        # Spin briefly again to ensure the message is sent
        rclpy.spin_once(node, timeout_sec=2)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
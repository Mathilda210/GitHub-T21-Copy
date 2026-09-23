# Python example: creates a ROS 2 subscriber that listens 
# on the /internal/arm_status topic and logs the current 
# status of the left/right arm, waist, head, and hands 
# as human-readable strings.


#--------------------------- IMPORTS ---------------------------
import rclpy
from rclpy.node import Node
# px_mc_msgs.msg is a custom message package that contains the ArmStatus message type
from px_mc_msgs.msg import ArmStatus



#--------------------------- SUBSCRIBER NODE CLASS ---------------------------
class ArmStatusSubscriber(Node):
    def __init__(self):
        # Initialize the parent class (Node) and set the node name to 'arm_status_subscriber'
        # The node name must be unique in the ROS network and is used to identify this subscriber node
        super().__init__('arm_status_subscriber')

        # Create a subscription using the internal method create_subscription
        self.subscription = self.create_subscription(
            # message type
            ArmStatus,
            # topic name
            '/internal/arm_status',
            # callback function to handle incoming messages (defined below)
            self.listener_callback,
            # queue size for the subscriber
            10
        )

    def listener_callback(self, msg):
        # Define a mapping of status codes to human-readable strings
        state_map = {
            0: 'INITIAL',
            1: 'LOCKED',
            2: 'HOLD',
            3: 'MOVE',
            4: 'FREEDRAG',
            9: 'ERROR',
            10: 'IDLE'
        }

        # Log the current state to the console using the get_logger() method of the Node class
        # The default state is UNKNOWN
        self.get_logger().info(
            f'Left arm  state: {state_map.get(msg.left_arm_status, "UNKNOWN")}'
        )
        self.get_logger().info(
            f'Right arm  state: {state_map.get(msg.right_arm_status, "UNKNOWN")}'
        )
        self.get_logger().info(
            f'Waist  state: {state_map.get(msg.waist_arm_status, "UNKNOWN")}'
        )
        self.get_logger().info(
            f'Head  state: {state_map.get(msg.head_arm_status, "UNKNOWN")}'
        )
        self.get_logger().info(
            f'Left hand  state: {state_map.get(msg.left_hand_status, "UNKNOWN")}'
        )
        self.get_logger().info(
            f'Right hand  state: {state_map.get(msg.right_hand_status, "UNKNOWN")}'
        )



# --------------------------- MAIN FUNCTION ---------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ArmStatusSubscriber()

    try:
        # Spin the node to keep it alive and listening for messages on the /internal/arm_status 
        # topic indefinitely until the program is interrupted (e.g., Ctrl+C)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# Python example: creates a subscriber listens to the /system_status topic
# and logs the current system state as a readable label whenever a new status message is received.


#--------------------------- IMPORTS ---------------------------
import rclpy
from rclpy.node import Node
# custom message package that contains the SystemStatus message type
from px_mc_msgs.msg import SystemStatus



#--------------------------- SUBSCRIBER NODE CLASS ---------------------------
class SystemStatusSubscriber(Node):
    def __init__(self):
        # Initialize the parent class (Node) and set the node name to 'system_status_subscriber'
        # The node name must be unique in the ROS network and is used to identify this subscriber node
        super().__init__('system_status_subscriber')

        self.subscription = self.create_subscription(
            # message type
            SystemStatus,
            # topic name
            '/system_status',
            # callback function to handle incoming messages (defined below)
            self.listener_callback,
            # queue size for the subscriber
            10
            )
            

    # callback function called whenever a new message is received on the /system_status topic
    def listener_callback(self, msg):
        state_map = {
            0: 'INITIAL',
            1: 'LOCKED',
            2: 'HOLD',
            3: 'MOVE',
            4: 'FREEDRAG',
            9: 'ERROR',
            20: 'HOLDING'
        }

        # Log the current system state to the console using the get_logger() method of the Node class
        self.get_logger().info(
            f'Current system state: {state_map[msg.status]}'
        )



#--------------------------- MAIN FUNCTION ---------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SystemStatusSubscriber()

    try:
        # Spin the node to keep it active and listening for incoming messages on the /system_status topic
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
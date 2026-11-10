# Python 示例
import rclpy
from rclpy.node import Node
from px_mc_msgs.msg import ArmStatus

class ArmStatusSubscriber(Node):
    def __init__(self):
        # 初始化父类（Node），设置节点名称为'arm_status_subscriber'
        # 节点名称在ROS网络中唯一，用于标识该订阅节点
        super().__init__('arm_status_subscriber')

        self.subscription = self.create_subscription(
            ArmStatus,
            '/internal/arm_status',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        state_map = {
            0: 'INITIAL',
            1: 'LOCKED',
            2: 'HOLD',
            3: 'MOVE',
            4: 'FREEDRAG',
            9: 'ERROR',
            10: 'IDLE'
        }

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

def main(args=None):
    rclpy.init(args=args)
    node = ArmStatusSubscriber()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()                
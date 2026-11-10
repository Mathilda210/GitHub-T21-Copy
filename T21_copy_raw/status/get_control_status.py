# Python 示例
import rclpy
from rclpy.node import Node
from px_mc_msgs.msg import SystemStatus

class SystemStatusSubscriber(Node):
    def __init__(self):
        # 初始化父类（Node），设置节点名称为'system_status_subscriber'
        # 节点名称在ROS网络中唯一，用于标识该订阅节点
        super().__init__('system_status_subscriber')

        self.subscription = self.create_subscription(
            SystemStatus,
            '/system_status',
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
            20: 'HOLDING'
        }

        self.get_logger().info(
            f'Current system state: {state_map[msg.status]}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SystemStatusSubscriber()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()         
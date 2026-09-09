# Python 示例
import rclpy
from rclpy.node import Node
from px_mc_msgs.msg import DataToArm

class JointDataSubscriber(Node):
    def __init__(self):
        # 初始化父类（Node），设置节点名称为'joint_data_subscriber'
        # 节点名称在ROS网络中唯一，用于标识该订阅节点
        super().__init__('joint_data_subscriber')

        # 创建订阅者（Subscriber）
        # 参数说明：
        # 1. DataToArm：订阅的消息类型（自定义的机械臂关节数据类型）
        # 2. '/internal/data_to_arm'：订阅的话题名称（机械臂状态发布节点会向该话题推送数据）
        # 3. self.listener_callback：消息接收后的回调函数（解析并打印关节数据）
        # 4. 10：消息队列大小（缓存10条消息，超出则丢弃旧消息，避免数据积压）
        self.subscription = self.create_subscription(
            DataToArm,
            '/internal/data_to_arm',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        for i, pos in enumerate(msg.left_arm_pos):
            # 解析并打印左臂关节位置（单位：弧度rad，保留4位小数）
            self.get_logger().info(f'Left arm joint {i}: {pos:.4f} rad')

        for i, pos in enumerate(msg.right_arm_pos):
            # 解析并打印右臂关节位置（单位：弧度rad，保留4位小数）
            self.get_logger().info(f'Right arm joint {i}: {pos:.4f} rad')

        for i, pos in enumerate(msg.waist_arm_pos):
            # 解析并打印腰部关节位置（单位：弧度rad，保留4位小数）
            self.get_logger().info(f'Waist joint {i}: {pos:.4f} rad')

        for i, pos in enumerate(msg.head_arm_pos):
            # 解析并打印头部关节位置（单位：弧度rad，保留4位小数）
            self.get_logger().info(f'Head joint {i}: {pos:.4f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = JointDataSubscriber()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from px_perception_msgs.msg import ChassisState


class ChassisStateListener(Node):
    def __init__(self):
        super().__init__('chassis_state_listener')

        self.sub = self.create_subscription(
            ChassisState,
            '/chassis/chassis_state',
            self.callback,
            10
        )

    def callback(self, msg):
        state_map = {
            0: '初始化中',
            1: '手动',
            2: '自动',
            3: '停止'
        }

        self.get_logger().info(
            f'系统状态: {state_map.get(msg.system_state, "未知")}, '
            f'自动状态: {msg.auto_state}, '
            f'急停: {msg.emergency_stop}'
        )


if __name__ == '__main__':
    rclpy.init()

    rclpy.spin(ChassisStateListener())

    rclpy.shutdown()
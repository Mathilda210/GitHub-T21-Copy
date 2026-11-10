# Python 示例
import rclpy
from rclpy.node import Node
from px_mc_msgs.msg import ControlCommand

class ControlCommandPublisher(Node):
    def __init__(self):
        # 初始化父类（Node），设置节点名称为'control_command_publisher'
        # 节点名称在ROS网络中唯一，用于标识该订阅节点
        super().__init__('control_command_publisher')
        self.publisher = self.create_publisher(ControlCommand, '/control_command', 10)

    def send_hold_command(self):
        msg = ControlCommand()
        msg.command = ControlCommand.HOLD
        self.publisher.publish(msg)
        self.get_logger().info('Published HOLD command')

def main(args=None):
    rclpy.init(args=args)
    node = ControlCommandPublisher()
    try:
        # 等待一小段时间确保节点就绪，然后发送指令
        import time
        time.sleep(1)

        node.send_hold_command()

        # 再 spin 一会儿确保消息发出
        rclpy.spin_once(node, timeout_sec=2)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
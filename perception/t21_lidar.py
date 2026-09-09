#!/usr/bin/env python3
# 上一行指定了脚本的解释器，确保在ROS2环境中能正确执行

import rclpy
from rclpy.node import Node

# 引入QoS相关的配置项，这对于连接雷达至关重要
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String


class LidarNodePy(Node):
    def __init__(self):
        # 初始化节点名称为 'lidar_node_py'
        super().__init__('lidar_node_py')

        # --- 定义 Topic 名称 ---
        topic_lidar = '/rslidar_points'
        topic_exception_name = "/rslidar_sdk/rslidar_sdk/exception"
        topic_imu = '/rslidar_imu_data'

        # QoS 设置：必须设置为 BEST_EFFORT
        # 注释：雷达驱动通常为了保证低延迟，使用 "Best Effort" (尽力而为) 策略发送数据。
        # 如果接收端（这里）默认使用 "Reliable" (可靠)，会导致QoS策略不兼容，从而收不到任何数据。
        # depth=10 表示消息队列的长度，防止数据堆积。
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # 创建订阅者
        # 参数1: 消息类型 PointCloud2
        # 参数2: Topic 名称 '/rslidar_points'
        # 参数3: 回调函数 self.listener_callback
        # 参数4: 上面定义的 QoS 配置
        self.create_subscription(
            PointCloud2,
            topic_lidar,
            self.listener_callback,
            qos
        )

        # --- 雷达异常状态订阅配置 ---
        # 状态信息通常比较关键且频率低，通常使用默认的 QoS (Reliable) 即可
        self.create_subscription(
            String,
            topic_exception_name,
            self.exception_callback,
            10
        )

        # --- 订阅 IMU 数据 ---
        # 注意：这里同样使用了 qos_sensor (Best Effort)
        self.create_subscription(
            Imu,
            topic_imu,
            self.imu_callback,
            qos
        )

        self.get_logger().info(
            "Python 节点已启动，正在监听 /rslidar_points"
        )
        self.get_logger().info(
            "Python 节点已启动，正在监听 /rslidar_sdk/rslidar_sdk/exception"
        )
        self.get_logger().info(
            "Python 节点已启动，正在监听 /rslidar_imu_data"
        )

        self.count = 0
        self.imu_count = 0

    def listener_callback(self, msg):
        """
        回调函数：每当接收到一帧雷达消息，ROS2就会自动调用这个函数
        msg: 收到的 PointCloud2 原始消息对象
        """
        self.count += 1

        # 降频处理：每接收10帧只处理1帧
        if self.count % 10 != 0:
            return

        # 打印当前帧的 Frame ID 和点云总数
        self.get_logger().info(
            f"--- [Python] Frame: {msg.header.frame_id}, "
            f"Points: {msg.width * msg.height} ---"
        )

        # 解析点云
        gen = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        # 打印前3个点
        i = 0

        for p in gen:
            self.get_logger().info(
                f"Point {i}: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.      )

            i += 1

            # 只打印前3个点
            if i >= 3:
                break

    def exception_callback(self, msg):
        """
        回调函数：处理雷达异常/状态信息
        每当 /rslidar_sdk/rslidar_sdk/exception 发来消息时调用
        """
        self.get_logger().warn(
            f">>> [Python EXCEPTION] 收到雷达状态: {msg.data}"
        )

    def imu_callback(self, msg):
        """
        IMU 回调函数
        msg 类型: sensor_msgs/msg/Imu
        """
        self.imu_count += 1

        # IMU 频率通常很高 (100Hz+)，建议降频打印
        if self.imu_count % 50 != 0:
            return

        # 提取加速度
        acc = msg.linear_acceleration

        # 提取角速度
        gyro = msg.angular_velocity

        self.get_logger().info(
            f"--- [IMU] Acc: x={acc.x:.2f}, y={acc.y:.2f}, z={acc.z:.2f} | "
            f"Gyro: x={gyro.x:.2f}, y={gyro.y:.2f}, z={gyro.z:.2f} ---"
        )


def main(args=None):
    # 初始化 ROS 2 Python 客户端库
    rclpy.init(args=args)

    # 启动节点并阻塞在这里，直到节点被关闭
    rclpy.spin(LidarNodePy())

    # 销毁节点资源并关闭库
    rclpy.shutdown()


if __name__ == '__main__':
    main()
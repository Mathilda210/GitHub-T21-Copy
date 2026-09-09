import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Float32


class SensorFrequencyMonitor(Node):
    def __init__(self):
        super().__init__('sensor_freq_monitor')

        # ----------- 定义 Best Effort 的 QoS 策略 -----------
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # ----------- 1. 订阅话题 -----------

        # 订阅点云
        self.sub_points = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.points_callback,
            best_effort_qos
        )

        # 订阅 IMU
        self.sub_imu = self.create_subscription(
            Imu,
            '/rslidar_imu_data',
            self.imu_callback,
            best_effort_qos
        )

        # ----------- 2. 发布频率话题 -----------

        # 分别发布两个话题的频率
        self.pub_points_hz = self.create_publisher(
            Float32,
            '/rslidar_points_hz',
            best_effort_qos
        )

        self.pub_imu_hz = self.create_publisher(
            Float32,
            '/rslidar_imu_hz',
            best_effort_qos
        )

        # ----------- 3. 计数与计时 -----------

        self.point_count = 0
        self.imu_count = 0
        self.start_time = self.get_clock().now()

        # 1秒定时器
        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.get_logger().info(
            '节点已启动: 正在同时监控 LiDAR 和 IMU 的频率...'
        )

    def points_callback(self, msg):
        """点云计数"""
        self.point_count += 1

    def imu_callback(self, msg):
        """IMU 计数"""
        self.imu_count += 1

    def timer_callback(self):
        """每秒计算一次频率"""

        now = self.get_clock().now()
        dt = (
            now - self.start_time
        ).nanoseconds / 1e9  # 时间间隔(秒)

        if dt > 0:
            # 1. 计算 LiDAR 频率
            points_hz = self.point_count / dt

            msg_p = Float32()
            msg_p.data = points_hz

            self.pub_points_hz.publish(msg_p)

            # 2. 计算 IMU 频率
            imu_hz = self.imu_count / dt

            msg_i = Float32()
            msg_i.data = imu_hz

            self.pub_imu_hz.publish(msg_i)

            # 3. 终端打印日志
            self.get_logger().info(
                f'LiDAR: {points_hz:6.2f} Hz  |  '
                f'IMU: {imu_hz:6.2f} Hz'
            )

        # 重置计数器和时间
        self.point_count = 0
        self.imu_count = 0
        self.start_time = now


def main(args=None):
    rclpy.init(args=args)

    node = SensorFrequencyMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
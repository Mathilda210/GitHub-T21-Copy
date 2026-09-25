#!/usr/bin/env python3
# The shebang above selects the interpreter used to run this script.

import rclpy
from rclpy.node import Node
# Import QoS settings, which are important for connecting to the LiDAR.
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# custom
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2 as pc2


class LidarNodePy(Node):
    def __init__(self):
        # Initialize the node with the name 'lidar_node_py'.
        super().__init__('lidar_node_py')

        # --- Define topic names ---
        topic_lidar = '/rslidar_points'
        topic_exception_name = "/rslidar_sdk/rslidar_sdk/exception"
        topic_imu = '/rslidar_imu_data'

        # QoS setting: BEST_EFFORT is required.
        # LiDAR drivers usually use Best Effort to maintain low latency.
        # If the subscriber uses Reliable by default, the QoS policies are
        # incompatible and no data will be received.
        # depth=10 sets the message queue length to prevent data buildup.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Create the point cloud subscriber.
        # Argument 1: PointCloud2 message type.
        # Argument 2: Topic name '/rslidar_points'.
        # Argument 3: Callback function self.listener_callback.
        # Argument 4: The QoS configuration defined above.
        self.create_subscription(
            PointCloud2,
            topic_lidar,
            self.listener_callback,
            qos
        )

        # --- Configure the LiDAR exception/status subscriber ---
        # Status information is important but low-frequency, so the default
        # Reliable QoS is sufficient.
        self.create_subscription(
            String,
            topic_exception_name,
            self.exception_callback,
            10
        )

        # --- Subscribe to IMU data ---
        # This subscriber also uses the Best Effort QoS profile.
        self.create_subscription(
            Imu,
            topic_imu,
            self.imu_callback,
            qos
        )

        self.get_logger().info(
            "Python node started; listening to /rslidar_points"
        )
        self.get_logger().info(
            "Python node started; listening to /rslidar_sdk/rslidar_sdk/exception"
        )
        self.get_logger().info(
            "Python node started; listening to /rslidar_imu_data"
        )

        self.count = 0
        self.imu_count = 0

    def listener_callback(self, msg):
        """
        Callback automatically invoked by ROS 2 when a LiDAR message is
        received.
        msg: Received raw PointCloud2 message.
        """
        self.count += 1

        # Throttle processing: process only one frame out of every ten.
        if self.count % 10 != 0:
            return

        # Log the current frame ID and total number of points.
        self.get_logger().info(
            f"--- [Python] Frame: {msg.header.frame_id}, "
            f"Points: {msg.width * msg.height} ---"
        )

        # Parse the point cloud.
        gen = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        # Log the first three points.
        i = 0

        for p in gen:
            self.get_logger().info(
                f"Point {i}: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})"
            )

            i += 1

            # Only log the first three points.
            if i >= 3:
                break

    def exception_callback(self, msg):
        """
        Callback for LiDAR exception and status information.
        Called whenever a message is received on
        /rslidar_sdk/rslidar_sdk/exception.
        """
        self.get_logger().warn(
            f">>> [Python EXCEPTION] LiDAR status received: {msg.data}"
        )

    def imu_callback(self, msg):
        """
        IMU callback.
        msg type: sensor_msgs/msg/Imu.
        """
        self.imu_count += 1

        # IMU data is usually high-frequency (100 Hz or more), so throttle
        # logging.
        if self.imu_count % 50 != 0:
            return

        # Extract linear acceleration.
        acc = msg.linear_acceleration

        # Extract angular velocity.
        gyro = msg.angular_velocity

        self.get_logger().info(
            f"--- [IMU] Acc: x={acc.x:.2f}, y={acc.y:.2f}, z={acc.z:.2f} | "
            f"Gyro: x={gyro.x:.2f}, y={gyro.y:.2f}, z={gyro.z:.2f} ---"
        )


def main(args=None):
    # Initialize the ROS 2 Python client library.
    rclpy.init(args=args)

    # Start the node and block until it is shut down.
    rclpy.spin(LidarNodePy())

    # Destroy the node resources and shut down the library.
    rclpy.shutdown()


if __name__ == '__main__':
    main()
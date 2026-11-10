import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import (
    CameraInfo,
    Image,
    PointCloud2,
)


NODE_NAME = "waist_depth_camera_test_node"

# robot_id 必须从环境变量读取再拼接 topic
# （实际 topic 带 /ROBOT_ID 前缀，写死会取不到数据）
ROBOT_ID = os.environ.get('ROBOT_ID', 'robot1')

DEPTH_INFO_TOPIC = (
    f"/{ROBOT_ID}/ascamera_hp60c/"
    "camera_publisher/depth0/camera_info"
)

DEPTH_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/ascamera_hp60c/"
    "camera_publisher/depth0/image_raw"
)

DEPTH_CLOUD_TOPIC = (
    f"/{ROBOT_ID}/ascamera_hp60c/"
    "camera_publisher/depth0/points"
)

RGB_INFO_TOPIC = (
    f"/{ROBOT_ID}/ascamera_hp60c/"
    "camera_publisher/rgb0/camera_info"
)

RGB_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/ascamera_hp60c/"
    "camera_publisher/rgb0/image"
)


class WaistCameraTestNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.subscriptions_ = []

        self.subscriptions_.append(
            self.create_subscription(
                CameraInfo,
                DEPTH_INFO_TOPIC,
                self.depth_info_callback,
                qos_profile_system_default
            )
        )

        self.subscriptions_.append(
            self.create_subscription(
                Image,
                DEPTH_IMAGE_TOPIC,
                self.depth_image_callback,
                qos_profile_system_default
            )
        )

        self.subscriptions_.append(
            self.create_subscription(
                PointCloud2,
                DEPTH_CLOUD_TOPIC,
                self.depth_cloud_callback,
                qos_profile_system_default
            )
        )

        self.subscriptions_.append(
            self.create_subscription(
                CameraInfo,
                RGB_INFO_TOPIC,
                self.rgb_info_callback,
                qos_profile_system_default
            )
        )

        self.subscriptions_.append(
            self.create_subscription(
                Image,
                RGB_IMAGE_TOPIC,
                self.rgb_image_callback,
                qos_profile_system_default
            )
        )

    def depth_info_callback(self, msg):
        self.get_logger().info(
            f"Depth camera info "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def depth_image_callback(self, msg):
        self.get_logger().info(
            f"Depth camera image "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def depth_cloud_callback(self, msg):
        self.get_logger().info(
            f"Depth camera point cloud size=[{len(msg.data)}]"
        )

    def rgb_info_callback(self, msg):
        self.get_logger().info(
            f"RGB camera info "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def rgb_image_callback(self, msg):
        self.get_logger().info(
            f"RGB camera image "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )


def main(args=None):
    rclpy.init(args=args)

    node = WaistCameraTestNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except RuntimeError:
        # rclpy(Humble) 的 Ctrl+C 恰好落在取消息瞬间时会抛 RuntimeError
        if rclpy.ok():
            raise

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
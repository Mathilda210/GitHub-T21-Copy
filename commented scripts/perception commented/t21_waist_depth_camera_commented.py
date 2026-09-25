#!/usr/bin/env python3
# ROS 2 test node that subscribes to the waist camera's depth, RGB, and point
# cloud topics, then logs basic information about the received messages.

# --------------- IMPORTS --------------------
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


# custom
from sensor_msgs.msg import (
    CameraInfo,
    Image,
    PointCloud2,
)


# --------- COSNTANTS -----------------
NODE_NAME = "waist_depth_camera_test_node"

# Read the robot ID from the environment before building topic names.
# The actual topics include the /ROBOT_ID prefix, so hardcoding it would prevent
# the node from receiving data from other robots.
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




# ------------ WaistCameraTestNode defintion --------------
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




# ---------------- MAIN ----------------------------
def main(args=None):
    rclpy.init(args=args)

    node = WaistCameraTestNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except RuntimeError:
        # rclpy (Humble) may raise RuntimeError if Ctrl+C occurs while
        # receiving a message.
        if rclpy.ok():
            raise

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
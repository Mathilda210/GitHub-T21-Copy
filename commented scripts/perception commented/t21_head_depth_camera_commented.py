#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Head RGB-D camera test node.
# This script subscribes to the RealSense D435i depth, RGB, infrared,
# metadata, and extrinsics topics, then logs basic information from each message.

# Before running, source the following ROS 2 environments:
# source /opt/ros/humble/setup.bash
# source ~/px_robot_code/paxini_ros2_msgs-1.0.1-99ad226/install/setup.bash
# source ~/px_robot_code/camera_service/realsense/install/setup.bash


# -------------- IMPORTS -----------------------
import os
import rclpy
from rclpy.node import Node

# Import QoS profiles:
# qos_profile_sensor_data: Best Effort, suitable for high-bandwidth data where
# occasional packet loss is acceptable, such as image streams.
# qos_profile_system_default: Reliable, suitable for important data such as
# camera calibration and parameters.
from rclpy.qos import (
    qos_profile_sensor_data,
    qos_profile_system_default,
)

# Custom imports

# Import standard message types.
from sensor_msgs.msg import CameraInfo, Image

# Import RealSense-specific message types.
# Make sure realsense2_camera_msgs is installed.
from realsense2_camera_msgs.msg import Metadata, Extrinsics


# ----------------- Constant definitions --------------------
NODE_NAME = "head_depth_camera_test_node"

# Read the environment variable; use 'robot1' if it is not set.
ROBOT_ID = os.environ.get('ROBOT_ID', 'robot1')

# Topic name definitions.
HEAD_RGBD_CAMERA_DEPTH_INFO_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/depth/camera_info"
)
HEAD_RGBD_CAMERA_DEPTH_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/depth/image_rect_raw"
)

HEAD_RGBD_CAMERA_RGB_INFO_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/color/camera_info"
)
HEAD_RGBD_CAMERA_RGB_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/color/image_raw"
)

HEAD_RGBD_CAMERA_RGB_METADATA_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/color/metadata"
)
HEAD_RGBD_CAMERA_DEPTH_METADATA_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/depth/metadata"
)

HEAD_RGBD_CAMERA_EXTRINSICS_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/extrinsics/depth_to_color"
)

# Left infrared stream (infra1) and right infrared stream (infra2).
HEAD_RGBD_CAMERA_LEFT_IR_INFO_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/infra1/camera_info"
)
HEAD_RGBD_CAMERA_LEFT_IR_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/infra1/image_rect_raw"
)

HEAD_RGBD_CAMERA_RIGHT_IR_INFO_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/infra2/camera_info"
)
HEAD_RGBD_CAMERA_RIGHT_IR_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/infra2/image_rect_raw"
)

HEAD_RGBD_CAMERA_LEFT_IR_METADATA_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/infra1/metadata"
)
HEAD_RGBD_CAMERA_RIGHT_IR_METADATA_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/infra2/metadata"
)

# Raw color stream.
HEAD_RGBD_CAMERA_RGB_YUYV_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/color/image_raw_yuyv"
)

# Extrinsics.
HEAD_RGBD_CAMERA_EXTRINSICS_DEPTH_TO_LEFT_IR_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/extrinsics/depth_to_infra1"
)

HEAD_RGBD_CAMERA_EXTRINSICS_DEPTH_TO_RIGHT_IR_TOPIC = (
    f"/{ROBOT_ID}/D435i_1/extrinsics/depth_to_infra2"
)





# ----------------- HeadRGBCamera Node definition ----------------------
class HeadRGBDCameraNode(Node):
    """
    Class name: HeadRGBDCameraNode

    Function:
        Inherits from rclpy.node.Node.
        Tests the main data streams of the RealSense D435i camera.
        The node subscribes to and processes depth images, RGB images,
        camera calibration data, metadata, and extrinsics.
    """

    def __init__(self):
        super().__init__(NODE_NAME)

        # Store subscription objects to prevent them from being garbage-collected.
        self.subscriptions_ = []

        # 1. Depth camera info
        self.subscriptions_.append(
            self.create_subscription(
                CameraInfo,
                HEAD_RGBD_CAMERA_DEPTH_INFO_TOPIC,
                self.depth_info_callback,
                qos_profile_system_default
            )
        )

        # 2. Depth image
        self.subscriptions_.append(
            self.create_subscription(
                Image,
                HEAD_RGBD_CAMERA_DEPTH_IMAGE_TOPIC,
                self.depth_image_callback,
                qos_profile_sensor_data
            )
        )

        # 3. RGB camera info
        self.subscriptions_.append(
            self.create_subscription(
                CameraInfo,
                HEAD_RGBD_CAMERA_RGB_INFO_TOPIC,
                self.rgb_info_callback,
                qos_profile_system_default
            )
        )

        # 4. RGB image
        self.subscriptions_.append(
            self.create_subscription(
                Image,
                HEAD_RGBD_CAMERA_RGB_IMAGE_TOPIC,
                self.rgb_image_callback,
                qos_profile_sensor_data
            )
        )

        # 5. RGB metadata
        self.subscriptions_.append(
            self.create_subscription(
                Metadata,
                HEAD_RGBD_CAMERA_RGB_METADATA_TOPIC,
                self.rgb_metadata_callback,
                qos_profile_sensor_data
            )
        )

        # 6. Depth metadata
        self.subscriptions_.append(
            self.create_subscription(
                Metadata,
                HEAD_RGBD_CAMERA_DEPTH_METADATA_TOPIC,
                self.depth_metadata_callback,
                qos_profile_sensor_data
            )
        )

        # 7. Extrinsics depth -> color
        self.subscriptions_.append(
            self.create_subscription(
                Extrinsics,
                HEAD_RGBD_CAMERA_EXTRINSICS_TOPIC,
                self.extrinsics_callback,
                qos_profile_sensor_data
            )
        )

        # 8. Left IR info
        self.subscriptions_.append(
            self.create_subscription(
                CameraInfo,
                HEAD_RGBD_CAMERA_LEFT_IR_INFO_TOPIC,
                self.left_ir_info_callback,
                qos_profile_system_default
            )
        )

        # 9. Left IR image
        self.subscriptions_.append(
            self.create_subscription(
                Image,
                HEAD_RGBD_CAMERA_LEFT_IR_IMAGE_TOPIC,
                self.left_ir_image_callback,
                qos_profile_sensor_data
            )
        )

        # 10. Right IR info
        self.subscriptions_.append(
            self.create_subscription(
                CameraInfo,
                HEAD_RGBD_CAMERA_RIGHT_IR_INFO_TOPIC,
                self.right_ir_info_callback,
                qos_profile_system_default
            )
        )

        # 11. Right IR image
        self.subscriptions_.append(
            self.create_subscription(
                Image,
                HEAD_RGBD_CAMERA_RIGHT_IR_IMAGE_TOPIC,
                self.right_ir_image_callback,
                qos_profile_sensor_data
            )
        )

        # 12. Left IR metadata
        self.subscriptions_.append(
            self.create_subscription(
                Metadata,
                HEAD_RGBD_CAMERA_LEFT_IR_METADATA_TOPIC,
                self.left_ir_metadata_callback,
                qos_profile_sensor_data
            )
        )

        # 13. Right IR metadata
        self.subscriptions_.append(
            self.create_subscription(
                Metadata,
                HEAD_RGBD_CAMERA_RIGHT_IR_METADATA_TOPIC,
                self.right_ir_metadata_callback,
                qos_profile_sensor_data
            )
        )

        # 14. RGB YUYV image
        self.subscriptions_.append(
            self.create_subscription(
                Image,
                HEAD_RGBD_CAMERA_RGB_YUYV_IMAGE_TOPIC,
                self.rgb_yuyv_image_callback,
                qos_profile_sensor_data
            )
        )

        # 15. Depth -> Left IR extrinsics
        self.subscriptions_.append(
            self.create_subscription(
                Extrinsics,
                HEAD_RGBD_CAMERA_EXTRINSICS_DEPTH_TO_LEFT_IR_TOPIC,
                self.extrinsics_depth_to_left_ir_callback,
                qos_profile_sensor_data
            )
        )

        # 16. Depth -> Right IR extrinsics
        self.subscriptions_.append(
            self.create_subscription(
                Extrinsics,
                HEAD_RGBD_CAMERA_EXTRINSICS_DEPTH_TO_RIGHT_IR_TOPIC,
                self.extrinsics_depth_to_right_ir_callback,
                qos_profile_sensor_data
            )
        )

    def rgb_yuyv_image_callback(self, msg):
        self.get_logger().info(
            f"RGB(YUYV) image height=[{msg.height}] "
            f"width=[{msg.width}] "
            f"encoding=[{msg.encoding}]"
        )

    def extrinsics_depth_to_left_ir_callback(self, msg):
        self.get_logger().info(
            f"Extrinsics depth->infra1 translation=[{msg.translation}]"
        )

    def extrinsics_depth_to_right_ir_callback(self, msg):
        self.get_logger().info(
            f"Extrinsics depth->infra2 translation=[{msg.translation}]"
        )

    def depth_info_callback(self, msg):
        self.get_logger().info(
            f"Depth camera info height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def depth_image_callback(self, msg):
        self.get_logger().info(
            f"Depth camera image height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def rgb_info_callback(self, msg):
        self.get_logger().info(
            f"RGB camera info height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def rgb_image_callback(self, msg):
        self.get_logger().info(
            f"RGB camera image height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def rgb_metadata_callback(self, msg):
        self.get_logger().info(
            f"RGB Metadata Frame: [{msg.header.frame_id}]"
        )

    def depth_metadata_callback(self, msg):
        self.get_logger().info(
            f"Depth Metadata Frame: [{msg.header.frame_id}]"
        )

    def extrinsics_callback(self, msg):
        self.get_logger().info(
            f"Extrinsics Translation: [{msg.translation}]"
        )

    def left_ir_info_callback(self, msg):
        self.get_logger().info(
            f"Left IR camera info "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def left_ir_image_callback(self, msg):
        self.get_logger().info(
            f"Left IR camera image "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def right_ir_info_callback(self, msg):
        self.get_logger().info(
            f"Right IR camera info "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def right_ir_image_callback(self, msg):
        self.get_logger().info(
            f"Right IR camera image "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def left_ir_metadata_callback(self, msg):
        self.get_logger().info(
            f"Left IR Metadata Frame: "
            f"[{msg.header.frame_id}]"
        )

    def right_ir_metadata_callback(self, msg):
        self.get_logger().info(
            f"Right IR Metadata Frame: "
            f"[{msg.header.frame_id}]"
        )




# ----------------- MAIN ----------------------
def main(args=None):
    rclpy.init(args=args)

    node = HeadRGBDCameraNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except RuntimeError:
        if rclpy.ok():
            raise

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
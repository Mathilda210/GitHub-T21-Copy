#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    qos_profile_system_default,
)

from sensor_msgs.msg import CameraInfo, Image


# 定义常量：节点名称和话题名称
NODE_NAME = "hand_rgb_camera_test_node"

# 獲取環境變量，如果沒有讀取到，預設使用 'robot1'
ROBOT_ID = os.environ.get('ROBOT_ID', 'robot1')

HAND_LEFT_RGB_INFO_TOPIC = (
    f"/{ROBOT_ID}/left_hand_video1/camera_info"
)
HAND_LEFT_RGB_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/left_hand_video1/image_raw"
)

HAND_RIGHT_RGB_INFO_TOPIC = (
    f"/{ROBOT_ID}/right_hand_video1/camera_info"
)
HAND_RIGHT_RGB_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/right_hand_video1/image_raw"
)


class HandRgbCameraNode(Node):
    """
    类名称: HandRgbCameraNode

    功能:
        继承自 rclpy.node.Node。
        用于创建一个 ROS 2 节点，该节点专门用于订阅机器人的手部 RGB 相机数据。
        它同时订阅相机的元数据 (CameraInfo) 和图像数据 (Image)。
    """

    def __init__(self):
        """
        函数名称: __init__

        功能:
            类的构造函数（初始化方法）。
            初始化父类 Node，并创建两个订阅者（Subscriber）
            来监听指定的话题。

        参数:
            无
        """

        # 调用父类 (Node) 的构造函数，初始化节点名称
        super().__init__(NODE_NAME)

        # 创建左手相机信息订阅者
        self.sub_left_info = self.create_subscription(
            CameraInfo,
            HAND_LEFT_RGB_INFO_TOPIC,
            self.hand_left_rgb_info_callback,
            qos_profile_system_default
        )

        # 创建左手图像订阅者
        self.sub_left_image = self.create_subscription(
            Image,
            HAND_LEFT_RGB_IMAGE_TOPIC,
            self.hand_left_rgb_image_callback,
            qos_profile_sensor_data
        )

        # 创建右手相机信息订阅者
        self.sub_right_info = self.create_subscription(
            CameraInfo,
            HAND_RIGHT_RGB_INFO_TOPIC,
            self.hand_right_rgb_info_callback,
            qos_profile_system_default
        )

        # 创建右手图像订阅者
        self.sub_right_image = self.create_subscription(
            Image,
            HAND_RIGHT_RGB_IMAGE_TOPIC,
            self.hand_right_rgb_image_callback,
            qos_profile_sensor_data
        )

    def hand_left_rgb_info_callback(self, msg):
        """
        函数名称: hand_left_rgb_info_callback

        功能:
            相机信息话题的回调函数。
            当节点接收到左手 RGB 相机 camera_info
            话题消息时触发。

        参数:
            msg (sensor_msgs.msg.CameraInfo):
                包含相机内参、失真模型和分辨率等信息。
        """

        self.get_logger().info(
            f"Hand Left RGB camera info "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def hand_left_rgb_image_callback(self, msg):
        """
        函数名称: hand_left_rgb_image_callback

        功能:
            相机图像话题的回调函数。
            当节点接收到左手 RGB 相机 image_raw
            话题消息时触发。

        参数:
            msg (sensor_msgs.msg.Image):
                包含原始图像数据和分辨率信息。
        """

        self.get_logger().info(
            f"Hand Left RGB camera image "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def hand_right_rgb_info_callback(self, msg):
        """
        函数名称: hand_right_rgb_info_callback

        功能:
            相机信息话题的回调函数。
            当节点接收到右手 RGB 相机 camera_info
            话题消息时触发。

        参数:
            msg (sensor_msgs.msg.CameraInfo):
                包含相机内参、失真模型和分辨率等信息。
        """

        self.get_logger().info(
            f"Hand Right RGB camera info "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def hand_right_rgb_image_callback(self, msg):
        """
        函数名称: hand_right_rgb_image_callback

        功能:
            相机图像话题的回调函数。
            当节点接收到右手 RGB 相机 image_raw
            话题消息时触发。

        参数:
            msg (sensor_msgs.msg.Image):
                包含原始图像数据和分辨率信息。
        """

        self.get_logger().info(
            f"Hand Right RGB camera image "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )


def main(args=None):
    """
    函数名称: main

    功能:
        程序入口。
        初始化 ROS 2，实例化节点，并保持运行。

    参数:
        args (list, optional):
            命令行参数列表。
    """

    # 初始化 ROS 2 通信层
    rclpy.init(args=args)

    # 实例化节点
    node = HandRgbCameraNode()

    try:
        # 保持节点运行
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 捕获 Ctrl+C
        pass

    except RuntimeError:
        # rclpy(Humble) 的 Ctrl+C 恰好落在取消息瞬间时会抛 RuntimeError
        if rclpy.ok():
            raise

    finally:
        # 清理资源
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
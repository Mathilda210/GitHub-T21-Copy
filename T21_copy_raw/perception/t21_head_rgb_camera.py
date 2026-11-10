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
NODE_NAME = "head_rgb_camera_test_node"

# 獲取環境變量，如果沒有讀取到，預設使用 'robot1'
ROBOT_ID = os.environ.get('ROBOT_ID', 'robot1')

HEAD_RGB_INFO_TOPIC = (
    f"/{ROBOT_ID}/forehead_video/camera_info"
)

HEAD_RGB_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/forehead_video/image_raw"
)


class HeadRgbCameraNode(Node):
    """
    类名称: HeadRgbCameraNode

    功能:
        继承自 rclpy.node.Node。
        用于创建一个 ROS 2 节点，该节点专门用于订阅机器人的头部 RGB 相机数据。
        它同时订阅相机的元数据 (CameraInfo) 和图像数据 (Image)。
    """

    def __init__(self):
        """
        函数名称: __init__

        功能:
            类的构造函数 (初始化方法)。
            初始化父类 Node，并创建两个订阅者来监听指定的话题。

        参数:
            无
        """

        # 调用父类 (Node) 的构造函数，初始化节点名称
        super().__init__(NODE_NAME)

        # 创建相机信息的订阅者
        self.sub_info = self.create_subscription(
            CameraInfo,
            HEAD_RGB_INFO_TOPIC,
            self.rgb_info_callback,
            qos_profile_system_default
        )

        # 创建图像数据的订阅者
        self.sub_image = self.create_subscription(
            Image,
            HEAD_RGB_IMAGE_TOPIC,
            self.rgb_image_callback,
            qos_profile_sensor_data
        )

    def rgb_info_callback(self, msg):
        """
        函数名称: rgb_info_callback

        功能:
            相机信息话题的回调函数。
            当节点接收到 camera_info 话题的消息时触发。
            它会在日志中打印相机的分辨率信息。

        参数:
            msg (sensor_msgs.msg.CameraInfo):
                包含相机内参、失真模型和分辨率信息的消息对象。
        """

        self.get_logger().info(
            f"RGB camera info height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def rgb_image_callback(self, msg):
        """
        函数名称: rgb_image_callback

        功能:
            相机图像话题的回调函数。
            当节点接收到 image_raw 话题的消息时触发。
            它会在日志中打印图像的分辨率信息。

        参数:
            msg (sensor_msgs.msg.Image):
                包含原始图像数据、编码格式、步长和分辨率信息的消息对象。
        """

        self.get_logger().info(
            f"RGB camera image height=[{msg.height}] "
            f"width=[{msg.width}]"
        )


def main(args=None):
    """
    函数名称: main

    功能:
        程序的入口点。
        负责初始化 ROS 2 客户端库，实例化节点，
        并保持节点运行直到被中断。

    参数:
        args (list, optional):
            命令行参数列表，默认为 None。
    """

    # 初始化 ROS 2 通信层
    rclpy.init(args=args)

    # 实例化自定义节点
    node = HeadRgbCameraNode()

    try:
        # 保持节点运行
        rclpy.spin(node)

    except KeyboardInterrupt:
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
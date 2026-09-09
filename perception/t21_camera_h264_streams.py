#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# H264 码流以 CompressedImage 消息发布，因此只需要这一种消息类型
from sensor_msgs.msg import CompressedImage

NODE_NAME = "camera_h264_streams_test_node"

# robot_id 必须从环境变量读取再拼接 topic（实际 topic 带 /ROBOT_ID 前缀，写死会取不到数据）
ROBOT_ID = os.environ.get('ROBOT_ID', 'robot1')

# 相机服务对外发布的全部 H264 码流话题。
# 命名规则：在原图话题后面追加 "/h264"（perception_params.yaml 里 codec 配成 H265 时后缀为 "/h265"）。
# 这些码流由 ORIN 的 NVENC 硬件编码产生，一帧一条消息，msg.data 里是裸 H264 访问单元。
H264_TOPICS = {
    # 三路 USB RGB 相机（头部 / 左手 / 右手）
    "head_rgb": f"/{ROBOT_ID}/forehead_video/image_raw/h264",
    "left_hand_rgb": f"/{ROBOT_ID}/left_hand_video1/image_raw/h264",
    "right_hand_rgb": f"/{ROBOT_ID}/right_hand_video1/image_raw/h264",

    # 头部 RGBD 相机 D435i 的四条流
    "d435i_color": f"/{ROBOT_ID}/D435i_1/color/image_raw/h264",
    "d435i_depth": f"/{ROBOT_ID}/D435i_1/depth/image_rect_raw/h264",
    "d435i_infra1": f"/{ROBOT_ID}/D435i_1/infra1/image_rect_raw/h264",
    "d435i_infra2": f"/{ROBOT_ID}/D435i_1/infra2/image_rect_raw/h264",

    # 腰部 RGBD 相机 ASJ HP60C 的两条流
    "waist_rgb": f"/{ROBOT_ID}/ascamera_hp60c/camera_publisher/rgb0/image/h264",
    "waist_depth": f"/{ROBOT_ID}/ascamera_hp60c/camera_publisher/depth0/image_raw/h264",
}


class CameraH264StreamsNode(Node):
    """
    类名称: CameraH264StreamsNode

    功能:
        订阅相机服务发布的全部 H264 码流。
        与订阅原图 (Image) 的示例相比，区别有三点：
        1. 消息类型是 CompressedImage，msg.data 是编码后的字节流，不能直接当像素用；
        2. msg.format 里带的是编码格式标识，不是像素格式；
        3. 码流必须从关键帧 (IDR) 开始解码才有画面，中途接入会先收到若干无法独立解码的帧。
    """

    def __init__(self):
        super().__init__(NODE_NAME)

        self.frame_count = {}
        self.subs = []

        for name, topic in H264_TOPICS.items():
            self.frame_count[name] = 0

            # 码流是高带宽尽力而为的数据，QoS 必须与发布端一致，用 sensor_data
            self.subs.append(
                self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, n=name: self.stream_callback(n, msg),
                    qos_profile_sensor_data
                )
            )

            self.get_logger().info(f"subscribe [{name}] -> {topic}")

    def stream_callback(self, name, msg):
        """
        函数名称: stream_callback

        功能:
            H264 码流回调。每收到 30 帧打印一次，避免 9 路流同时刷屏。
            len(msg.data) 就是这一帧编码后的字节数，可以直接用来估算码率。
        """
        self.frame_count[name] += 1
        count = self.frame_count[name]

        if count == 1 or count % 30 == 0:
            self.get_logger().info(
                f"[{name}] frames=[{count}] format=[{msg.format}] bytes=[{len(msg.data)}]"
            )


def main(args=None):
    rclpy.init(args=args)

    node = CameraH264StreamsNode()

    try:
        rclpy.spin(node)

    # Ctrl+C 与外部 SIGTERM 都是正常退出路径，不打印堆栈
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
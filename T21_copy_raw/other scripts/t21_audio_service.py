#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from px_audio_msgs.srv import Input

import threading
import time


class DemoNode(Node):

    def __init__(self):
        super().__init__('audio_service_demo_node')

        # 创建订阅者
        self.subscription = self.create_subscription(
            String,
            '/Audio/OUTPUT/Publisher',
            self.topic_callback,
            10
        )

        # 创建 service client
        self.input_client = self.create_client(
            Input,
            '/Audio/InputService/Server'
        )

        self.get_logger().info(
            "Python Demo node started."
        )

        # 启动后台线程
        thread = threading.Thread(
            target=self.background_task
        )
        thread.daemon = True
        thread.start()

    # ---------------- 订阅回调 ----------------

    def topic_callback(self, msg):
        self.get_logger().info(
            f"Demo Received: {msg.data}"
        )

        if not self.input_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().warn(
                "Input service not available."
            )
            return

    # ---------------- Service 请求函数 ----------------

    def send_tts_request(self):
        request = Input.Request()
        request.input = (
            "TTS:这个是我需要进行语音合成的文本"
        )

        future = self.input_client.call_async(
            request
        )

        future.add_done_callback(
            self.input_response_callback
        )

        self.get_logger().info(
            "Sent TTS request"
        )

    def send_volume_request(self):
        request = Input.Request()
        request.input = "VOL:80%"

        future = self.input_client.call_async(
            request
        )

        future.add_done_callback(
            self.input_response_callback
        )

        self.get_logger().info(
            "Sent volume request"
        )

    def send_wakeup_request(self):
        request = Input.Request()
        request.input = "WKU:1"

        future = self.input_client.call_async(
            request
        )

        future.add_done_callback(
            self.input_response_callback
        )

        self.get_logger().info(
            "Sent wakeup request"
        )

    # ---------------- 回调函数 ----------------

    def input_response_callback(self, future):
        try:
            response = future.result()

            self.get_logger().info(
                f"Input Service response: "
                f"{response.input_response}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Input service call failed: {e}"
            )

    # ---------------- 后台线程 ----------------

    def background_task(self):
        time.sleep(3)

        self.send_tts_request()
        time.sleep(6)

        self.send_volume_request()
        time.sleep(6)

        self.send_wakeup_request()
        time.sleep(6)


# ---------------- main ----------------

def main(args=None):
    rclpy.init(args=args)

    node = DemoNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
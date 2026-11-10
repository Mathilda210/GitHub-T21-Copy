#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Subscribe to the robot's H264 camera streams and monitor incoming frames.

The node subscribes to each configured camera topic, counts received frames,
and periodically logs the stream format and encoded frame size.
"""

# ----------------------IMPORTS----------------------
import os
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Custom imports
# H264 streams are published as CompressedImage messages, so only this message type is needed.
from sensor_msgs.msg import CompressedImage

# ----------------------CONSTANTS----------------------
NODE_NAME = "camera_h264_streams_test_node"

# ROBOT_ID must be read from the environment before building the topics.
# The actual topics use a /ROBOT_ID prefix, so hardcoding it would prevent receiving data.
ROBOT_ID = os.environ.get('ROBOT_ID', 'robot1')



# ----------------------H264 STREAM TOPICS----------------------

# List all H264 stream topics published by the camera service.

# Naming rule: append "/h264" to the raw image topic.
# Ex: The raw image topic for the head camera is "/robot1/forehead_video/image_raw/h264".
# If the codec is set to H265 in perception_params.yaml, the suffix is "/h265".
# These streams are encoded by ORIN's NVENC hardware. Each message contains one frame,
# and msg.data contains a raw H264 access unit.

H264_TOPICS = {
    # Three USB RGB cameras (head / left hand / right hand)
    "head_rgb": f"/{ROBOT_ID}/forehead_video/image_raw/h264",
    "left_hand_rgb": f"/{ROBOT_ID}/left_hand_video1/image_raw/h264",
    "right_hand_rgb": f"/{ROBOT_ID}/right_hand_video1/image_raw/h264",

    # Four streams from the head-mounted D435i RGB-D camera
    "d435i_color": f"/{ROBOT_ID}/D435i_1/color/image_raw/h264",
    "d435i_depth": f"/{ROBOT_ID}/D435i_1/depth/image_rect_raw/h264",
    "d435i_infra1": f"/{ROBOT_ID}/D435i_1/infra1/image_rect_raw/h264",
    "d435i_infra2": f"/{ROBOT_ID}/D435i_1/infra2/image_rect_raw/h264",

    # Two streams from the waist-mounted ASJ HP60C RGB-D camera
    "waist_rgb": f"/{ROBOT_ID}/ascamera_hp60c/camera_publisher/rgb0/image/h264",
    "waist_depth": f"/{ROBOT_ID}/ascamera_hp60c/camera_publisher/depth0/image_raw/h264",
}



# ----------------------CAMERA H264 STREAMS NODE----------------------
class CameraH264StreamsNode(Node):
    """
        Class name: CameraH264StreamsNode

    Function:
        Subscribe to all H264 streams published by the camera service.
        Compared with subscribing to raw images (Image), there are three differences:
        1. The message type is CompressedImage. msg.data is an encoded byte stream,
           so it cannot be used directly as pixels.
        2. msg.format contains the encoding format identifier, not the pixel format.
        3. The stream must be decoded from a keyframe (IDR) to produce an image.
           When joining mid-stream, several frames may be received before decoding is possible.
    """

    def __init__(self):
        super().__init__(NODE_NAME)

        # Initialize a frame counter for each stream.
        self.frame_count = {}
        # Initialize a list to hold the subscriptions.
        self.subs = []

        # Subscribe to all H264 streams
        for name, topic in H264_TOPICS.items():
            # Initialize the frame counter for this stream. 
            self.frame_count[name] = 0

            # The stream is high-bandwidth best-effort data. QoS must match the publisher;
            # use sensor_data.
            self.subs.append(
                self.create_subscription(
                    # The message type is CompressedImage (custom), which contains the encoded H264 stream.
                    CompressedImage,
                    # The topic name is the H264 stream topic.
                    topic,
                    # The callback function is a lambda that captures the stream name and passes it to stream_callback.
                    lambda msg, n=name: self.stream_callback(n, msg),

                    # Mini fonction rapide mais si on voulait on pourrait écrire: 
                    # def callback(msg, n=name):
                    #   return self.stream_callback(n, msg)
                    # Et l'appeler ici avec:
                    # self.stream_callback(msg, name))

                    # The QoS profile is sensor_data, which is best-effort and high-bandwidth.
                    qos_profile_sensor_data
                )
            )

            # Log the subscription for debugging purposes.
            self.get_logger().info(f"subscribe [{name}] -> {topic}")


    def stream_callback(self, name, msg):
        """
        Function name: stream_callback

        Function:
            H264 stream callback. Log every 30 frames to avoid flooding the console
            when all nine streams are active.
            len(msg.data) is the number of encoded bytes in the frame and can be used
            to estimate the bitrate.
        """
        self.frame_count[name] += 1
        count = self.frame_count[name]

        if count == 1 or count % 30 == 0:
            self.get_logger().info(
                f"[{name}] frames=[{count}] format=[{msg.format}] bytes=[{len(msg.data)}]"
            )



# ----------------------MAIN FUNCTION----------------------
def main(args=None):
    rclpy.init(args=args)

    node = CameraH264StreamsNode()

    try:
        rclpy.spin(node)

    # Ctrl+C and external SIGTERM are normal exit paths; do not print a traceback.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
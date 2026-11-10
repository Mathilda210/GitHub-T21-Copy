#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS 2 test node that subscribes to the robot's head RGB camera information
# and image topics, then logs the received image dimensions.



# ------ IMPORTS--------------
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    qos_profile_system_default,
)

# Custom imports
from sensor_msgs.msg import CameraInfo, Image


# ------------ CONSTANTS -----------------
# Define constants for the node and topic names.
NODE_NAME = "head_rgb_camera_test_node"

# Read the environment variable; use 'robot1' if it is not set.
ROBOT_ID = os.environ.get('ROBOT_ID', 'robot1')

HEAD_RGB_INFO_TOPIC = (
    f"/{ROBOT_ID}/forehead_video/camera_info"
)

HEAD_RGB_IMAGE_TOPIC = (
    f"/{ROBOT_ID}/forehead_video/image_raw"
)



# ----------------- HeadRGBCamera Node definition -------------------
class HeadRgbCameraNode(Node):
    """
    Class name: HeadRgbCameraNode

    Function:
        Inherits from rclpy.node.Node.
        Creates a ROS 2 node dedicated to subscribing to the robot's head RGB
        camera data.
        Subscribes to both camera information (CameraInfo) and image data
        (Image).
    """

    def __init__(self):
        """
        Function name: __init__

        Function:
            Class constructor.
            Initializes the parent Node and creates two subscribers for the
            specified topics.

        Parameters:
            None
        """

        # Initialize the parent Node with the node name.
        super().__init__(NODE_NAME)

        # Create a subscriber for camera information.
        self.sub_info = self.create_subscription(
            # msg type
            CameraInfo,
            # topic name
            HEAD_RGB_INFO_TOPIC,
            # callback function defined here below
            self.rgb_info_callback,
            # qos profile
            qos_profile_system_default
        )

        # Create a subscriber for image data.
        self.sub_image = self.create_subscription(
            Image,
            HEAD_RGB_IMAGE_TOPIC,
            self.rgb_image_callback,
            qos_profile_sensor_data
        )

    def rgb_info_callback(self, msg):
        """
        Function name: rgb_info_callback

        Function:
            Callback for the camera information topic.
            Triggered when the node receives a message from the camera_info
            topic.
            Logs the camera resolution.

        Parameters:
            msg (sensor_msgs.msg.CameraInfo):
                Message containing the camera intrinsics, distortion model,
                and resolution information.
        """

        self.get_logger().info(
            f"RGB camera info height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def rgb_image_callback(self, msg):
        """
        Function name: rgb_image_callback

        Function:
            Callback for the camera image topic.
            Triggered when the node receives a message from the image_raw
            topic.
            Logs the image resolution.

        Parameters:
            msg (sensor_msgs.msg.Image):
                Message containing the raw image data, encoding, row stride,
                and resolution information.
        """

        self.get_logger().info(
            f"RGB camera image height=[{msg.height}] "
            f"width=[{msg.width}]"
        )



# -------------- MAIN --------------------------
def main(args=None):
    """
    Function name: main

    Function:
        Program entry point.
        Initializes the ROS 2 client library, creates the node,
        and keeps it running until interrupted.

    Parameters:
        args (list, optional):
            Command-line arguments. Defaults to None.
    """

    # Initialize the ROS 2 communication layer.
    rclpy.init(args=args)

    # Create the custom node.
    node = HeadRgbCameraNode()

    try:
        # Keep the node running.
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except RuntimeError:
        # rclpy (Humble) may raise RuntimeError if Ctrl+C occurs while
        # receiving a message.
        if rclpy.ok():
            raise

    finally:
        # Clean up resources.
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
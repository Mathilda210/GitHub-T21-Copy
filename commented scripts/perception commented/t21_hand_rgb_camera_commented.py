#!/usr/bin/env python3
# -*- coding: utf-8 -*-



# ----------------------IMPORTS----------------------
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    qos_profile_system_default,
)

# Custom imports
from sensor_msgs.msg import CameraInfo, Image



# ----------------------CONSTANTS----------------------
# Define constants: node name and topic names
NODE_NAME = "hand_rgb_camera_test_node"

# Read the environment variable; use 'robot1' by default if it is not set.
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



# ----------------------HAND RGB CAMERA NODE----------------------
class HandRgbCameraNode(Node):
    """
    Class name: HandRgbCameraNode

    Function:
        Inherits from rclpy.node.Node.
        Creates a ROS 2 node dedicated to subscribing to the robot's hand RGB camera data.
        It subscribes to both camera metadata (CameraInfo) and image data (Image).
    """

    def __init__(self):
        """
        Function name: __init__

        Function:
            Class constructor.
            Initializes the parent Node and creates subscribers
            to monitor the specified topics.

        Parameters:
            None
        """

        # Call the parent Node constructor and initialize the node name.
        super().__init__(NODE_NAME)

        # Create the left-hand camera information subscriber.
        self.sub_left_info = self.create_subscription(
            CameraInfo,
            HAND_LEFT_RGB_INFO_TOPIC,
            self.hand_left_rgb_info_callback,
            qos_profile_system_default
        )

        # Create the left-hand image subscriber.
        self.sub_left_image = self.create_subscription(
            Image,
            HAND_LEFT_RGB_IMAGE_TOPIC,
            self.hand_left_rgb_image_callback,
            qos_profile_sensor_data
        )

        # Create the right-hand camera information subscriber.
        self.sub_right_info = self.create_subscription(
            CameraInfo,
            HAND_RIGHT_RGB_INFO_TOPIC,
            self.hand_right_rgb_info_callback,
            qos_profile_system_default
        )

        # Create the right-hand image subscriber.
        self.sub_right_image = self.create_subscription(
            Image,
            HAND_RIGHT_RGB_IMAGE_TOPIC,
            self.hand_right_rgb_image_callback,
            qos_profile_sensor_data
        )


    # NB: les callbacks images et infos sont identiques ici
    # C'est l'appel qui les rends différents
    # info est appelé avec un message type CameraInfo 
    # alors que image est appelé avec un message type Image
    def hand_left_rgb_info_callback(self, msg):
        """
        Function name: hand_left_rgb_info_callback

        Function:
            Callback for the camera information topic.
            Triggered when the node receives a message from the left-hand RGB camera's
            camera_info topic.

        Parameters:
            msg (sensor_msgs.msg.CameraInfo):
                Contains the camera intrinsic parameters, distortion model, and resolution.
        """

        self.get_logger().info(
            f"Hand Left RGB camera info "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def hand_left_rgb_image_callback(self, msg):
        """
        Function name: hand_left_rgb_image_callback

        Function:
            Callback for the camera image topic.
            Triggered when the node receives a message from the left-hand RGB camera's
            image_raw topic.

        Parameters:
            msg (sensor_msgs.msg.Image):
                Contains the raw image data and resolution information.
        """

        self.get_logger().info(
            f"Hand Left RGB camera image "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def hand_right_rgb_info_callback(self, msg):
        """
        Function name: hand_right_rgb_info_callback

        Function:
            Callback for the camera information topic.
            Triggered when the node receives a message from the right-hand RGB camera's
            camera_info topic.

        Parameters:
            msg (sensor_msgs.msg.CameraInfo):
                Contains the camera intrinsic parameters, distortion model, and resolution.
        """

        self.get_logger().info(
            f"Hand Right RGB camera info "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )

    def hand_right_rgb_image_callback(self, msg):
        """
        Function name: hand_right_rgb_image_callback

        Function:
            Callback for the camera image topic.
            Triggered when the node receives a message from the right-hand RGB camera's
            image_raw topic.

        Parameters:
            msg (sensor_msgs.msg.Image):
                Contains the raw image data and resolution information.
        """

        self.get_logger().info(
            f"Hand Right RGB camera image "
            f"height=[{msg.height}] "
            f"width=[{msg.width}]"
        )




# ----------------- MAIN -------------------------------
def main(args=None):
    """
    Function name: main

    Function:
        Program entry point.
        Initializes ROS 2, creates the node, and keeps it running.

    Parameters:
        args (list, optional):
            List of command-line arguments.
    """

    # Initialize the ROS 2 communication layer.
    rclpy.init(args=args)

    # Create the node instance.
    node = HandRgbCameraNode()

    try:
        # Keep the node running.
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Handle Ctrl+C.
        pass

    except RuntimeError:
        # rclpy (Humble) may raise RuntimeError if Ctrl+C occurs while retrieving a message.
        if rclpy.ok():
            raise

    finally:
        # Clean up resources.
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
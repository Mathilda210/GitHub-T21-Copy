#!/usr/bin/env python3
"""
ROS 2 Video Ring Buffer Node

Description:
    This node subscribes to a ROS 2 CompressedImage topic and maintains
    a circular buffer containing the most recent 10 seconds of video.

    The compressed image data is stored in memory without being decoded
    until a video export is requested. This reduces processing overhead
    during normal operation.

    Video export can be triggered in two ways:
        1. By calling the '~/save_video' ROS 2 service.
        2. Automatically when the node shuts down.

    The exported video is saved as an MP4 file in the current working
    directory. The output filename includes a timestamp.

Configuration:
    - Robot ID: Read from the ROBOT_ID environment variable
                 (defaults to 'robot1').
    - Camera topic: /<robot_id>/left_hand_video1/image_raw/compressed
    - Buffer duration: 10 seconds
    - Expected frame rate: 30 FPS
    - Output format: MP4 using the mp4v codec

Requirements:
    - rclpy
    - sensor_msgs
    - std_srvs
    - OpenCV (cv2)
    - NumPy
"""

import datetime
import os
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger


class VideoRingBufferNode(Node):
    """Subscribe to compressed images and export the latest video frames."""

    def __init__(self):
        super().__init__("video_ring_buffer_node_left_hand")

        # Configure the robot ID and camera topic.
        self.robot_id = os.environ.get("ROBOT_ID", "robot1")
        self.topic_name = (
            f"/{self.robot_id}/left_hand_video1/image_raw/compressed"
        )

        # Configure the video buffer.
        self.fps = 30
        self.duration_sec = 10
        self.max_frames = self.fps * self.duration_sec
        self.buffer = deque(maxlen=self.max_frames)

        # Subscribe to the compressed image topic using sensor-data QoS.
        self.sub = self.create_subscription(
            CompressedImage,
            self.topic_name,
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        # Create a service to export the buffered video on demand.
        self.srv = self.create_service(
            Trigger,
            "~/save_video",
            self.save_service_callback,
        )

        self.get_logger().info(
            f"Video ring buffer started. Listening to: {self.topic_name}"
        )
        self.get_logger().info(
            f"Buffer capacity: {self.duration_sec} seconds "
            f"({self.max_frames} frames)"
        )
        self.get_logger().info(
            "The buffered video will be saved automatically on shutdown."
        )

    def image_callback(self, msg):
        """Store incoming compressed image data in the circular buffer."""
        self.buffer.append(bytes(msg.data))

    def save_video(self):
        """
        Decode the buffered images and export them to an MP4 file.

        Returns:
            tuple[bool, str]: Whether the export succeeded and a status message.
        """
        if not self.buffer:
            return False, "The video buffer is empty; no frames to save."

        # Copy the buffer to obtain a consistent snapshot for export.
        frames_to_save = list(self.buffer)

        # Decode the first frame to determine the video resolution.
        first_frame_data = np.frombuffer(frames_to_save[0], dtype=np.uint8)
        first_frame = cv2.imdecode(first_frame_data, cv2.IMREAD_COLOR)

        if first_frame is None:
            return False, "Failed to decode the first image."

        height, width = first_frame.shape[:2]

        # Generate a timestamped output filename.
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"left_hand_last_10s_{timestamp}.mp4"
        abs_path = os.path.abspath(filename)

        # Initialize the MP4 video writer.
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(
            abs_path,
            fourcc,
            self.fps,
            (width, height),
        )

        if not out.isOpened():
            out.release()
            return False, f"Failed to open the video output file: {abs_path}"

        self.get_logger().info(
            f"Exporting {len(frames_to_save)} buffered frames to {abs_path}"
        )

        valid_frames = 0

        try:
            for frame_data in frames_to_save:
                # Decode each compressed image before writing it to the video.
                image_array = np.frombuffer(frame_data, dtype=np.uint8)
                frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

                if frame is None:
                    continue

                # Ensure all frames match the output video resolution.
                if frame.shape[1] != width or frame.shape[0] != height:
                    frame = cv2.resize(frame, (width, height))

                out.write(frame)
                valid_frames += 1
        finally:
            # Release the video writer to finalize the MP4 file.
            out.release()

        if valid_frames == 0:
            return False, "No valid frames could be written to the video."

        message = (
            f"Video saved successfully: {abs_path} "
            f"({valid_frames} frames)"
        )
        self.get_logger().info(message)

        return True, message

    def save_service_callback(self, request, response):
        """Handle requests to export the buffered video."""
        success, message = self.save_video()
        response.success = success
        response.message = message
        return response


def main(args=None):
    """Initialize ROS 2, run the node, and save the video on shutdown."""
    # Check whether compressed image publishing is enabled on the camera.
    print(
        "\nWARNING: Ensure that 'ros_compressed_topic_enabled' is enabled "
        "in the camera configuration.",
        flush=True,
    )

    rclpy.init(args=args)
    node = VideoRingBufferNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested.")
    finally:
        # Save the buffered video before destroying the node.
        if node.buffer:
            node.save_video()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
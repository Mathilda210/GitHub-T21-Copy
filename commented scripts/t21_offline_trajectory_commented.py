# This script creates a ROS 2 node that periodically publishes 
# offline joint-trajectory (offline=pre-computed) commands for the left arm, right arm, 
# and waist to the /external/offline_motion_command topic, 
# then stops the timers once all commands have been sent.




#--------------------------- IMPORTS ---------------------------
import rclpy
from rclpy.node import Node
# standard ROS 2 message type for representing durations of time
from builtin_interfaces.msg import Duration
import numpy as np
# custom interfaces for offline motion commands and trajectory points
from px_mc_msgs.msg import OfflineMotionCommand, TrajectoryArray, TrajectoryPoint



#--------------------------- PUBLISHER NODE CLASS ---------------------------
class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        self.publisher = self.create_publisher(
            # message type
            OfflineMotionCommand,
            # topic name
            '/external/offline_motion_command',
            # queue size for the publisher
            10
        )

        # Flags to track whether each arm's trajectory has been sent, 
        # each flag is initially set to False
        self.sent_right = False
        self.sent_left = False
        self.sent_waist = False

        # Create timers to periodically call the publish methods for each arm's trajectory
        # The methods are defined below in this script 
        # Wait for 2.9s and then call the publish_right_arm_trajectory method
        self.timer_right = self.create_timer(
            2.9, self.publish_right_arm_trajectory
        )
        # Wait for 3.5s and then call the publish_left_arm_trajectory method
        self.timer_left = self.create_timer(
            3.5, self.publish_left_arm_trajectory
        )
        # Wait for 4.0s and then call the publish_waist_arm_trajectory method
        self.timer_waist = self.create_timer(
            4.0, self.publish_waist_arm_trajectory
        )


    # Method to generate a simple joint trajectory point for the specified arm index (0=left, 1=right)
    def generate_trajectory(self, arm_idx, num_joints=7, steps=11, start=0.0, end=1.0):
        # Create a new empty TrajectoryPoint message (custom)
        point = TrajectoryPoint()
        # time_from_start is a Duration message that specifies how long it should take to 
        # reach this trajectory point from the start of the trajectory
        # Here we set it to 1 second 
        point.time_from_start = Duration(sec=1, nanosec=0)
        # Initialize the position array with 0.1 for all joints 
        # NB: in python you can multiply a list by an integer to repeat its elements
        # so [0.1] * 7 creates [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        # I suppose these are joint position in radians ? 
        point.positions = [0.1] * num_joints
        # Initialize the velocity array with 0.0 for all joints
        point.velocities = [0.0] * num_joints

        # Set the 1st joint to 0.6 rad and the 4th joint to -1.57 rad for both arms 
        point.positions[0] = 0.6
        point.positions[3] = -1.57

        # If the arm index is 0 (left arm), set the 2nd joint to 0.5 rad and the 6th joint to 0.0 rad
        if arm_idx == 0:
            point.positions[1] = 0.5
            point.positions[5] = 0.0
            # So the final position array is : [0.1, 0.5, 0.1, -1.57, 0.1, 0.0, 0.1]

        # If the arm index is 1 (right arm), set the 2nd joint to -0.5 rad and the 6th joint to -0.0 rad
        elif arm_idx == 1:
            point.positions[1] = -0.5
            point.positions[5] = -0.0
            # So the final position array is : [0.1, -0.5, 0.1, -1.57, 0.1, -0.0, 0.1]

        return point


    # Method to generate a simple joint trajectory point for both arms in a "home" position
    def generate_home_trajectory(self, num_joints=7, steps=11, start=0.0, end=1.0):
        point = TrajectoryPoint()
        point.time_from_start = Duration(sec=1, nanosec=0)
        point.positions = [0.0] * num_joints
        point.velocities = [0.0] * num_joints
        return point

    # Method to generate a simple joint trajectory point for the waist_arm (6 joints for legged/waist_arm)
    # num_joints represent the number of joints to use 
    def generate_waist_trajectory(self, num_joints=6):
        """Simple waist_arm joint trajectory point (6 joints for legged/waist_arm)."""

        point = TrajectoryPoint()
        point.time_from_start = Duration(sec=1, nanosec=0)
        point.positions = [0.0] * num_joints
        point.velocities = [0.0] * num_joints

        # Example: small symmetric motion on first two joints and slight bend on others
        # The script is adaptative to the number of joints specified by num_joints
        if num_joints >= 1:
            point.positions[0] = 0.0
            # positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if num_joints >= 2:
            point.positions[1] = 0.0
            # positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if num_joints >= 3:
            point.positions[2] = 0.5
            # positions = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
        if num_joints >= 4:
            point.positions[3] = -0.5
            # positions = [0.0, 0.0, 0.5, -0.5, 0.0, 0.0]
        if num_joints >= 5:
            point.positions[4] = 0.15
            # positions = [0.0, 0.0, 0.5, -0.5, 0.15, 0.0]
        if num_joints >= 6:
            point.positions[5] = 0.0
            # positions = [0.0, 0.0, 0.5, -0.5, 0.15, 0.0]

        return point

    def publish_left_arm_trajectory(self):
        msg = OfflineMotionCommand()
        msg.type = OfflineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = [0]
        msg.async_trajectory = [self.generate_trajectory(0)]
        msg.reference_frame = "world"
        msg.command_id = 1
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

        print(msg.async_trajectory)
        print(len(msg.async_trajectory))

        self.get_logger().info("Publish LEFT arm trajectory")

        self.sent_left = True
        self.maybe_destroy_timers()

    def publish_right_arm_trajectory(self):
        msg = OfflineMotionCommand()
        msg.type = OfflineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = [1]
        msg.async_trajectory = [self.generate_trajectory(1)]
        msg.reference_frame = "world"
        msg.command_id = 2
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

        print(msg.async_trajectory)
        print(len(msg.async_trajectory))

        self.get_logger().info("Publish RIGHT arm trajectory")

        self.sent_right = True
        self.maybe_destroy_timers()

    def publish_waist_arm_trajectory(self):
        """Publish a simple offline joint trajectory command for waist_arm."""

        msg = OfflineMotionCommand()
        msg.type = OfflineMotionCommand.JOINT_TRAJECTORY

        # waist_arm index is 2 in SystemStateMachine / RobotData
        msg.target_arm_indices = [2]

        # For legged/waist_arm, send 6 joints as a test trajectory
        msg.async_trajectory = [
            self.generate_waist_trajectory(num_joints=6)
        ]

        msg.reference_frame = "world"
        msg.command_id = 7
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

        print(msg.async_trajectory)
        print(len(msg.async_trajectory))

        self.get_logger().info("Publish WAIST arm trajectory")

        self.sent_waist = True
        self.maybe_destroy_timers()

    def maybe_destroy_timers(self):
        """Cancel timers once all configured commands have been sent, independent of order."""

        if self.sent_right and self.sent_left and self.sent_waist:
            self.destroy_timers()

    def publish_sync_trajectory(self):
        msg = OfflineMotionCommand()
        msg.type = OfflineMotionCommand.JOINT_TRAJECTORY
        msg.sync_arm_indices = [0, 1]
        msg.sync_trajectory = [
            self.generate_trajectory(0),
            self.generate_trajectory(1)
        ]
        msg.reference_frame = "world"
        msg.command_id = 3

        self.publisher.publish(msg)

        print(msg.sync_trajectory)
        print(len(msg.sync_trajectory))

        self.get_logger().info(
            "Publish LEFT and RIGHT arm sync trajectory"
        )

    def publish_home_trajectory(self):
        msg = OfflineMotionCommand()
        msg.type = OfflineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = [0, 1]
        msg.async_trajectory = [
            self.generate_home_trajectory(),
            self.generate_home_trajectory()
        ]
        msg.reference_frame = "world"
        msg.command_id = 4

        self.publisher.publish(msg)

        print(msg.async_trajectory)
        print(len(msg.async_trajectory))

        self.get_logger().info(
            "Publish LEFT and RIGHT arm home trajectory"
        )

    def publish_sync_home_trajectory(self):
        msg = OfflineMotionCommand()
        msg.type = OfflineMotionCommand.JOINT_TRAJECTORY
        msg.sync_arm_indices = [0, 1]
        msg.sync_trajectory = [
            self.generate_home_trajectory(),
            self.generate_home_trajectory()
        ]
        msg.reference_frame = "world"
        msg.command_id = 4

        self.publisher.publish(msg)

        print(msg.sync_trajectory)
        print(len(msg.sync_trajectory))

        self.get_logger().info(
            "Publish LEFT and RIGHT arm home trajectory"
        )

    def destroy_timers(self):
        """Clean up all timers"""

        self.timer_left.cancel()
        self.timer_right.cancel()
        self.timer_waist.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()                       
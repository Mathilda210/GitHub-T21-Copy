#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px_mc_msgs.msg import (
    OnlineMotionCommand,
    TrajectoryPoint,
    ArmConstraintParameters,
)
from rclpy.executors import MultiThreadedExecutor


class OnlineTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('online_trajectory_publisher')

        self.publisher = self.create_publisher(
            OnlineMotionCommand,
            '/external/online_motion_command',
            10
        )

        self.arm_indices = [0, 1, 2]

        # self.target_positions = {
        #     0: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     1: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     2: [0.0, 0.0, 0.0, 0.0],
        #     3: [0.0, 0.0],
        #     4: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     5: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # }

        self.target_positions = {
            0: [0.0, 0.6, 0.0, -1.57, 0.0, -0.3, 0.0],
            1: [0.0, -0.6, 0.0, -1.57, 0.0, 0.3, 0.0],
            2: [0.0, 0.0, 0.2, -0.2, 0.0, 0.0],
            3: [0.3, 0.3],
            4: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            5: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }

        # Publish command once after a short delay to ensure system is ready
        self.publish_timer = self.create_timer(
            1.0,
            self.publish_command
        )
        self.command_sent = False

    def publish_command(self):
        if self.command_sent:
            return

        msg = OnlineMotionCommand()
        msg.type = OnlineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = self.arm_indices

        # Create target points for each arm
        target_points = []

        arm_names = {
            0: "Left Arm",
            1: "Right Arm",
            2: "Waist Arm",
            3: "Head Arm",
            4: "Left Hand",
            5: "Right Hand"
        }

        for arm_idx in self.arm_indices:
            point = TrajectoryPoint()
            point.positions = self.target_positions[arm_idx]
            point.velocities = [0.0] * len(point.positions)

            target_points.append(point)

            self.get_logger().info(
                f"{arm_names[arm_idx]} target positions: "
                f"{[f'{p:.3f}' for p in point.positions]}"
            )

        msg.target_point = target_points
        msg.planner = OnlineMotionCommand.PLANNER_RUCKIG

        # Set constraint parameters per arm
        # Format: ArmConstraintParameters array,
        # one per target_arm_indices
        msg.max_velocity = []
        msg.max_acceleration = []
        msg.max_jerk = []

        for arm_idx in self.arm_indices:
            if arm_idx in [0, 1]:
                # Left or right arm (7 joints)
                # Single value for all joints
                msg.max_velocity.append(
                    ArmConstraintParameters(values=[0.4])
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(values=[0.5])
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(values=[5.0])
                )

            elif arm_idx == 2:
                # Waist arm (6 joints)
                # Per-joint values
                msg.max_velocity.append(
                    ArmConstraintParameters(
                        values=[0.1, 0.1, 0.05, 0.15, 0.13, 0.14]
                    )
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(
                        values=[0.4, 0.5, 0.1, 0.4, 0.5, 0.3]
                    )
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(
                        values=[4.0, 5.0, 3.0, 4.0, 5.0, 3.0]
                    )
                )

            else:
                # Other arms
                msg.max_velocity.append(
                    ArmConstraintParameters(values=[0.02])
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(values=[0.5])
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(values=[5.0])
                )

        # Alternative: Use default values by leaving arrays empty
        # msg.max_velocity = []
        # msg.max_acceleration = []
        # msg.max_jerk = []

        msg.is_whole_body = False
        msg.reference_frame = "world_frame"
        msg.command_id = 1
        msg.need_interpolation = True
        msg.is_urgent = False
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

        self.command_sent = True

        self.get_logger().info(
            f"Published online motion command for arms "
            f"{self.arm_indices}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = OnlineTrajectoryPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()                            
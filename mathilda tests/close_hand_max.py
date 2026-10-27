#!/usr/bin/env python3
"""Open both T21 hands, close them, pause, and open them again.

The hand targets and joint order below follow the existing
``other scripts/pick_and_place.py`` example. Run this from a ROS 2 environment
with the robot's motion controller and joint-state topic available.
"""

import argparse
import math
import os
import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from px_mc_msgs.msg import (
    ArmConstraintParameters,
    OnlineMotionCommand,
    TrajectoryPoint,
)
from sensor_msgs.msg import JointState


# The hand groups in OnlineMotionCommand are five fingers with three joints
# each, in index, middle, ring, pinky, and thumb order.
HAND_JOINTS = {
    4: [
        f"left_{finger}_joint_{joint}"
        for finger in ("index", "middle", "ring", "pinky", "thumb")
        for joint in range(3)
    ],
    5: [
        f"right_{finger}_joint_{joint}"
        for finger in ("index", "middle", "ring", "pinky", "thumb")
        for joint in range(3)
    ],
}

# All-zero hand positions are used as the open pose in the project's existing
# pick-and-place targets. The repository does not specify hardware joint limits.
OPEN_POSITION = [0.0] * 15

# This closed-hand pose is taken from the existing right-hand grasp target and
# mirrored to the left hand. Confirm that these example positions are suitable
# for the specific robot and hand calibration before running the script.
CLOSED_POSITION = [
    1.3, 0.0, 0.0,
    1.3, 0.0, 0.0,
    1.3, 0.0, 0.0,
    1.3, 0.0, 0.0,
    1.5697, 0.6, 0.95,
]

POSITION_TOLERANCE = 0.05
VELOCITY_TOLERANCE = 0.03
MOTION_TIMEOUT_SECONDS = 30.0
SUBSCRIBER_TIMEOUT_SECONDS = 10.0
HOLD_SECONDS = 5


class TwoHandSequence(Node):
    """Publish hand targets and wait until joint feedback confirms each one."""

    def __init__(self, joint_state_topic: str):
        super().__init__("open_close_both_hands")

        self.publisher = self.create_publisher(
            OnlineMotionCommand,
            "/external/online_motion_command",
            10,
        )
        self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            20,
        )

        self.joint_state_topic = joint_state_topic
        self.command_id = 0
        self.latest_positions: Optional[Dict[str, float]] = None
        self.latest_velocities: Optional[Dict[str, float]] = None
        self.last_feedback_warning = 0.0

    def _warn_feedback(self, message: str) -> None:
        """Rate-limit feedback warnings so the console remains readable."""
        now = time.monotonic()
        if now - self.last_feedback_warning >= 3.0:
            self.get_logger().warning(message)
            self.last_feedback_warning = now

    def _joint_state_callback(self, msg: JointState) -> None:
        """Keep valid position and velocity feedback for both hands."""
        if len(msg.name) != len(msg.position):
            self._warn_feedback(
                "JointState name and position arrays have different lengths."
            )
            return
        if len(msg.name) != len(msg.velocity):
            self._warn_feedback(
                "JointState velocity data is missing or incomplete."
            )
            return

        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        required_joints = HAND_JOINTS[4] + HAND_JOINTS[5]
        missing_joints = [name for name in required_joints if name not in positions]
        if missing_joints:
            self._warn_feedback(
                "Waiting for feedback for all hand joints; missing: "
                + ", ".join(missing_joints)
            )
            return

        hand_positions = {name: float(positions[name]) for name in required_joints}
        hand_velocities = {name: float(velocities[name]) for name in required_joints}
        if not all(
            math.isfinite(value)
            for value in (*hand_positions.values(), *hand_velocities.values())
        ):
            self._warn_feedback("JointState contains non-finite hand data.")
            return

        self.latest_positions = hand_positions
        self.latest_velocities = hand_velocities

    def wait_for_controller(self) -> None:
        """Wait briefly for a motion-controller subscriber before publishing."""
        self.get_logger().info(
            f"Waiting for a motion controller on "
            f"{self.publisher.topic_name}..."
        )
        deadline = time.monotonic() + SUBSCRIBER_TIMEOUT_SECONDS
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "No motion controller subscribed to "
                    f"{self.publisher.topic_name} after "
                    f"{SUBSCRIBER_TIMEOUT_SECONDS:.0f} seconds."
                )
            rclpy.spin_once(self, timeout_sec=0.1)

        if not rclpy.ok():
            raise RuntimeError("ROS 2 stopped while waiting for the controller.")
        self.get_logger().info("Motion controller found.")

    def publish_hand_target(self, label: str, position: List[float]) -> None:
        """Send the same 15-joint target to the left and right hands."""
        self.command_id += 1
        msg = OnlineMotionCommand()
        msg.type = OnlineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = [4, 5]
        msg.target_point = []

        for side in ("left", "right"):
            point = TrajectoryPoint()
            point.positions = list(position)
            point.velocities = [0.0] * len(position)
            msg.target_point.append(point)
            self.get_logger().info(
                f"{side.capitalize()} hand {label} target sent "
                f"({len(point.positions)} joints)."
            )

        # Use the same hand motion limits as the project's pick-and-place code.
        msg.max_velocity = [
            ArmConstraintParameters(values=[1.0]),
            ArmConstraintParameters(values=[1.0]),
        ]
        msg.max_acceleration = [
            ArmConstraintParameters(values=[0.8]),
            ArmConstraintParameters(values=[0.8]),
        ]
        msg.max_jerk = [
            ArmConstraintParameters(values=[5.0]),
            ArmConstraintParameters(values=[5.0]),
        ]

        msg.planner = OnlineMotionCommand.PLANNER_RUCKIG
        msg.is_whole_body = False
        msg.reference_frame = "world_frame"
        msg.command_id = self.command_id
        msg.need_interpolation = True
        msg.is_urgent = False
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published command {self.command_id}: both hands {label}."
        )

    def wait_for_hand_target(
        self,
        label: str,
        position: List[float],
    ) -> None:
        """Wait for measured positions and velocities to reach the target."""
        targets = {
            joint_name: target_position
            for arm_index in (4, 5)
            for joint_name, target_position in zip(
                HAND_JOINTS[arm_index], position
            )
        }
        deadline = time.monotonic() + MOTION_TIMEOUT_SECONDS
        next_status_time = 0.0

        while rclpy.ok():
            if self.latest_positions is not None and self.latest_velocities is not None:
                max_position_error = max(
                    abs(self.latest_positions[name] - target)
                    for name, target in targets.items()
                )
                max_velocity = max(
                    abs(self.latest_velocities[name]) for name in targets
                )
                if (
                    max_position_error <= POSITION_TOLERANCE
                    and max_velocity <= VELOCITY_TOLERANCE
                ):
                    self.get_logger().info(
                        f"Both hands reached the {label} target and are still."
                    )
                    return

                if time.monotonic() >= next_status_time:
                    self.get_logger().info(
                        f"Moving to {label}: maximum joint error "
                        f"{max_position_error:.3f}, maximum speed "
                        f"{max_velocity:.3f}."
                    )
                    next_status_time = time.monotonic() + 3.0
            else:
                self._warn_feedback(
                    f"Waiting for complete joint feedback on "
                    f"{self.joint_state_topic}."
                )

            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Both hands did not reach the {label} target within "
                    f"{MOTION_TIMEOUT_SECONDS:.0f} seconds."
                )

            rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError("ROS 2 stopped before the hand motion completed.")

    def wait_before_reopening(self) -> None:
        """Keep the hands closed for five seconds and show a countdown."""
        self.get_logger().info(
            f"Both hands are closed. Holding for {HOLD_SECONDS} seconds."
        )
        end_time = time.monotonic() + HOLD_SECONDS
        last_countdown = None

        while rclpy.ok():
            remaining = max(0, math.ceil(end_time - time.monotonic()))
            if remaining != last_countdown:
                self.get_logger().info(f"Reopening in {remaining}...")
                last_countdown = remaining

            if remaining == 0:
                return

            rclpy.spin_once(self, timeout_sec=min(0.2, end_time - time.monotonic()))

        raise RuntimeError("ROS 2 stopped during the five-second hold.")

    def run_sequence(self) -> None:
        """Run the confirmed open, close, hold, and reopen sequence."""
        self.get_logger().info(
            "Safety check: clear the area around both hands and ensure the "
            "example close pose is suitable for this robot."
        )
        try:
            input("Press Enter when it is safe to move both hands (Ctrl+C to cancel) > ")
        except EOFError as error:
            raise RuntimeError(
                "Interactive safety confirmation is required; no motion was sent."
            ) from error

        self.wait_for_controller()

        self.get_logger().info("STEP 1/3: Opening both hands.")
        self.publish_hand_target("open", OPEN_POSITION)
        self.wait_for_hand_target("open", OPEN_POSITION)

        self.get_logger().info("STEP 2/3: Closing both hands.")
        self.publish_hand_target("closed", CLOSED_POSITION)
        self.wait_for_hand_target("closed", CLOSED_POSITION)
        self.wait_before_reopening()

        self.get_logger().info("STEP 3/3: Reopening both hands.")
        self.publish_hand_target("open", OPEN_POSITION)
        self.wait_for_hand_target("open", OPEN_POSITION)

        self.get_logger().info("Sequence complete: both hands are open.")


def main(args=None) -> int:
    """Initialize ROS 2, run the sequence, and report any failure clearly."""
    parser = argparse.ArgumentParser(
        description="Open, close, hold, and reopen both T21 hands."
    )
    default_robot_id = os.environ.get("ROBOT_ID", "ROBOT_10")
    parser.add_argument(
        "--robot-id",
        default=default_robot_id,
        help=(
            "Robot ID used to build the default joint-state topic "
            f"(default: {default_robot_id})."
        ),
    )
    parser.add_argument(
        "--joint-state-topic",
        default="",
        help="JointState topic; overrides the topic built from --robot-id.",
    )
    parsed_args, ros_args = parser.parse_known_args(args=args)
    joint_state_topic = (
        parsed_args.joint_state_topic.strip()
        or f"/{parsed_args.robot_id}/mc/pr10_joint_states_topic"
    )
    if not joint_state_topic.startswith("/"):
        joint_state_topic = "/" + joint_state_topic

    rclpy.init(args=ros_args)
    node = TwoHandSequence(joint_state_topic)

    try:
        node.get_logger().info(f"Using joint-state topic: {joint_state_topic}")
        node.run_sequence()
        return 0
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted by operator.")
        return 130
    except (RuntimeError, TimeoutError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
#!/usr/bin/env python3

"""Move through the saved pick poses, then close the right hand."""

import argparse
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from px_mc_msgs.msg import (
    ArmConstraintParameters,
    OnlineMotionCommand,
    TrajectoryPoint,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _hand_joints(side: str) -> List[str]:
    fingers = ("index", "middle", "ring", "pinky", "thumb")
    return [
        f"{side}_{finger}_joint_{index}"
        for finger in fingers
        for index in range(3)
    ]


TARGET_JOINT_GROUPS = {
    0: [f"left_arm_joint_{index}" for index in range(7)],
    1: [f"right_arm_joint_{index}" for index in range(7)],
    2: [
        "right_front_leg_joint",
        "left_front_leg_joint",
        "waist_joint_1",
        "waist_joint_2",
        "waist_joint_3",
        "waist_joint_4",
    ],
    3: ["neck_joint", "head_joint_0"],
    4: _hand_joints("left"),
    5: _hand_joints("right"),
}

POSE_PAUSE_SECONDS = 2.0
POSITION_TOLERANCE = 0.05
VELOCITY_TOLERANCE = 0.01
COMPARISON_EPSILON = 1e-9

INITIAL_POSE = {
    0: [0.0, 0.4, 0.0, -1.57, 0.0, 0.0, 0.0],
    1: [0.0, -0.4, 0.0, -1.57, 0.0, 0.0, 0.0],
    2: [0.0] * 6,
    3: [0.0, 0.0],
    4: [
        0.008, 0.0, 0.0,
        0.009, 0.0, 0.0,
        0.004, 0.0, 0.0,
        0.007, 0.0, 0.0,
        0.004, 0.0, 0.0,
    ],
    5: [
        0.004, 0.004, 0.002,
        0.004, 0.005, 0.003,
        0.005, 0.0, 0.0,
        0.001, 0.0, 0.0,
        0.002, 0.001, 0.004,
    ],
}

PREPARE_HAND_POSE = {
    0: [0.0, 0.4, 0.0, -1.57, 0.0, 0.0, 0.0],
    1: [0.0, -0.4, 0.0, -1.57, 0.0, 0.0, 0.0],
    2: [0.0] * 6,
    3: [0.0, -0.001],
    4: [
        0.010, 0.0, 0.0,
        0.019, 0.0, 0.0,
        0.005, 0.0, 0.0,
        0.018, 0.0, 0.0,
        0.0, 0.001, 0.004,
    ],
    5: [
        0.729, 0.0, 0.0,
        0.693, 0.033, 0.017,
        0.698, 0.0, 0.0,
        0.695, 0.011, 0.010,
        1.351, 0.0, 1.076,
    ],
}

HAND_AROUND_OBJECT_POSE = {
    0: [0.0, 0.4, 0.0, -1.57, 0.0, 0.0, 0.0],
    1: [-0.711, -0.282, 0.618, -0.469, -0.054, 0.578, -0.652],
    2: [0.0] * 6,
    3: [0.0, -0.001],
    4: [
        0.010, 0.0, 0.0,
        0.019, 0.0, 0.0,
        0.005, 0.0, 0.0,
        0.018, 0.0, 0.0,
        0.0, 0.001, 0.004,
    ],
    5: [
        0.727, 0.002, 0.004,
        0.685, 0.048, 0.023,
        0.698, 0.002, 0.002,
        0.694, 0.020, 0.013,
        1.351, 0.0, 1.076,
    ],
}

CLOSED_RIGHT_HAND = [
    1.3, 0.0, 0.0,
    1.3, 0.0, 0.0,
    1.3, 0.0, 0.0,
    1.3, 0.0, 0.0,
    1.5697, 0.6, 0.95,
]


class SumanPick(Node):
    """Publish each saved pose and close the right hand without moving the arm."""

    def __init__(self, joint_state_topic: str):
        super().__init__("suman_pick")
        self.publisher = self.create_publisher(
            OnlineMotionCommand,
            "/external/online_motion_command",
            10,
        )
        self.arm_indices = [0, 1, 2, 3, 4, 5]
        self.poses = [
            ("initial pose", INITIAL_POSE),
            ("prepare hand", PREPARE_HAND_POSE),
            ("hand around object", HAND_AROUND_OBJECT_POSE),
        ]
        self.pose_index = 0
        self.phase = "send_pose"
        self.next_action_time = time.monotonic()
        self.finished = False
        self.waiting_target: Optional[Dict[int, List[float]]] = None
        self.waiting_groups: List[int] = []
        self._joint_names = [
            joint_name
            for arm_index in self.arm_indices
            for joint_name in TARGET_JOINT_GROUPS[arm_index]
        ]
        self._latest_positions: Optional[Dict[str, float]] = None
        self._latest_velocities: Optional[Dict[str, float]] = None
        self._last_feedback_warning = 0.0
        self.cmd_id = 0

        self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            20,
        )
        self.timer = self.create_timer(0.1, self._advance)
        self.get_logger().info(f"Joint-state feedback: {joint_state_topic}")

    def _joint_state_callback(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
            self._warn_feedback(
                "Waiting for valid JointState position and velocity data."
            )
            return

        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        missing = [name for name in self._joint_names if name not in positions]
        if missing:
            self._warn_feedback(
                "Waiting for complete JointState; missing: " + ", ".join(missing)
            )
            return

        self._latest_positions = {
            name: float(positions[name]) for name in self._joint_names
        }
        self._latest_velocities = {
            name: float(velocities[name]) for name in self._joint_names
        }

    def _warn_feedback(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_feedback_warning >= 3.0:
            self.get_logger().warning(message)
            self._last_feedback_warning = now

    def _target_reached(self) -> Tuple[bool, float, float]:
        if (
            self._latest_positions is None
            or self._latest_velocities is None
            or self.waiting_target is None
        ):
            self._warn_feedback("Waiting for joint position and velocity feedback.")
            return False, float("inf"), float("inf")

        max_position_error = 0.0
        max_velocity = 0.0
        for arm_index in self.waiting_groups:
            for joint_name, target_position in zip(
                TARGET_JOINT_GROUPS[arm_index],
                self.waiting_target[arm_index],
            ):
                max_position_error = max(
                    max_position_error,
                    abs(self._latest_positions[joint_name] - target_position),
                )
                max_velocity = max(
                    max_velocity,
                    abs(self._latest_velocities[joint_name]),
                )

        reached = (
            max_position_error <= POSITION_TOLERANCE + COMPARISON_EPSILON
            and max_velocity <= VELOCITY_TOLERANCE + COMPARISON_EPSILON
        )
        if not reached:
            self._warn_feedback(
                f"Waiting for target: position error {max_position_error:.4f}, "
                f"velocity {max_velocity:.4f}."
            )
        return reached, max_position_error, max_velocity

    def _advance(self) -> None:
        now = time.monotonic()
        if self.phase == "await_target":
            reached, position_error, velocity = self._target_reached()
            if not reached:
                return

            if self.pose_index < len(self.poses):
                pose_name = self.poses[self.pose_index][0]
                self.get_logger().info(
                    f"Reached {pose_name}: position error {position_error:.4f}, "
                    f"velocity {velocity:.4f}."
                )
                self.phase = "pose_pause"
                self.next_action_time = now + POSE_PAUSE_SECONDS
            else:
                self.get_logger().info("Right hand closed; pick sequence complete.")
                self.finished = True
                self.timer.cancel()
            return

        if now + COMPARISON_EPSILON < self.next_action_time:
            return

        if self.phase == "pose_pause":
            self.pose_index += 1
            if self.pose_index == len(self.poses):
                self.phase = "close_hand"
            else:
                self.phase = "send_pose"

        if self.phase == "send_pose":
            pose_name, target = self.poses[self.pose_index]
            self._publish_target(target, self.arm_indices)
            self.get_logger().info(f"Moving to {pose_name}.")
            self.waiting_target = target
            self.waiting_groups = self.arm_indices
            self.phase = "await_target"
            return

        if self.phase == "close_hand":
            hand_target = {5: CLOSED_RIGHT_HAND}
            self._publish_target(hand_target, [5])
            self.get_logger().info(
                "Closing the right hand while keeping the arm at hand around object."
            )
            self.waiting_target = hand_target
            self.waiting_groups = [5]
            self.phase = "await_target"

    def _publish_target(
        self,
        target: Dict[int, List[float]],
        group_indices: List[int],
    ) -> None:
        msg = OnlineMotionCommand()
        msg.type = OnlineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = group_indices

        target_points = []
        for arm_index in group_indices:
            point = TrajectoryPoint()
            point.positions = target[arm_index]
            point.velocities = [0.0] * len(point.positions)
            target_points.append(point)

        msg.target_point = target_points
        msg.planner = OnlineMotionCommand.PLANNER_RUCKIG
        msg.max_velocity = []
        msg.max_acceleration = []
        msg.max_jerk = []

        for arm_index in group_indices:
            if arm_index in (0, 1):
                msg.max_velocity.append(ArmConstraintParameters(values=[0.5]))
                msg.max_acceleration.append(ArmConstraintParameters(values=[0.5]))
                msg.max_jerk.append(ArmConstraintParameters(values=[5.0]))
            elif arm_index == 2:
                msg.max_velocity.append(
                    ArmConstraintParameters(values=[0.3, 0.3, 0.3, 0.3, 0.13, 0.14])
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(values=[0.4, 0.5, 0.1, 0.4, 0.5, 0.3])
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(values=[4.0, 5.0, 3.0, 4.0, 5.0, 3.0])
                )
            elif arm_index == 3:
                msg.max_velocity.append(ArmConstraintParameters(values=[0.3]))
                msg.max_acceleration.append(ArmConstraintParameters(values=[0.5]))
                msg.max_jerk.append(ArmConstraintParameters(values=[5.0]))
            else:
                msg.max_velocity.append(ArmConstraintParameters(values=[1.0]))
                msg.max_acceleration.append(ArmConstraintParameters(values=[0.8]))
                msg.max_jerk.append(ArmConstraintParameters(values=[5.0]))

        msg.is_whole_body = False
        msg.reference_frame = "world_frame"
        self.cmd_id += 1
        msg.command_id = self.cmd_id
        msg.need_interpolation = True
        msg.is_urgent = False
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published command {self.cmd_id} for groups {group_indices}."
        )


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Move through the saved pick poses and close the right hand."
    )
    default_robot_id = os.environ.get("ROBOT_ID", "ROBOT_10")
    parser.add_argument(
        "--robot-id",
        default=default_robot_id,
        help=(
            "Robot ID used to build the default JointState topic "
            f"(default: {default_robot_id})."
        ),
    )
    parser.add_argument(
        "--joint-state-topic",
        default="",
        help="JointState feedback topic; overrides --robot-id.",
    )
    parsed_args, _ = parser.parse_known_args(args=args)
    joint_state_topic = (
        parsed_args.joint_state_topic.strip()
        or f"/{parsed_args.robot_id}/mc/pr10_joint_states_topic"
    )
    if not joint_state_topic.startswith("/"):
        joint_state_topic = "/" + joint_state_topic

    rclpy.init(args=args)
    node = SumanPick(joint_state_topic)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
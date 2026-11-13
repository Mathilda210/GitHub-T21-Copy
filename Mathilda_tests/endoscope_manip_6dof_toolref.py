
#!/usr/bin/env python3

"""
Robot grasp endoscope sequence.
Human puts endoscope in robot hand.
Possible movements: 
- translation back and forth 10cm (x,y,z)
- rotation around x,y,z axes 5 degrees
! In the object referential, offset 20 cm above the hand along local Z!
"""

import argparse
import math
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from px_mc_msgs.msg import (
    ArmConstraintParameters,
    OnlineMotionCommand,
    TrajectoryPoint,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


RIGHT_HAND_GROUP = 5
RIGHT_ARM_GROUP = 1
RIGHT_HAND_JOINTS = [
    f"right_{finger}_joint_{index}"
    for finger in ("index", "middle", "ring", "pinky", "thumb")
    for index in range(3)
]
HOME_HAND_POSE = [0.0] * len(RIGHT_HAND_JOINTS)

POSE_PAUSE_SECONDS = 2.0
POSITION_TOLERANCE = 0.1
HAND_CLOSE_MIN_PROGRESS = 0.05
HAND_CLOSE_STALL_SECONDS = 1.5
VELOCITY_TOLERANCE = 0.01
COMPARISON_EPSILON = 1e-9
OBJECT_FRAME_OFFSET = (0.0, 0.0, 0.20)
CARTESIAN_TRANSLATION_METERS = 0.10
CARTESIAN_ROTATION_DEGREES = 5.0
CARTESIAN_POSITION_TOLERANCE = 0.01
CARTESIAN_ORIENTATION_TOLERANCE_RADIANS = math.radians(1.0)
CARTESIAN_SETTLE_SECONDS = 0.5
CARTESIAN_TIMEOUT_SECONDS = 60.0
TF_WAIT_TIMEOUT_SECONDS = 5.0
TF_DIAGNOSTIC_INTERVAL_SECONDS = 2.0
RIGHT_ARM_TF_PARENT = "waist_link_4"
RIGHT_ARM_TF_CHILD = "right_arm_link_6"
TRANSLATION_AXES = {"x": 0, "y": 1, "z": 2}
ROTATION_AXES = {"x": 0, "y": 1, "z": 2}
ONLINE_MOTION_TOPIC = "/external/online_motion_command"

ENDOSCOPE_STEPS: List[Tuple[str, List[float]]] = [
    (
        "pouce referme",
        [
            0.003, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.023, 0.005, 0.0,
            0.004, 0.0, 0.0,
            1.534, 1.189, 1.686,
        ],
    ),
    (
        "index replie sur l'endoscope",
        [
            0.719, 1.049, 0.880,
            0.0, 0.0, 0.0,
            0.023, 0.005, 0.0,
            0.004, 0.0, 0.0,
            1.534, 1.120, 1.838,
        ],
    ),
]


class SumanEndoscopeHold(Node):
    """Apply the measured endoscope grasp sequence and keep its final pose."""

    def __init__(
        self,
        joint_state_topic: str,
        arm_tf_topic: str,
        main_frame: str,
        tool_frame: str,
    ) -> None:
        super().__init__("suman_hold_endoscope")
        self.publisher = self.create_publisher(
            OnlineMotionCommand,
            ONLINE_MOTION_TOPIC,
            10,
        )
        self.steps = ENDOSCOPE_STEPS
        self.home_reset = False
        self.step_index = 0
        self.phase = "send_step"
        self.next_action_time = time.monotonic()
        self.waiting_target: Optional[List[float]] = None
        self._latest_positions: Optional[Dict[str, float]] = None
        self._latest_velocities: Optional[Dict[str, float]] = None
        self._latest_arm_pose: Optional[List[float]] = None
        self._latest_arm_pose_received_at = 0.0
        self._latest_arm_tf_stamp: Optional[Tuple[int, int]] = None
        self._seen_arm_tf_frames: List[Tuple[str, str]] = []
        self._arm_tf_message_count = 0
        self._arm_tf_transform_count = 0
        self._matching_arm_tf_count = 0
        self._last_tf_diagnostic_at = 0.0
        self.arm_tf_topic = arm_tf_topic
        self._last_feedback_warning = 0.0
        self._hand_close_initial_error: Optional[float] = None
        self._hand_close_stall_since: Optional[float] = None
        self._hand_close_progress_detected = False
        self._hand_close_contact_detected = False
        self.cmd_id = 0
        self._publish_tool_frame(main_frame, tool_frame)

        self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            20,
        )
        self.create_subscription(
            TFMessage,
            arm_tf_topic,
            self._arm_tf_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.1, self._advance)
        self.get_logger().info(
            f"Joint-state feedback: {joint_state_topic}; "
            f"arm TF feedback: {arm_tf_topic}; selected instrument: endoscope. "
            f"Expected arm TF: {RIGHT_ARM_TF_PARENT}->{RIGHT_ARM_TF_CHILD}."
        )

    def _publish_tool_frame(self, main_frame: str, tool_frame: str) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = main_frame
        transform.child_frame_id = tool_frame
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = OBJECT_FRAME_OFFSET
        transform.transform.rotation.w = 1.0
        self._static_transform_broadcaster = StaticTransformBroadcaster(self)
        self._static_transform_broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Published TF {main_frame} -> {tool_frame}: "
            f"x={OBJECT_FRAME_OFFSET[0]:.2f} m, "
            f"y={OBJECT_FRAME_OFFSET[1]:.2f} m, "
            f"z={OBJECT_FRAME_OFFSET[2]:.2f} m; "
            "orientation unchanged."
        )
        print(
            "\nRepere main : origine (0, 0, 0)\n"
            f"Repere outil ({tool_frame}) : "
            f"x={OBJECT_FRAME_OFFSET[0]:+.2f} m, "
            f"y={OBJECT_FRAME_OFFSET[1]:+.2f} m, "
            f"z={OBJECT_FRAME_OFFSET[2]:+.2f} m\n"
            "Orientation : identique au repere main.\n"
            "Pour voir les axes dans RViz, ajoutez l'affichage TF."
        )

    def start_endoscope(self) -> None:
        self.steps = ENDOSCOPE_STEPS
        self.home_reset = False
        self.step_index = 0
        self.phase = "send_step"
        self.next_action_time = time.monotonic()
        self.waiting_target = None

    def start_home_reset(self) -> None:
        self.steps = [
            ("retour a la position initiale (doigts ouverts)", HOME_HAND_POSE)
        ]
        self.home_reset = True
        self.step_index = 0
        self.phase = "send_step"
        self.next_action_time = time.monotonic()
        self.waiting_target = None
        self.get_logger().info("Returning the right hand to its open home position.")

    def _joint_state_callback(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
            self._warn_feedback(
                "Waiting for valid JointState position and velocity data."
            )
            return

        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        missing = [name for name in RIGHT_HAND_JOINTS if name not in positions]
        if missing:
            self._warn_feedback(
                "Waiting for complete JointState; missing: " + ", ".join(missing)
            )
            return

        self._latest_positions = {
            name: float(positions[name]) for name in RIGHT_HAND_JOINTS
        }
        self._latest_velocities = {
            name: float(velocities[name]) for name in RIGHT_HAND_JOINTS
        }

    def _arm_tf_callback(self, msg: TFMessage) -> None:
        self._arm_tf_message_count += 1
        self._arm_tf_transform_count += len(msg.transforms)
        if self._arm_tf_message_count == 1:
            self.get_logger().info(
                f"First TFMessage received on {self.arm_tf_topic}: "
                f"{len(msg.transforms)} transform(s)."
            )

        for transform in msg.transforms:
            frame_pair = (
                transform.header.frame_id.strip("/"),
                transform.child_frame_id.strip("/"),
            )
            if (
                frame_pair not in self._seen_arm_tf_frames
                and len(self._seen_arm_tf_frames) < 10
            ):
                self._seen_arm_tf_frames.append(frame_pair)
                self.get_logger().info(
                    f"TF frame observed: {frame_pair[0]}->{frame_pair[1]}."
                )
            if (
                frame_pair[0] == RIGHT_ARM_TF_PARENT
                and frame_pair[1] == RIGHT_ARM_TF_CHILD
            ):
                self._matching_arm_tf_count += 1
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self._latest_arm_pose = [
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                    float(rotation.w),
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                ]
                self._latest_arm_pose_received_at = time.monotonic()
                self._latest_arm_tf_stamp = (
                    int(transform.header.stamp.sec),
                    int(transform.header.stamp.nanosec),
                )
                now = self._latest_arm_pose_received_at
                if (
                    self._matching_arm_tf_count == 1
                    or now - self._last_tf_diagnostic_at
                    >= TF_DIAGNOSTIC_INTERVAL_SECONDS
                ):
                    self.get_logger().info(
                        f"Matching arm TF #{self._matching_arm_tf_count}: "
                        f"{RIGHT_ARM_TF_PARENT}->{RIGHT_ARM_TF_CHILD}, "
                        f"position=({translation.x:.4f}, {translation.y:.4f}, "
                        f"{translation.z:.4f}), "
                        f"quaternion=({rotation.x:.4f}, {rotation.y:.4f}, "
                        f"{rotation.z:.4f}, {rotation.w:.4f}), "
                        f"header_stamp={self._latest_arm_tf_stamp[0]}."
                        f"{self._latest_arm_tf_stamp[1]:09d}."
                    )
                    self._last_tf_diagnostic_at = now

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
            self._warn_feedback("Waiting for right-hand position and velocity data.")
            return False, float("inf"), float("inf")

        errors = [
            abs(self._latest_positions[name] - target)
            for name, target in zip(RIGHT_HAND_JOINTS, self.waiting_target)
        ]
        max_position_error = max(errors)
        total_position_error = sum(errors)
        max_velocity = max(
            abs(self._latest_velocities[name]) for name in RIGHT_HAND_JOINTS
        )

        if self._hand_close_initial_error is None:
            self._hand_close_initial_error = total_position_error
        elif (
            self._hand_close_initial_error - total_position_error
            >= HAND_CLOSE_MIN_PROGRESS
        ):
            self._hand_close_progress_detected = True

        reached = (
            max_position_error <= POSITION_TOLERANCE + COMPARISON_EPSILON
            and max_velocity <= VELOCITY_TOLERANCE + COMPARISON_EPSILON
        )
        self._hand_close_contact_detected = False
        if (
            not reached
            and not self.home_reset
            and self._hand_close_progress_detected
            and max_velocity <= VELOCITY_TOLERANCE + COMPARISON_EPSILON
        ):
            now = time.monotonic()
            if self._hand_close_stall_since is None:
                self._hand_close_stall_since = now
            elif now - self._hand_close_stall_since >= HAND_CLOSE_STALL_SECONDS:
                self._hand_close_contact_detected = True
                reached = True
        elif max_velocity > VELOCITY_TOLERANCE:
            self._hand_close_stall_since = None

        if not reached:
            self._warn_feedback(
                f"Waiting for target: position error {max_position_error:.4f}, "
                f"velocity {max_velocity:.4f}."
            )
        return reached, max_position_error, max_velocity

    def _advance(self) -> None:
        if self.phase == "done":
            return

        now = time.monotonic()
        if self.phase == "await_target":
            reached, position_error, velocity = self._target_reached()
            if not reached:
                return

            step_name = self.steps[self.step_index][0]
            if self._hand_close_contact_detected:
                self.get_logger().info(
                    f"Contact detected during {step_name}; "
                    "continuing with the next measured pose."
                )
            else:
                self.get_logger().info(
                    f"Reached {step_name}: position error {position_error:.4f}, "
                    f"velocity {velocity:.4f}."
                )

            self.phase = "step_pause"
            self.next_action_time = now + POSE_PAUSE_SECONDS
            return

        if now + COMPARISON_EPSILON < self.next_action_time:
            return

        if self.phase == "step_pause":
            self.step_index += 1
            if self.step_index >= len(self.steps):
                self.phase = "done"
                if self.home_reset:
                    self.get_logger().info(
                        "The right hand reached its open home position."
                    )
                else:
                    self.get_logger().info(
                        "The endoscope is held in the final pose. "
                        "No further motion will be sent."
                    )
                return
            self.phase = "send_step"

        if self.phase == "send_step":
            step_name, target = self.steps[self.step_index]
            self.waiting_target = target
            self._hand_close_initial_error = None
            self._hand_close_stall_since = None
            self._hand_close_progress_detected = False
            self._hand_close_contact_detected = False
            if self._latest_positions is not None:
                self._hand_close_initial_error = sum(
                    abs(self._latest_positions[name] - position)
                    for name, position in zip(RIGHT_HAND_JOINTS, target)
                )
            self._publish_target(target)
            self.get_logger().info(f"Moving to {step_name}.")
            self.phase = "await_target"

    def _publish_target(self, target: List[float]) -> None:
        msg = OnlineMotionCommand()
        msg.type = OnlineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = [RIGHT_HAND_GROUP]

        point = TrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * len(target)
        msg.target_point = [point]
        msg.planner = OnlineMotionCommand.PLANNER_RUCKIG
        msg.max_velocity = [ArmConstraintParameters(values=[1.0])]
        msg.max_acceleration = [ArmConstraintParameters(values=[0.8])]
        msg.max_jerk = [ArmConstraintParameters(values=[5.0])]
        msg.is_whole_body = False
        msg.reference_frame = "world_frame"
        self.cmd_id += 1
        msg.command_id = self.cmd_id
        msg.need_interpolation = True
        msg.is_urgent = False
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published command {self.cmd_id} for the right hand on "
            f"{ONLINE_MOTION_TOPIC}; subscribers currently detected="
            f"{self.count_subscribers(ONLINE_MOTION_TOPIC)}."
        )

    def _publish_cartesian_target(self, target: List[float]) -> None:
        msg = OnlineMotionCommand()
        msg.type = OnlineMotionCommand.CARTESIAN_TRAJECTORY
        msg.target_arm_indices = [RIGHT_ARM_GROUP]

        point = TrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * len(target)
        point.accelerations = [0.0] * len(target)
        msg.target_point = [point]
        msg.planner = OnlineMotionCommand.PLANNER_RUCKIG
        msg.max_velocity = [ArmConstraintParameters(values=[0.2])]
        msg.max_acceleration = [ArmConstraintParameters(values=[0.3])]
        msg.max_jerk = [ArmConstraintParameters(values=[5.0])]
        msg.is_whole_body = False
        msg.reference_frame = "world_frame"
        self.cmd_id += 1
        msg.command_id = self.cmd_id
        msg.need_interpolation = True
        msg.is_urgent = False
        msg.pos_tolerance = 0.0
        msg.vel_tolerance = 0.0
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published Cartesian command {self.cmd_id} for the right arm: "
            f"reference_frame={msg.reference_frame}, target={target[:3]} m, "
            f"orientation(wxyz)={target[3:]}, topic={ONLINE_MOTION_TOPIC}, "
            f"subscribers currently detected="
            f"{self.count_subscribers(ONLINE_MOTION_TOPIC)}."
        )


def _confirm_endoscope_ready() -> bool:
    print(
        "\nInstrument choisi : endoscope.\n"
        "Placez l'instrument dans la main droite et degagez la zone autour "
        "de la main."
    )
    try:
        return input("Pret ? [Entree/q] : ").strip().lower() not in ("q", "quitter")
    except EOFError:
        print("\nAucune confirmation recue ; lancement annule.")
        return False


def _confirm_motion(prompt: str) -> bool:
    try:
        response = input(f"{prompt} [o/n] : ").strip().lower()
    except EOFError:
        print("\nAucune confirmation recue ; mouvement annule.")
        return False
    return response in ("o", "oui")


def _normalize_quaternion(quaternion: List[float]) -> List[float]:
    norm = math.hypot(*quaternion)
    if norm <= COMPARISON_EPSILON:
        raise ValueError("Cannot use a zero-length orientation quaternion.")
    return [component / norm for component in quaternion]


def _quaternion_multiply(
    left: List[float],
    right: List[float],
) -> List[float]:
    left_w, left_x, left_y, left_z = left
    right_w, right_x, right_y, right_z = right
    return [
        left_w * right_w - left_x * right_x - left_y * right_y - left_z * right_z,
        left_w * right_x + left_x * right_w + left_y * right_z - left_z * right_y,
        left_w * right_y - left_x * right_z + left_y * right_w + left_z * right_x,
        left_w * right_z + left_x * right_y - left_y * right_x + left_z * right_w,
    ]


def _rotate_vector_by_quaternion(
    quaternion: List[float],
    vector: Tuple[float, float, float],
) -> List[float]:
    rotation = _normalize_quaternion(quaternion)
    conjugate = [rotation[0], -rotation[1], -rotation[2], -rotation[3]]
    rotated = _quaternion_multiply(
        _quaternion_multiply(rotation, [0.0, *vector]),
        conjugate,
    )
    return rotated[1:]


def _hand_to_object_pose(hand_pose: List[float]) -> List[float]:
    object_pose = list(hand_pose)
    offset = _rotate_vector_by_quaternion(
        hand_pose[3:],
        OBJECT_FRAME_OFFSET,
    )
    for axis in range(3):
        object_pose[axis] += offset[axis]
    return object_pose


def _object_to_hand_pose(object_pose: List[float]) -> List[float]:
    hand_pose = list(object_pose)
    offset = _rotate_vector_by_quaternion(
        object_pose[3:],
        OBJECT_FRAME_OFFSET,
    )
    for axis in range(3):
        hand_pose[axis] -= offset[axis]
    return hand_pose


def _rotate_quaternion_in_local_frame(
    quaternion: List[float],
    axis_index: int,
    angle_radians: float,
) -> List[float]:
    half_angle = angle_radians / 2.0
    delta = [math.cos(half_angle), 0.0, 0.0, 0.0]
    delta[axis_index + 1] = math.sin(half_angle)
    return _normalize_quaternion(
        _quaternion_multiply(_normalize_quaternion(quaternion), delta)
    )


def _quaternion_angular_error(
    first: List[float],
    second: List[float],
) -> float:
    first = _normalize_quaternion(first)
    second = _normalize_quaternion(second)
    dot_product = abs(sum(a * b for a, b in zip(first, second)))
    return 2.0 * math.acos(min(1.0, dot_product))


def _wait_for_arm_pose(
    node: SumanEndoscopeHold,
    executor: MultiThreadedExecutor,
) -> Optional[List[float]]:
    print(
        f"[TF] Recherche de {RIGHT_ARM_TF_PARENT}->{RIGHT_ARM_TF_CHILD} sur "
        f"{node.arm_tf_topic} (timeout={TF_WAIT_TIMEOUT_SECONDS:.1f}s, "
        f"publishers actuellement detectes="
        f"{node.count_publishers(node.arm_tf_topic)}).",
        flush=True,
    )
    if node._latest_arm_pose is not None:
        age = time.monotonic() - node._latest_arm_pose_received_at
        print(
            f"[TF] Pose deja en cache; age a la reception={age:.3f}s, "
            f"pose xyz=({node._latest_arm_pose[0]:.4f}, "
            f"{node._latest_arm_pose[1]:.4f}, "
            f"{node._latest_arm_pose[2]:.4f}) m; "
            f"header_stamp={node._latest_arm_tf_stamp}; "
            f"TF correspondantes recues={node._matching_arm_tf_count}.",
            flush=True,
        )
        return list(node._latest_arm_pose)

    started_at = time.monotonic()
    deadline = started_at + TF_WAIT_TIMEOUT_SECONDS
    last_report_at = started_at
    while rclpy.ok() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if node._latest_arm_pose is not None:
            elapsed = time.monotonic() - started_at
            print(
                f"[TF] Pose attendue recue apres {elapsed:.2f}s: "
                f"xyz=({node._latest_arm_pose[0]:.4f}, "
                f"{node._latest_arm_pose[1]:.4f}, "
                f"{node._latest_arm_pose[2]:.4f}) m; "
                f"header_stamp={node._latest_arm_tf_stamp}; "
                f"{node._arm_tf_message_count} message(s), "
                f"{node._arm_tf_transform_count} transform(s), "
                f"{node._matching_arm_tf_count} correspondance(s).",
                flush=True,
            )
            return list(node._latest_arm_pose)

        now = time.monotonic()
        if now - last_report_at >= TF_DIAGNOSTIC_INTERVAL_SECONDS:
            seen = ", ".join(
                f"{parent}->{child}"
                for parent, child in node._seen_arm_tf_frames
            ) or "(aucune)"
            print(
                f"[TF] Toujours en attente ({now - started_at:.1f}s): "
                f"{node._arm_tf_message_count} message(s), "
                f"{node._arm_tf_transform_count} transform(s), "
                f"publishers={node.count_publishers(node.arm_tf_topic)}, "
                f"frames observees={seen}.",
                flush=True,
            )
            last_report_at = now

    seen = ", ".join(
        f"{parent}->{child}" for parent, child in node._seen_arm_tf_frames
    ) or "(aucune)"
    print(
        f"[TF][ECHEC] Pose {RIGHT_ARM_TF_PARENT}->{RIGHT_ARM_TF_CHILD} "
        f"non recue en {TF_WAIT_TIMEOUT_SECONDS:.1f}s. "
        f"publishers={node.count_publishers(node.arm_tf_topic)}, "
        f"messages={node._arm_tf_message_count}, "
        f"transforms={node._arm_tf_transform_count}, "
        f"frames observees={seen}.",
        flush=True,
    )
    return None


def _wait_for_cartesian_target(
    node: SumanEndoscopeHold,
    executor: MultiThreadedExecutor,
    target: List[float],
) -> bool:
    started_at = time.monotonic()
    deadline = started_at + CARTESIAN_TIMEOUT_SECONDS
    settled_since: Optional[float] = None
    last_pose_received_at = node._latest_arm_pose_received_at
    last_report_at = started_at
    received_new_tf = False
    last_position_error: Optional[float] = None
    last_orientation_error: Optional[float] = None
    print(
        f"[TF] Suivi de la cible xyz=({target[0]:.4f}, {target[1]:.4f}, "
        f"{target[2]:.4f}) m; orientation(wxyz)="
        f"({target[3]:.4f}, {target[4]:.4f}, {target[5]:.4f}, "
        f"{target[6]:.4f}); tolerances="
        f"{CARTESIAN_POSITION_TOLERANCE:.3f}m/"
        f"{math.degrees(CARTESIAN_ORIENTATION_TOLERANCE_RADIANS):.1f}deg, "
        f"stabilisation={CARTESIAN_SETTLE_SECONDS:.1f}s, "
        f"timeout={CARTESIAN_TIMEOUT_SECONDS:.1f}s. "
        "Attente d'au moins une nouvelle TF apres l'envoi.",
        flush=True,
    )

    while rclpy.ok() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        pose = node._latest_arm_pose
        pose_received_at = node._latest_arm_pose_received_at
        now = time.monotonic()
        if pose is not None and pose_received_at > last_pose_received_at:
            received_new_tf = True
        else:
            pose = None

        if pose is None:
            if now - last_report_at >= TF_DIAGNOSTIC_INTERVAL_SECONDS:
                latest_pose = node._latest_arm_pose
                pose_description = (
                    "aucune pose"
                    if latest_pose is None
                    else (
                        f"derniere pose=({latest_pose[0]:.4f}, "
                        f"{latest_pose[1]:.4f}, {latest_pose[2]:.4f}) m, "
                        f"age={now - node._latest_arm_pose_received_at:.2f}s, "
                        f"header_stamp={node._latest_arm_tf_stamp}"
                    )
                )
                print(
                    f"[TF] Suivi en cours depuis {now - started_at:.1f}s; "
                    f"aucune nouvelle TF depuis l'envoi; "
                    f"{pose_description}; "
                    f"TF correspondantes total={node._matching_arm_tf_count}.",
                    flush=True,
                )
                last_report_at = now
            continue

        last_pose_received_at = pose_received_at

        position_error = max(
            abs(pose[index] - target[index]) for index in range(3)
        )
        orientation_error = _quaternion_angular_error(pose[3:], target[3:])
        last_position_error = position_error
        last_orientation_error = orientation_error
        if (
            position_error <= CARTESIAN_POSITION_TOLERANCE
            and orientation_error <= CARTESIAN_ORIENTATION_TOLERANCE_RADIANS
        ):
            if settled_since is None:
                settled_since = now
            elif now - settled_since >= CARTESIAN_SETTLE_SECONDS:
                print(
                    f"[TF] Cible confirmee apres {now - started_at:.2f}s: "
                    f"erreur position={position_error:.4f}m, "
                    f"erreur orientation={math.degrees(orientation_error):.2f}deg, "
                    f"pose=({pose[0]:.4f}, {pose[1]:.4f}, "
                    f"{pose[2]:.4f}) m.",
                    flush=True,
                )
                return True
        else:
            settled_since = None

        if now - last_report_at >= TF_DIAGNOSTIC_INTERVAL_SECONDS:
            print(
                f"[TF] Suivi depuis {now - started_at:.1f}s: "
                f"pose=({pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}) m, "
                f"erreur position={position_error:.4f}m, "
                f"erreur orientation={math.degrees(orientation_error):.2f}deg, "
                f"nouvelle TF recue={'oui' if received_new_tf else 'non'}, "
                f"header_stamp={node._latest_arm_tf_stamp}, "
                f"TF correspondantes total={node._matching_arm_tf_count}.",
                flush=True,
            )
            last_report_at = now

    latest_pose = node._latest_arm_pose
    latest_description = (
        "aucune pose disponible"
        if latest_pose is None
        else (
            f"derniere pose=({latest_pose[0]:.4f}, "
            f"{latest_pose[1]:.4f}, {latest_pose[2]:.4f}) m, "
            f"age={time.monotonic() - node._latest_arm_pose_received_at:.2f}s, "
            f"header_stamp={node._latest_arm_tf_stamp}"
        )
    )
    print(
        f"[TF][ECHEC] Cible non confirmee en "
        f"{CARTESIAN_TIMEOUT_SECONDS:.1f}s; nouvelle TF recue="
        f"{'oui' if received_new_tf else 'non'}, "
        f"derniere erreur position="
        f"{'inconnue' if last_position_error is None else f'{last_position_error:.4f}m'}, "
        f"derniere erreur orientation="
        f"{'inconnue' if last_orientation_error is None else f'{math.degrees(last_orientation_error):.2f}deg'}, "
        f"{latest_description}, "
        f"TF correspondantes total={node._matching_arm_tf_count}.",
        flush=True,
    )
    return False


def _run_motion_menu(
    node: SumanEndoscopeHold,
    executor: MultiThreadedExecutor,
) -> bool:
    print(
        "\nMouvements de l'endoscope : translations de +10 cm ou rotations de "
        f"+/-{CARTESIAN_ROTATION_DEGREES:g} degres, puis retour a la pose de depart.\n"
        "Axes exprimes dans le repere objet, decale de +20 cm sur Z local "
        "par rapport a la main."
    )
    while rclpy.ok():
        try:
            choice = (
                input(
                    "Choisir [x/y/z] pour translater, [rx+/rx-/ry+/ry-/rz+/rz-] "
                    "pour tourner, ou q pour terminer : "
                )
                .strip()
                .lower()
            )
        except EOFError:
            print("\nAucune saisie disponible ; fin du menu de mouvement.")
            return True

        if choice in ("q", "quitter"):
            return True
        is_rotation = (
            len(choice) == 3
            and choice[0] == "r"
            and choice[1] in ROTATION_AXES
            and choice[2] in ("+", "-")
        )
        if choice not in TRANSLATION_AXES and not is_rotation:
            print(
                "Choix invalide ; saisir x, y, z, rx+, rx-, ry+, ry-, rz+, "
                "rz- ou q."
            )
            continue

        if is_rotation:
            axis = choice[1]
            direction = 1.0 if choice[2] == "+" else -1.0
            rotation_degrees = direction * CARTESIAN_ROTATION_DEGREES
            operation_label = (
                f"rotation de {rotation_degrees:+g} degres autour de {axis.upper()}"
            )
        else:
            axis = choice
            operation_label = f"deplacement de +10 cm sur {axis.upper()}"

        confirmation = (
            f"Confirmer la {operation_label}"
            if is_rotation
            else f"Confirmer le {operation_label}"
        )
        if not _confirm_motion(confirmation):
            print("Mouvement annule.")
            continue

        start = _wait_for_arm_pose(node, executor)
        if start is None:
            if node.count_publishers(node.arm_tf_topic) == 0:
                print(
                    f"Aucun publisher sur {node.arm_tf_topic} ; "
                    "aucun mouvement envoye."
                )
            elif node._seen_arm_tf_frames:
                frames = ", ".join(
                    f"{parent}->{child}"
                    for parent, child in node._seen_arm_tf_frames
                )
                print(
                    "Des TF sont recus, mais la paire attendue "
                    f"{RIGHT_ARM_TF_PARENT}->{RIGHT_ARM_TF_CHILD} est absente. "
                    f"Paires observees : {frames}. Aucun mouvement envoye."
                )
            else:
                print(
                    f"Aucune pose TF recue sur {node.arm_tf_topic} ; "
                    "aucun mouvement envoye."
                )
            return False

        object_start = _hand_to_object_pose(start)
        object_target = list(object_start)
        if is_rotation:
            object_target[3:] = _rotate_quaternion_in_local_frame(
                object_start[3:],
                ROTATION_AXES[axis],
                math.radians(rotation_degrees),
            )
        else:
            local_translation = [0.0, 0.0, 0.0]
            local_translation[TRANSLATION_AXES[axis]] = (
                CARTESIAN_TRANSLATION_METERS
            )
            parent_translation = _rotate_vector_by_quaternion(
                object_start[3:],
                tuple(local_translation),
            )
            for index in range(3):
                object_target[index] += parent_translation[index]
        target = _object_to_hand_pose(object_target)
        print(
            f"Pose objet initiale xyz=({object_start[0]:.4f}, "
            f"{object_start[1]:.4f}, {object_start[2]:.4f}) m.\n"
            f"Mouvement : {operation_label}; pose objet cible xyz="
            f"({object_target[0]:.3f}, {object_target[1]:.3f}, "
            f"{object_target[2]:.3f}) m, "
            f"quaternion(wxyz)=({object_target[3]:.4f}, "
            f"{object_target[4]:.4f}, {object_target[5]:.4f}, "
            f"{object_target[6]:.4f})."
        )
        node._publish_cartesian_target(target)
        if not _wait_for_cartesian_target(node, executor, target):
            print(
                "La cible n'a pas ete confirmee par le retour TF dans le delai "
                "imparti ; aucun retour automatique n'est envoye."
            )
            return False

        if not _confirm_motion(
            f"Confirmer le retour a la pose de depart apres {operation_label}"
        ):
            print(
                "Retour non valide ; la position actuelle est conservee. "
                "Fin du menu de mouvement."
            )
            return True

        node._publish_cartesian_target(start)
        if not _wait_for_cartesian_target(node, executor, start):
            print(
                "Le retour n'a pas ete confirme par le retour TF dans le delai "
                "imparti ; arret du menu de mouvement."
            )
            return False
        print(f"Retour confirme apres {operation_label}.")

    return True


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Saisir l'endoscope et publier son repere outil."
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
    parser.add_argument(
        "--arm-tf-topic",
        default="",
        help="Arm TF feedback topic; overrides the topic derived from --robot-id.",
    )
    parser.add_argument(
        "--main-frame",
        default="main",
        help="Parent TF frame for the endoscope frame (default: main).",
    )
    parser.add_argument(
        "--tool-frame",
        default="outil_endoscope",
        help="Child TF frame for the endoscope (default: outil_endoscope).",
    )
    parsed_args, _ = parser.parse_known_args(args=args)
    if not parsed_args.main_frame.strip() or not parsed_args.tool_frame.strip():
        parser.error("--main-frame et --tool-frame ne peuvent pas etre vides.")
    if parsed_args.main_frame == parsed_args.tool_frame:
        parser.error("--main-frame et --tool-frame doivent etre differents.")

    if not _confirm_endoscope_ready():
        print("Lancement annule.")
        return

    print("Appuyez sur Ctrl+C pour arreter.")
    joint_state_topic = (
        parsed_args.joint_state_topic.strip()
        or f"/{parsed_args.robot_id}/mc/pr10_joint_states_topic"
    )
    arm_tf_topic = (
        parsed_args.arm_tf_topic.strip()
        or f"/{parsed_args.robot_id}/mc/pr10_arm_tf_topic"
    )
    if not joint_state_topic.startswith("/"):
        joint_state_topic = "/" + joint_state_topic
    if not arm_tf_topic.startswith("/"):
        arm_tf_topic = "/" + arm_tf_topic

    rclpy.init(args=args)
    node = SumanEndoscopeHold(
        joint_state_topic,
        arm_tf_topic,
        parsed_args.main_frame,
        parsed_args.tool_frame,
    )
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if node.phase != "done":
                continue

            if node.home_reset:
                if not _confirm_endoscope_ready():
                    print("Arret du test.")
                    break
                node.start_endoscope()
                continue

            if not _run_motion_menu(node, executor):
                print("Arret du test apres un probleme de mouvement cartesien.")
                break

            try:
                response = input(
                    "\nSequence terminee. Reprenez l'outil, puis appuyez sur "
                    "Entree pour ouvrir la main et revenir a la position "
                    "initiale [Entree/q] : "
                ).strip().lower()
            except EOFError:
                print("\nAucune saisie disponible ; arret du test.")
                break

            if response in ("q", "quitter"):
                break
            node.start_home_reset()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
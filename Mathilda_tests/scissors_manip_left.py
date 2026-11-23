#!/usr/bin/env python3
"""Saisie interactive de ciseaux et mouvements du bras gauche --> A TESTER """

import argparse
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from px_mc_msgs.msg import ArmConstraintParameters, OnlineMotionCommand, TrajectoryPoint
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


ONLINE_MOTION_TOPIC = "/external/online_motion_command"
LEFT_ARM_INDEX = 0
LEFT_HAND_INDEX = 4
LEFT_ARM_JOINTS = [f"left_arm_joint_{index}" for index in range(7)]
LEFT_HAND_JOINTS = [
    f"left_{finger}_joint_{index}"
    for finger in ("index", "middle", "ring", "pinky", "thumb")
    for index in range(3)
]

# Pose home gauche relevée dans les exemples du robot.
HOME_LEFT_ARM = [0.0, 0.4, 0.0, -1.57, 0.0, 0.0, 0.0]
HAND_OPEN = [0.0] * len(LEFT_HAND_JOINTS)

# Poses de saisie reprises de suman_hold_laparo_instruments.py.
SCISSORS_GRASP: List[Tuple[str, List[float]]] = [
    (
        "enfiler les ciseaux entre le pouce et l'index",
        [
            0.003, 0.0, 0.0,
            1.221, 0.017, 0.007,
            0.016, 0.003, 0.001,
            0.0, 0.0, 0.0,
            1.623, 0.757, 0.0,
        ],
    ),
    (
        "resserrer les doigts sur les ciseaux",
        [
            0.003, 0.0, 0.0,
            1.193, 0.201, 0.113,
            0.016, 0.003, 0.001,
            0.0, 0.0, 0.0,
            1.623, 0.978, 0.0,
        ],
    ),
    (
        "mettre l'index en appui",
        [
            1.037, 0.247, 0.146,
            1.265, 0.103, 0.057,
            0.016, 0.003, 0.001,
            0.0, 0.0, 0.0,
            1.623, 0.924, 0.0,
        ],
    ),
]

BASE_FRAME = "waist_link_0"
ARM_COMMAND_FRAME = "waist_link_4"
ARM_FRAME = "left_arm_link_6"
PALM_FRAME = "left_palm_link"
WAIST_JOINTS = [f"waist_joint_{index}" for index in range(1, 5)]
LEFT_ARM_LINK_JOINTS = [
    f"left_arm_joint_{index}" for index in range(7)
]
LEFT_ARM_FRAME_JOINTS = LEFT_ARM_LINK_JOINTS + ["left_palm_joint"]
ARM_CHAIN_FRAMES = [f"left_arm_link_{index}" for index in range(7)] + [
    PALM_FRAME
]
TOOL_FRAME = "tool_tip"
# Offset miroir de la configuration du bras droit; a valider sur le robot.
TOOL_OFFSET_IN_PALM = (0.06, 0.30, -0.35)
TOOL_ROTATION_X_DEGREES = -45.0
TF_REPUBLISH_PERIOD = 0.1
TRANSLATION_METERS = 0.10
ROTATION_DEGREES = 10.0
JOINT_TOLERANCE = 0.10
JOINT_VELOCITY_TOLERANCE = 0.01
GRASP_CONTACT_MIN_PROGRESS = 0.05
GRASP_CONTACT_STALL_SECONDS = 1.5
CARTESIAN_TOLERANCE = 0.01
CARTESIAN_ANGULAR_TOLERANCE = math.radians(2.0)
FEEDBACK_TIMEOUT = 30.0
MOTION_TIMEOUT = 60.0
TRANSLATION_AXES: Dict[str, Tuple[float, float, float]] = {
    "x": (1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
}


@dataclass(frozen=True)
class JointTransform:
    parent: str
    child: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    joint_type: str


def _parse_vector(
    value: Optional[str],
    default: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    if value is None:
        return default
    components = [float(component) for component in value.split()]
    if len(components) != 3:
        raise ValueError(f"Un vecteur URDF doit contenir 3 valeurs : {value!r}.")
    return components[0], components[1], components[2]


def _load_urdf_joints(urdf_path: Path) -> Dict[str, JointTransform]:
    root = ET.parse(str(urdf_path)).getroot()
    required_names = WAIST_JOINTS + LEFT_ARM_FRAME_JOINTS
    joints: Dict[str, JointTransform] = {}
    for joint_name in required_names:
        element = root.find(f"./joint[@name='{joint_name}']")
        if element is None:
            raise ValueError(f"Joint {joint_name!r} absent du fichier {urdf_path}.")
        parent = element.find("parent")
        child = element.find("child")
        if parent is None or child is None:
            raise ValueError(f"Joint {joint_name!r} sans parent ou enfant dans l'URDF.")
        origin = element.find("origin")
        axis = element.find("axis")
        joints[joint_name] = JointTransform(
            parent=parent.attrib["link"],
            child=child.attrib["link"],
            xyz=_parse_vector(
                origin.attrib.get("xyz") if origin is not None else None,
                (0.0, 0.0, 0.0),
            ),
            rpy=_parse_vector(
                origin.attrib.get("rpy") if origin is not None else None,
                (0.0, 0.0, 0.0),
            ),
            axis=_parse_vector(
                axis.attrib.get("xyz") if axis is not None else None,
                (1.0, 0.0, 0.0),
            ),
            joint_type=element.attrib["type"],
        )

    expected_chain = [
        (WAIST_JOINTS[0], BASE_FRAME, "waist_link_1"),
        ("waist_joint_2", "waist_link_1", "waist_link_2"),
        ("waist_joint_3", "waist_link_2", "waist_link_3"),
        (WAIST_JOINTS[3], "waist_link_3", ARM_COMMAND_FRAME),
        (LEFT_ARM_LINK_JOINTS[0], ARM_COMMAND_FRAME, ARM_CHAIN_FRAMES[0]),
    ]
    expected_chain.extend(
        (
            LEFT_ARM_LINK_JOINTS[index],
            ARM_CHAIN_FRAMES[index - 1],
            ARM_CHAIN_FRAMES[index],
        )
        for index in range(1, len(LEFT_ARM_LINK_JOINTS))
    )
    expected_chain.append(
        (LEFT_ARM_FRAME_JOINTS[-1], ARM_FRAME, PALM_FRAME)
    )
    for joint_name, expected_parent, expected_child in expected_chain:
        joint = joints[joint_name]
        if (joint.parent, joint.child) != (expected_parent, expected_child):
            raise ValueError(
                f"Chaîne URDF inattendue pour {joint_name!r} : "
                f"{joint.parent} -> {joint.child}."
            )
    return joints


def _rpy_to_quaternion(rpy: Sequence[float]) -> List[float]:
    roll, pitch, yaw = rpy
    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    half_yaw = yaw / 2.0
    return [
        math.cos(half_roll) * math.cos(half_pitch) * math.cos(half_yaw)
        + math.sin(half_roll) * math.sin(half_pitch) * math.sin(half_yaw),
        math.sin(half_roll) * math.cos(half_pitch) * math.cos(half_yaw)
        - math.cos(half_roll) * math.sin(half_pitch) * math.sin(half_yaw),
        math.cos(half_roll) * math.sin(half_pitch) * math.cos(half_yaw)
        + math.sin(half_roll) * math.cos(half_pitch) * math.sin(half_yaw),
        math.cos(half_roll) * math.cos(half_pitch) * math.sin(half_yaw)
        - math.sin(half_roll) * math.sin(half_pitch) * math.cos(half_yaw),
    ]


def _compose_poses(first: Sequence[float], second: Sequence[float]) -> List[float]:
    rotated_translation = _rotate_vector_by_quaternion(second[:3], first[3:7])
    return [
        first[index] + rotated_translation[index] for index in range(3)
    ] + _multiply_quaternions(first[3:7], second[3:7])


def _joint_pose(
    joint: JointTransform,
    joint_positions: Dict[str, float],
    joint_name: str,
) -> List[float]:
    orientation = _rpy_to_quaternion(joint.rpy)
    if joint.joint_type in ("revolute", "continuous"):
        angle = joint_positions[joint_name] / 2.0
        axis_norm = math.sqrt(sum(component * component for component in joint.axis))
        if axis_norm == 0.0:
            raise ValueError(f"L'axe du joint {joint_name!r} est nul dans l'URDF.")
        joint_rotation = [
            math.cos(angle),
            *(component / axis_norm * math.sin(angle) for component in joint.axis),
        ]
        orientation = _multiply_quaternions(orientation, joint_rotation)
    elif joint.joint_type != "fixed":
        raise ValueError(
            f"Type de joint {joint.joint_type!r} non gere pour {joint_name!r}."
        )
    return list(joint.xyz) + orientation


def _multiply_quaternions(
    first: Sequence[float],
    second: Sequence[float],
) -> List[float]:
    """Multiply quaternions in [w, x, y, z] order."""
    first_w, first_x, first_y, first_z = first
    second_w, second_x, second_y, second_z = second
    return [
        first_w * second_w
        - first_x * second_x
        - first_y * second_y
        - first_z * second_z,
        first_w * second_x
        + first_x * second_w
        + first_y * second_z
        - first_z * second_y,
        first_w * second_y
        - first_x * second_z
        + first_y * second_w
        + first_z * second_x,
        first_w * second_z
        + first_x * second_y
        - first_y * second_x
        + first_z * second_w,
    ]


def _conjugate_quaternion(quaternion: Sequence[float]) -> List[float]:
    return [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]


def _quaternion_angular_error(
    first: Sequence[float],
    second: Sequence[float],
) -> float:
    """Return the shortest angle between two [w, x, y, z] quaternions."""
    first_norm = math.sqrt(sum(component * component for component in first))
    second_norm = math.sqrt(sum(component * component for component in second))
    if first_norm == 0.0 or second_norm == 0.0:
        raise ValueError("Un quaternion nul ne definit pas une orientation.")
    dot = abs(
        sum(
            first_component * second_component
            for first_component, second_component in zip(first, second)
        )
        / (first_norm * second_norm)
    )
    return 2.0 * math.acos(min(1.0, dot))


def _rotate_vector_by_quaternion(
    vector: Sequence[float],
    quaternion: Sequence[float],
) -> List[float]:
    """Convert a vector from tool-local coordinates to the command frame."""
    q_w, q_x, q_y, q_z = quaternion
    vector_x, vector_y, vector_z = vector

    cross_x = q_y * vector_z - q_z * vector_y
    cross_y = q_z * vector_x - q_x * vector_z
    cross_z = q_x * vector_y - q_y * vector_x
    cross_x *= 2.0
    cross_y *= 2.0
    cross_z *= 2.0

    return [
        vector_x + q_w * cross_x + q_y * cross_z - q_z * cross_y,
        vector_y + q_w * cross_y + q_z * cross_x - q_x * cross_z,
        vector_z + q_w * cross_z + q_x * cross_y - q_y * cross_x,
    ]


def _pose_rotated_around_tool_tip(
    pose: Sequence[float],
    target_orientation: Sequence[float],
    tool_pose_in_arm: Sequence[float],
) -> List[float]:
    """Build an arm pose that rotates around the fixed tool-tip position."""
    current_tip_offset = _rotate_vector_by_quaternion(
        tool_pose_in_arm[:3],
        pose[3:7],
    )
    tool_tip_position = [
        pose[index] + current_tip_offset[index] for index in range(3)
    ]
    target_tip_offset = _rotate_vector_by_quaternion(
        tool_pose_in_arm[:3],
        target_orientation,
    )
    return [
        tool_tip_position[index] - target_tip_offset[index]
        for index in range(3)
    ] + list(target_orientation)


class LeftScissorsManipulation(Node):
    """Envoie les commandes de saisie des ciseaux et publie leurs repères."""

    def __init__(
        self,
        joint_topic: str,
        tf_topic: str,
        urdf_path: Path,
    ) -> None:
        super().__init__("scissors_manipulation_left")
        self._motion_publisher = self.create_publisher(
            OnlineMotionCommand, ONLINE_MOTION_TOPIC, 10
        )
        self._joint_positions: Dict[str, float] = {}
        self._joint_velocities: Dict[str, float] = {}
        self._arm_pose: Optional[List[float]] = None
        self._arm_pose_received_at = 0.0
        self._urdf_joints = _load_urdf_joints(urdf_path)
        palm_joint_name = LEFT_ARM_FRAME_JOINTS[-1]
        palm_pose = _joint_pose(
            self._urdf_joints[palm_joint_name],
            {},
            palm_joint_name,
        )
        half_tool_rotation = math.radians(TOOL_ROTATION_X_DEGREES) / 2.0
        tool_pose_in_palm = list(TOOL_OFFSET_IN_PALM) + [
            math.cos(half_tool_rotation),
            math.sin(half_tool_rotation),
            0.0,
            0.0,
        ]
        self._tool_pose_in_arm = _compose_poses(
            palm_pose,
            tool_pose_in_palm,
        )
        self._joint_state_warning_logged = False
        self._command_id = 0

        self.create_subscription(JointState, joint_topic, self._joint_state_cb, 10)
        self.create_subscription(
            TFMessage,
            tf_topic,
            self._tf_cb,
            rclpy.qos.qos_profile_sensor_data,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_timer(TF_REPUBLISH_PERIOD, self._publish_arm_chain)
        self._publish_tool_transform()
        self.get_logger().info(
            f"TF cinematiques calculees depuis {urdf_path} et les JointState : "
            f"{BASE_FRAME} -> {ARM_COMMAND_FRAME} -> {ARM_CHAIN_FRAMES[0]} "
            f"... -> {PALM_FRAME}."
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        self._joint_positions = dict(zip(msg.name, msg.position))
        if len(msg.name) == len(msg.velocity):
            self._joint_velocities = dict(zip(msg.name, msg.velocity))
        else:
            self._joint_velocities = {}

    def _tf_cb(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent != ARM_COMMAND_FRAME or child != ARM_FRAME:
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self._arm_pose = [
                float(translation.x),
                float(translation.y),
                float(translation.z),
                float(rotation.w),
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
            ]
            self._arm_pose_received_at = time.monotonic()

    def _publish_arm_chain(self) -> None:
        required_joint_names = WAIST_JOINTS + LEFT_ARM_FRAME_JOINTS[:-1]
        missing_joints = [
            name for name in required_joint_names if name not in self._joint_positions
        ]
        if missing_joints:
            if not self._joint_state_warning_logged:
                self.get_logger().warning(
                    "Publication TF en attente des joints : "
                    + ", ".join(missing_joints)
                )
                self._joint_state_warning_logged = True
            return

        self._joint_state_warning_logged = False
        now = self.get_clock().now().to_msg()
        transforms: List[TransformStamped] = []

        waist_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        for joint_name in WAIST_JOINTS:
            waist_pose = _compose_poses(
                waist_pose,
                _joint_pose(
                    self._urdf_joints[joint_name],
                    self._joint_positions,
                    joint_name,
                ),
            )
        transforms.append(
            self._transform_from_pose(
                BASE_FRAME,
                ARM_COMMAND_FRAME,
                waist_pose,
                now,
            )
        )

        for joint_name in LEFT_ARM_LINK_JOINTS:
            joint = self._urdf_joints[joint_name]
            transforms.append(
                self._transform_from_pose(
                    joint.parent,
                    joint.child,
                    _joint_pose(joint, self._joint_positions, joint_name),
                    now,
                )
            )

        palm_joint = self._urdf_joints[LEFT_ARM_FRAME_JOINTS[-1]]
        transforms.append(
            self._transform_from_pose(
                palm_joint.parent,
                palm_joint.child,
                _joint_pose(
                    palm_joint,
                    self._joint_positions,
                    LEFT_ARM_FRAME_JOINTS[-1],
                ),
                now,
            )
        )
        for transform in transforms:
            self._tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _transform_from_pose(
        parent: str,
        child: str,
        pose: Sequence[float],
        stamp,
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = pose[:3]
        (
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
        ) = pose[3:7]
        return transform

    def _publish_tool_transform(self) -> None:
        tool_tf = TransformStamped()
        tool_tf.header.stamp = self.get_clock().now().to_msg()
        tool_tf.header.frame_id = PALM_FRAME
        tool_tf.child_frame_id = TOOL_FRAME
        (
            tool_tf.transform.translation.x,
            tool_tf.transform.translation.y,
            tool_tf.transform.translation.z,
        ) = TOOL_OFFSET_IN_PALM
        half_tool_rotation = math.radians(TOOL_ROTATION_X_DEGREES) / 2.0
        tool_tf.transform.rotation.w = math.cos(half_tool_rotation)
        tool_tf.transform.rotation.x = math.sin(half_tool_rotation)
        self._static_tf_broadcaster.sendTransform(tool_tf)
        self.get_logger().info(
            f"TF statique publiee : {PALM_FRAME} -> {TOOL_FRAME}, "
            f"translation locale x={TOOL_OFFSET_IN_PALM[0]:.2f} m, "
            f"y={TOOL_OFFSET_IN_PALM[1]:.2f} m, "
            f"z={TOOL_OFFSET_IN_PALM[2]:.2f} m, "
            f"rotation autour de x={TOOL_ROTATION_X_DEGREES:.0f} deg."
        )

    def publish_joint_target(self, arm_index: int, target: Sequence[float]) -> None:
        if self.count_subscribers(ONLINE_MOTION_TOPIC) == 0:
            raise RuntimeError(
                f"Aucun abonne au topic de commande {ONLINE_MOTION_TOPIC}."
            )

        command = OnlineMotionCommand()
        command.type = OnlineMotionCommand.JOINT_TRAJECTORY
        command.target_arm_indices = [arm_index]
        command.target_point = [self._joint_point(target)]
        command.planner = OnlineMotionCommand.PLANNER_RUCKIG
        command.max_velocity = [ArmConstraintParameters(values=[0.4])]
        command.max_acceleration = [ArmConstraintParameters(values=[0.5])]
        command.max_jerk = [ArmConstraintParameters(values=[5.0])]
        command.is_whole_body = False
        command.reference_frame = "world_frame"
        self._publish(command)

    def publish_cartesian_target(self, target: Sequence[float]) -> None:
        if self.count_subscribers(ONLINE_MOTION_TOPIC) == 0:
            raise RuntimeError(
                f"Aucun abonne au topic de commande {ONLINE_MOTION_TOPIC}."
            )

        command = OnlineMotionCommand()
        command.type = OnlineMotionCommand.CARTESIAN_TRAJECTORY
        command.target_arm_indices = [LEFT_ARM_INDEX]
        command.target_point = [self._cartesian_point(target)]
        command.planner = OnlineMotionCommand.PLANNER_RUCKIG
        command.max_velocity = [ArmConstraintParameters(values=[0.2])]
        command.max_acceleration = [ArmConstraintParameters(values=[0.3])]
        command.max_jerk = [ArmConstraintParameters(values=[5.0])]
        command.is_whole_body = False
        command.reference_frame = ARM_COMMAND_FRAME
        command.pos_tolerance = CARTESIAN_TOLERANCE
        command.vel_tolerance = 0.01
        self._publish(command)

    def _publish(self, command: OnlineMotionCommand) -> None:
        self._command_id += 1
        command.command_id = self._command_id
        command.need_interpolation = True
        command.is_urgent = False
        command.stamp = self.get_clock().now().to_msg()
        self._motion_publisher.publish(command)
        self.get_logger().info(f"Commande de mouvement #{self._command_id} publiee.")

    @staticmethod
    def _joint_point(positions: Sequence[float]) -> TrajectoryPoint:
        point = TrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * len(positions)
        return point

    @staticmethod
    def _cartesian_point(pose: Sequence[float]) -> TrajectoryPoint:
        point = TrajectoryPoint()
        point.positions = list(pose)
        point.velocities = [0.0] * 7
        point.accelerations = [0.0] * 7
        return point


def _confirm(action: str) -> bool:
    """Demande une validation explicite avant tout mouvement."""
    try:
        answer = input(f"\n{action} ? [o/n] : ").strip().lower()
    except EOFError:
        print("Aucune validation recue ; mouvement annule.")
        return False
    return answer in ("o", "oui")


def _spin_until(
    predicate: Callable[[], bool],
    timeout: float,
) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.1)
    return False


def _move_joints(
    node: LeftScissorsManipulation,
    description: str,
    arm_index: int,
    joints: Sequence[str],
    target: Sequence[float],
) -> bool:
    if not _confirm(description):
        return False

    node.publish_joint_target(arm_index, target)

    def target_reached() -> bool:
        return all(
            name in node._joint_positions
            and abs(node._joint_positions[name] - value) <= JOINT_TOLERANCE
            for name, value in zip(joints, target)
        )

    if not _spin_until(target_reached, FEEDBACK_TIMEOUT):
        raise TimeoutError(
            f"Retour articulaire non conforme apres {FEEDBACK_TIMEOUT:.0f} s "
            f"pour l'action : {description}."
        )
    print(f"Position confirmee : {description}.")
    return True


def _move_scissors_grasp_step(
    node: LeftScissorsManipulation,
    description: str,
    target: Sequence[float],
) -> bool:
    if not _confirm(description):
        return False

    initial_error: Optional[float] = None
    if all(name in node._joint_positions for name in LEFT_HAND_JOINTS):
        initial_error = sum(
            abs(node._joint_positions[name] - value)
            for name, value in zip(LEFT_HAND_JOINTS, target)
        )
    node.publish_joint_target(LEFT_HAND_INDEX, target)
    progress_detected = False
    stall_since: Optional[float] = None
    deadline = time.monotonic() + FEEDBACK_TIMEOUT

    while rclpy.ok() and time.monotonic() < deadline:
        if all(name in node._joint_positions for name in LEFT_HAND_JOINTS):
            errors = [
                abs(node._joint_positions[name] - value)
                for name, value in zip(LEFT_HAND_JOINTS, target)
            ]
            total_error = sum(errors)
            if max(errors) <= JOINT_TOLERANCE:
                print(f"Position confirmee : {description}.")
                return True

            if initial_error is None:
                initial_error = total_error
            elif initial_error - total_error >= GRASP_CONTACT_MIN_PROGRESS:
                progress_detected = True

            if all(name in node._joint_velocities for name in LEFT_HAND_JOINTS):
                max_velocity = max(
                    abs(node._joint_velocities[name]) for name in LEFT_HAND_JOINTS
                )
                if (
                    progress_detected
                    and max_velocity <= JOINT_VELOCITY_TOLERANCE
                ):
                    if stall_since is None:
                        stall_since = time.monotonic()
                    elif time.monotonic() - stall_since >= GRASP_CONTACT_STALL_SECONDS:
                        print(f"Contact confirme : {description}.")
                        return True
                elif max_velocity > JOINT_VELOCITY_TOLERANCE:
                    stall_since = None

        time.sleep(0.1)

    raise TimeoutError(
        f"La pose de saisie n'a pas ete atteinte et aucun contact stable "
        f"n'a ete detecte en {FEEDBACK_TIMEOUT:.0f} s : {description}."
    )


def _wait_for_arm_pose(
    node: LeftScissorsManipulation,
) -> Optional[List[float]]:
    if node._arm_pose is None and not _spin_until(
        lambda: node._arm_pose is not None, FEEDBACK_TIMEOUT
    ):
        return None
    return list(node._arm_pose) if node._arm_pose is not None else None


def _move_cartesian(
    node: LeftScissorsManipulation,
    pose: Sequence[float],
    description: str,
    check_orientation: bool = False,
) -> bool:
    if not _confirm(f"Confirmer {description}"):
        return False

    start_received_at = node._arm_pose_received_at
    node.publish_cartesian_target(pose)
    deadline = time.monotonic() + MOTION_TIMEOUT
    while rclpy.ok() and time.monotonic() < deadline:
        current = node._arm_pose
        if (
            current is None
            or node._arm_pose_received_at <= start_received_at
            or node._arm_pose_received_at < time.monotonic() - 1.0
        ):
            time.sleep(0.1)
            continue
        error = max(abs(current[index] - pose[index]) for index in range(3))
        orientation_reached = (
            not check_orientation
            or _quaternion_angular_error(current[3:7], pose[3:7])
            <= CARTESIAN_ANGULAR_TOLERANCE
        )
        if error <= CARTESIAN_TOLERANCE and orientation_reached:
            print("Mouvement cartesien confirme par la TF du bras.")
            return True
        time.sleep(0.1)
    raise TimeoutError(
        "La nouvelle pose du bras n'a pas confirme la cible cartesienne "
        f"en {MOTION_TIMEOUT:.0f} s. Aucun retour automatique n'est envoye."
    )


def _translation_menu(
    node: LeftScissorsManipulation,
) -> None:
    while rclpy.ok():
        print(
            "\nActions possibles :\n"
            f"  x+ / x-   : translation +/-10 cm selon l'axe x de {TOOL_FRAME}\n"
            f"  y+ / y-   : translation +/-10 cm selon l'axe y de {TOOL_FRAME}\n"
            f"  z+ / z-   : translation +/-10 cm selon l'axe z de {TOOL_FRAME}\n"
            f"  rx+ / rx- : rotation +/-10 degres autour de x ({TOOL_FRAME})\n"
            f"  ry+ / ry- : rotation +/-10 degres autour de y ({TOOL_FRAME})\n"
            f"  rz+ / rz- : rotation +/-10 degres autour de z ({TOOL_FRAME})\n"
            "  h         : revenir au home du bras en gardant les ciseaux\n"
            "  r         : revenir au home puis relacher les ciseaux\n"
            "  q         : quitter sans autre mouvement"
        )
        try:
            choice = input("Votre choix : ").strip().lower()
        except EOFError:
            print("Fin de saisie ; aucun mouvement supplementaire.")
            return

        if choice in ("q", "quitter"):
            return
        if choice in ("h", "home"):
            if not _move_joints(
                node,
                "Revenir au home du bras gauche",
                LEFT_ARM_INDEX,
                LEFT_ARM_JOINTS,
                HOME_LEFT_ARM,
            ):
                continue
            continue
        if choice in ("r", "relacher", "release"):
            returned_home = _move_joints(
                node,
                "Revenir au home du bras gauche en gardant les ciseaux",
                LEFT_ARM_INDEX,
                LEFT_ARM_JOINTS,
                HOME_LEFT_ARM,
            )
            if not returned_home:
                continue
            opened = _move_joints(
                node,
                "Relacher les ciseaux en ouvrant la main gauche",
                LEFT_HAND_INDEX,
                LEFT_HAND_JOINTS,
                HAND_OPEN,
            )
            if opened:
                return
            continue
        translation_axis: Optional[str] = None
        direction = 1.0
        if choice in TRANSLATION_AXES:
            translation_axis = choice
        elif (
            len(choice) == 2
            and choice[0] in TRANSLATION_AXES
            and choice[1] in "+-"
        ):
            translation_axis = choice[0]
            direction = 1.0 if choice[1] == "+" else -1.0
        elif (
            len(choice) == 3
            and choice[0] == "r"
            and choice[1] in TRANSLATION_AXES
            and choice[2] in "+-"
        ):
            direction = 1.0 if choice[2] == "+" else -1.0
        else:
            print(
                "Choix invalide ; saisir x+/x-, y+/y-, z+/z-, "
                "rx+/rx-, ry+/ry-, rz+/rz-, h, r ou q."
            )
            continue

        current_pose = _wait_for_arm_pose(node)
        if current_pose is None:
            raise TimeoutError(
                f"TF {ARM_COMMAND_FRAME} -> {ARM_FRAME} introuvable "
                "sur le topic robot."
            )
        target = list(current_pose)
        current_tool_orientation = _multiply_quaternions(
            current_pose[3:7],
            node._tool_pose_in_arm[3:7],
        )
        if translation_axis is not None:
            tool_axis_in_command_frame = _rotate_vector_by_quaternion(
                TRANSLATION_AXES[translation_axis],
                current_tool_orientation,
            )
            for index, component in enumerate(tool_axis_in_command_frame):
                target[index] += direction * TRANSLATION_METERS * component
            sign = "+" if direction > 0 else "-"
            _move_cartesian(
                node,
                target,
                f"la translation {sign}10 cm selon l'axe "
                f"{translation_axis} de {TOOL_FRAME}",
            )
        else:
            axis = choice[1]
            half_angle = math.radians(direction * ROTATION_DEGREES) / 2.0
            sine = math.sin(half_angle)
            delta_rotation = [
                math.cos(half_angle),
                sine if axis == "x" else 0.0,
                sine if axis == "y" else 0.0,
                sine if axis == "z" else 0.0,
            ]
            target_tool_orientation = _multiply_quaternions(
                current_tool_orientation,
                delta_rotation,
            )
            target_orientation = _multiply_quaternions(
                target_tool_orientation,
                _conjugate_quaternion(node._tool_pose_in_arm[3:7]),
            )
            target = _pose_rotated_around_tool_tip(
                current_pose,
                target_orientation,
                node._tool_pose_in_arm,
            )
            sign = "+" if direction > 0 else "-"
            _move_cartesian(
                node,
                target,
                f"la rotation {sign}{ROTATION_DEGREES:.0f} degres autour de "
                f"l'axe {axis} de {TOOL_FRAME}",
                check_orientation=True,
            )


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Saisir des ciseaux avec la main gauche et les deplacer."
    )
    robot_id = os.environ.get("ROBOT_ID", "ROBOT_10")
    parser.add_argument("--robot-id", default=robot_id)
    parser.add_argument("--joint-state-topic", default="")
    parser.add_argument("--arm-tf-topic", default="")
    urdf_relative_path = (
        Path("px_robot_code")
        / "nav"
        / "control_location"
        / "pointcloud2_perception"
        / "share"
        / "pointcloud2_perception"
        / "URDF"
        / "tora_double_one_x_description"
        / "urdf"
        / "tora_double_one_x.urdf"
    )
    robot_urdf_path = Path("/home/user") / urdf_relative_path
    workspace_urdf_path = Path(__file__).resolve().parents[1] / urdf_relative_path
    default_urdf_path = (
        robot_urdf_path if robot_urdf_path.is_file() else workspace_urdf_path
    )
    parser.add_argument("--urdf-path", default=str(default_urdf_path))
    options, _ = parser.parse_known_args(args=args)

    joint_topic = options.joint_state_topic.strip() or (
        f"/{options.robot_id}/mc/pr10_joint_states_topic"
    )
    tf_topic = options.arm_tf_topic.strip() or (
        f"/{options.robot_id}/mc/pr10_arm_tf_topic"
    )
    if not joint_topic.startswith("/"):
        joint_topic = "/" + joint_topic
    if not tf_topic.startswith("/"):
        tf_topic = "/" + tf_topic

    print(
        "Avant de commencer, degagez la zone de mouvement et positionnez "
        "les ciseaux pour leur saisie. Les poses de doigts proviennent "
        "des mesures de la main droite et doivent etre validees a gauche."
    )
    if not _confirm("Demarrer la sequence home puis saisie des ciseaux"):
        print("Sequence annulee.")
        return

    rclpy.init(args=args)
    node = LeftScissorsManipulation(joint_topic, tf_topic, Path(options.urdf_path))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        if not _move_joints(
            node,
            "Placer le bras gauche en position home",
            LEFT_ARM_INDEX,
            LEFT_ARM_JOINTS,
            HOME_LEFT_ARM,
        ):
            return
        if not _move_joints(
            node,
            "Ouvrir la main gauche avant la saisie",
            LEFT_HAND_INDEX,
            LEFT_HAND_JOINTS,
            HAND_OPEN,
        ):
            return

        for description, pose in SCISSORS_GRASP:
            if not _move_scissors_grasp_step(
                node,
                f"{description.capitalize()} (pose mesuree)",
                pose,
            ):
                print("Saisie interrompue par l'utilisateur.")
                return

        print(
            f"\nLe repere {TOOL_FRAME} est fixe par rapport a {PALM_FRAME} : "
            f"x={TOOL_OFFSET_IN_PALM[0]:.2f} m, "
            f"y={TOOL_OFFSET_IN_PALM[1]:.2f} m, "
            f"z={TOOL_OFFSET_IN_PALM[2]:.2f} m, "
            f"rotation autour de x={TOOL_ROTATION_X_DEGREES:.0f} deg."
        )
        print(
            f"En attente de la TF {ARM_COMMAND_FRAME} -> {ARM_FRAME}; "
            "verifiez que le topic TF du robot est disponible."
        )
        if _wait_for_arm_pose(node) is None:
            raise TimeoutError(
                f"TF {ARM_COMMAND_FRAME} -> {ARM_FRAME} non recue en "
                f"{FEEDBACK_TIMEOUT:.0f} s."
            )
        _translation_menu(node)
    except KeyboardInterrupt:
        print("\nArret demande ; aucun mouvement supplementaire ne sera envoye.")
    finally:
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
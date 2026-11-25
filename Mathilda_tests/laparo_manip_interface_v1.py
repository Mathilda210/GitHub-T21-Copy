#!/usr/bin/env python3
"""Interface graphique autonome pour saisir et déplacer les outils laparo."""

import argparse
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Queue
from tkinter import messagebox
from typing import Dict, List, Optional, Sequence, Tuple

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
BASE_FRAME = "waist_link_0"
ARM_COMMAND_FRAME = "waist_link_4"
WAIST_JOINTS = [f"waist_joint_{index}" for index in range(1, 5)]
TOOL_FRAME_SUFFIX = "_tool_tip"

TF_REPUBLISH_PERIOD = 0.1
JOINT_TOLERANCE = 0.10
JOINT_VELOCITY_TOLERANCE = 0.01
GRASP_CONTACT_MIN_PROGRESS = 0.05
GRASP_CONTACT_STALL_SECONDS = 1.5
CARTESIAN_TOLERANCE = 0.01
CARTESIAN_ANGULAR_TOLERANCE = math.radians(2.0)
FEEDBACK_TIMEOUT = 30.0
MOTION_TIMEOUT = 60.0
TRANSLATION_STEP_METERS = 0.01
ROTATION_STEP_DEGREES = 5.0
GUI_POLL_PERIOD_MS = 100

TRANSLATION_AXES: Dict[str, Tuple[float, float, float]] = {
    "x": (1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
}


@dataclass(frozen=True)
class ArmConfig:
    side: str
    label: str
    arm_index: int
    hand_index: int
    arm_joints: List[str]
    hand_joints: List[str]
    home: List[float]
    arm_frame: str
    palm_frame: str
    arm_link_joints: List[str]
    arm_frame_joints: List[str]
    arm_chain_frames: List[str]


def _make_arm_config(side: str) -> ArmConfig:
    arm_joints = [f"{side}_arm_joint_{index}" for index in range(7)]
    arm_frame = f"{side}_arm_link_6"
    palm_frame = f"{side}_palm_link"
    hand_joints = [
        f"{side}_{finger}_joint_{index}"
        for finger in ("index", "middle", "ring", "pinky", "thumb")
        for index in range(3)
    ]
    is_left = side == "left"
    return ArmConfig(
        side=side,
        label="gauche" if is_left else "droite",
        arm_index=0 if is_left else 1,
        hand_index=4 if is_left else 5,
        arm_joints=arm_joints,
        hand_joints=hand_joints,
        home=[0.0, 0.4 if is_left else -0.4, 0.0, -1.57, 0.0, 0.0, 0.0],
        arm_frame=arm_frame,
        palm_frame=palm_frame,
        arm_link_joints=arm_joints,
        arm_frame_joints=arm_joints + ["left_palm_joint" if is_left else "right_palm_joint"],
        arm_chain_frames=[f"{side}_arm_link_{index}" for index in range(7)]
        + [palm_frame],
    )


ARM_CONFIGS = {
    "right": _make_arm_config("right"),
    "left": _make_arm_config("left"),
}

LAPARO_POSITION: Dict[str, List[float]] = {
    "left": [-0.930, 1.460, -1.260, -1.800, 0.430, -1.120, -0.340],
    "right": [1.040, -2.060, -0.870, -1.560, 1.110, -0.030, 0.380],
}

ENDOSCOPE_GRASP: List[Tuple[str, List[float]]] = [
    (
        "refermer le pouce",
        [
            0.003, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.023, 0.005, 0.0,
            0.004, 0.0, 0.0,
            1.534, 1.189, 1.686,
        ],
    ),
    (
        "replier l'index sur l'endoscope",
        [
            0.719, 1.049, 0.880,
            0.0, 0.0, 0.0,
            0.023, 0.005, 0.0,
            0.004, 0.0, 0.0,
            1.534, 1.120, 1.838,
        ],
    ),
]

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


@dataclass(frozen=True)
class ToolSetup:
    arm: ArmConfig
    tool_key: str
    tool_label: str
    grasp_steps: List[Tuple[str, List[float]]]
    tool_offset_in_palm: Tuple[float, float, float]
    tool_rotation_x_degrees: float
    tool_frame: str


def _make_tool_setup(side: str, tool_key: str) -> ToolSetup:
    arm = ARM_CONFIGS[side]
    if tool_key == "endoscope":
        return ToolSetup(
            arm=arm,
            tool_key=tool_key,
            tool_label="endoscope",
            grasp_steps=ENDOSCOPE_GRASP,
            tool_offset_in_palm=(0.25, 0.01, 0.0),
            tool_rotation_x_degrees=0.0,
            tool_frame=f"{side}_endoscope{TOOL_FRAME_SUFFIX}",
        )
    if tool_key == "ciseaux":
        # La géométrie gauche est le miroir de celle du script droit et reste à valider.
        return ToolSetup(
            arm=arm,
            tool_key=tool_key,
            tool_label="ciseaux",
            grasp_steps=SCISSORS_GRASP,
            tool_offset_in_palm=(0.06, 0.30 if side == "left" else -0.30, -0.35),
            tool_rotation_x_degrees=-45.0 if side == "left" else 45.0,
            tool_frame=f"{side}_ciseaux{TOOL_FRAME_SUFFIX}",
        )
    raise ValueError(f"Outil inconnu : {tool_key!r}.")


@dataclass(frozen=True)
class JointTransform:
    parent: str
    child: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    joint_type: str


@dataclass(frozen=True)
class WorkflowStep:
    side: str
    action: str
    description: str
    target: Optional[List[float]] = None
    allow_contact: bool = False


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


def _load_urdf_joints(
    urdf_path: Path,
    setups: Dict[str, ToolSetup],
) -> Dict[str, JointTransform]:
    root = ET.parse(str(urdf_path)).getroot()
    required_names = list(WAIST_JOINTS)
    for setup in setups.values():
        required_names.extend(setup.arm.arm_frame_joints)

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
    ]
    for setup in setups.values():
        arm = setup.arm
        expected_chain.append(
            (arm.arm_link_joints[0], ARM_COMMAND_FRAME, arm.arm_chain_frames[0])
        )
        expected_chain.extend(
            (
                arm.arm_link_joints[index],
                arm.arm_chain_frames[index - 1],
                arm.arm_chain_frames[index],
            )
            for index in range(1, len(arm.arm_link_joints))
        )
        expected_chain.append(
            (arm.arm_frame_joints[-1], arm.arm_frame, arm.palm_frame)
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


def _multiply_quaternions(
    first: Sequence[float],
    second: Sequence[float],
) -> List[float]:
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
    first_norm = math.sqrt(sum(component * component for component in first))
    second_norm = math.sqrt(sum(component * component for component in second))
    if first_norm == 0.0 or second_norm == 0.0:
        raise ValueError("Un quaternion nul ne definit pas une orientation.")
    dot = abs(
        sum(a * b for a, b in zip(first, second))
        / (first_norm * second_norm)
    )
    return 2.0 * math.acos(min(1.0, dot))


def _rotate_vector_by_quaternion(
    vector: Sequence[float],
    quaternion: Sequence[float],
) -> List[float]:
    q_w, q_x, q_y, q_z = quaternion
    vector_x, vector_y, vector_z = vector
    cross_x = 2.0 * (q_y * vector_z - q_z * vector_y)
    cross_y = 2.0 * (q_z * vector_x - q_x * vector_z)
    cross_z = 2.0 * (q_x * vector_y - q_y * vector_x)
    return [
        vector_x + q_w * cross_x + q_y * cross_z - q_z * cross_y,
        vector_y + q_w * cross_y + q_z * cross_x - q_x * cross_z,
        vector_z + q_w * cross_z + q_x * cross_y - q_y * cross_x,
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


def _pose_rotated_around_tool_tip(
    pose: Sequence[float],
    target_orientation: Sequence[float],
    tool_pose_in_arm: Sequence[float],
) -> List[float]:
    current_tip_offset = _rotate_vector_by_quaternion(
        tool_pose_in_arm[:3],
        pose[3:7],
    )
    tip_position = [pose[index] + current_tip_offset[index] for index in range(3)]
    target_tip_offset = _rotate_vector_by_quaternion(
        tool_pose_in_arm[:3],
        target_orientation,
    )
    return [
        tip_position[index] - target_tip_offset[index] for index in range(3)
    ] + list(target_orientation)


class LaparoManipulationNode(Node):
    """Commandes ROS et repères cinématiques des bras sélectionnés."""

    def __init__(
        self,
        joint_topic: str,
        tf_topic: str,
        urdf_path: Path,
        setups: Dict[str, ToolSetup],
    ) -> None:
        super().__init__("laparo_manipulation_interface")
        self.setups = dict(setups)
        self._motion_publisher = self.create_publisher(
            OnlineMotionCommand, ONLINE_MOTION_TOPIC, 10
        )
        self._joint_positions: Dict[str, float] = {}
        self._joint_velocities: Dict[str, float] = {}
        self._arm_poses: Dict[str, Optional[List[float]]] = {
            side: None for side in setups
        }
        self._arm_pose_received_at: Dict[str, float] = {
            side: 0.0 for side in setups
        }
        self._urdf_joints = _load_urdf_joints(urdf_path, setups)
        self._tool_poses_in_arm: Dict[str, List[float]] = {}
        for side, setup in setups.items():
            palm_joint_name = setup.arm.arm_frame_joints[-1]
            palm_pose = _joint_pose(
                self._urdf_joints[palm_joint_name],
                {},
                palm_joint_name,
            )
            half_rotation = math.radians(setup.tool_rotation_x_degrees) / 2.0
            tool_pose_in_palm = list(setup.tool_offset_in_palm) + [
                math.cos(half_rotation),
                math.sin(half_rotation),
                0.0,
                0.0,
            ]
            self._tool_poses_in_arm[side] = _compose_poses(
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
        self.create_timer(TF_REPUBLISH_PERIOD, self._publish_arm_chains)
        for setup in setups.values():
            self._publish_tool_transform(setup)

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
            for side, setup in self.setups.items():
                if parent != ARM_COMMAND_FRAME or child != setup.arm.arm_frame:
                    continue
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self._arm_poses[side] = [
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                    float(rotation.w),
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                ]
                self._arm_pose_received_at[side] = time.monotonic()

    def _publish_arm_chains(self) -> None:
        required_joint_names = list(WAIST_JOINTS)
        for setup in self.setups.values():
            required_joint_names.extend(setup.arm.arm_frame_joints[:-1])
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
        for setup in self.setups.values():
            arm = setup.arm
            for joint_name in arm.arm_link_joints:
                joint = self._urdf_joints[joint_name]
                transforms.append(
                    self._transform_from_pose(
                        joint.parent,
                        joint.child,
                        _joint_pose(joint, self._joint_positions, joint_name),
                        now,
                    )
                )
            palm_joint_name = arm.arm_frame_joints[-1]
            palm_joint = self._urdf_joints[palm_joint_name]
            transforms.append(
                self._transform_from_pose(
                    palm_joint.parent,
                    palm_joint.child,
                    _joint_pose(
                        palm_joint,
                        self._joint_positions,
                        palm_joint_name,
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

    def _publish_tool_transform(self, setup: ToolSetup) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = setup.arm.palm_frame
        transform.child_frame_id = setup.tool_frame
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = setup.tool_offset_in_palm
        half_rotation = math.radians(setup.tool_rotation_x_degrees) / 2.0
        transform.transform.rotation.w = math.cos(half_rotation)
        transform.transform.rotation.x = math.sin(half_rotation)
        self._static_tf_broadcaster.sendTransform(transform)

    def publish_joint_target(
        self,
        side: str,
        target_arm_index: int,
        target: Sequence[float],
    ) -> None:
        if self.count_subscribers(ONLINE_MOTION_TOPIC) == 0:
            raise RuntimeError(
                f"Aucun abonne au topic de commande {ONLINE_MOTION_TOPIC}."
            )
        setup = self.setups[side]
        command = OnlineMotionCommand()
        command.type = OnlineMotionCommand.JOINT_TRAJECTORY
        command.target_arm_indices = [target_arm_index]
        command.target_point = [self._joint_point(target)]
        command.planner = OnlineMotionCommand.PLANNER_RUCKIG
        command.max_velocity = [ArmConstraintParameters(values=[0.4])]
        command.max_acceleration = [ArmConstraintParameters(values=[0.5])]
        command.max_jerk = [ArmConstraintParameters(values=[5.0])]
        command.is_whole_body = False
        command.reference_frame = "world_frame"
        self._publish(command)

    def publish_cartesian_target(
        self,
        side: str,
        target: Sequence[float],
    ) -> None:
        if self.count_subscribers(ONLINE_MOTION_TOPIC) == 0:
            raise RuntimeError(
                f"Aucun abonne au topic de commande {ONLINE_MOTION_TOPIC}."
            )
        setup = self.setups[side]
        command = OnlineMotionCommand()
        command.type = OnlineMotionCommand.CARTESIAN_TRAJECTORY
        command.target_arm_indices = [setup.arm.arm_index]
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


def _wait_for_joint_target(
    node: LaparoManipulationNode,
    side: str,
    joints: Sequence[str],
    target: Sequence[float],
) -> None:
    if len(joints) != len(target):
        raise ValueError("Le nombre de joints ne correspond pas à la cible.")
    deadline = time.monotonic() + FEEDBACK_TIMEOUT
    while rclpy.ok() and time.monotonic() < deadline:
        if all(
            name in node._joint_positions
            and abs(node._joint_positions[name] - value) <= JOINT_TOLERANCE
            for name, value in zip(joints, target)
        ):
            return
        time.sleep(0.1)
    raise TimeoutError(
        f"Retour articulaire non conforme après {FEEDBACK_TIMEOUT:.0f} s "
        f"pour le bras {side}."
    )


def _move_scissors_grasp_step(
    node: LaparoManipulationNode,
    side: str,
    target: Sequence[float],
) -> None:
    joints = node.setups[side].arm.hand_joints
    if len(joints) != len(target):
        raise ValueError("La pose de saisie ne correspond pas aux joints de la main.")
    initial_error: Optional[float] = None
    if all(name in node._joint_positions for name in joints):
        initial_error = sum(
            abs(node._joint_positions[name] - value)
            for name, value in zip(joints, target)
        )
    node.publish_joint_target(
        side,
        node.setups[side].arm.hand_index,
        target,
    )
    progress_detected = False
    stall_since: Optional[float] = None
    deadline = time.monotonic() + FEEDBACK_TIMEOUT
    while rclpy.ok() and time.monotonic() < deadline:
        if all(name in node._joint_positions for name in joints):
            errors = [
                abs(node._joint_positions[name] - value)
                for name, value in zip(joints, target)
            ]
            total_error = sum(errors)
            if max(errors) <= JOINT_TOLERANCE:
                return
            if initial_error is None:
                initial_error = total_error
            elif initial_error - total_error >= GRASP_CONTACT_MIN_PROGRESS:
                progress_detected = True
            if progress_detected and all(
                name in node._joint_velocities for name in joints
            ):
                max_velocity = max(
                    abs(node._joint_velocities[name]) for name in joints
                )
                if max_velocity <= JOINT_VELOCITY_TOLERANCE:
                    if stall_since is None:
                        stall_since = time.monotonic()
                    elif time.monotonic() - stall_since >= GRASP_CONTACT_STALL_SECONDS:
                        return
                else:
                    stall_since = None
        time.sleep(0.1)
    raise TimeoutError(
        f"La pose des ciseaux ou un contact stable n'a pas été confirmé "
        f"en {FEEDBACK_TIMEOUT:.0f} s."
    )


def _move_tool_tip(
    node: LaparoManipulationNode,
    side: str,
    axis: str,
    direction: float,
    is_rotation: bool,
    stop_event: Optional[threading.Event] = None,
) -> None:
    current_pose = node._arm_poses[side]
    if (
        current_pose is None
        or node._arm_pose_received_at[side] < time.monotonic() - 1.0
    ):
        raise RuntimeError(
            f"Aucune pose récente du bras {node.setups[side].arm.label} "
            "n'est disponible. Vérifiez le topic TF du robot."
        )
    current_pose = list(current_pose)
    tool_pose_in_arm = node._tool_poses_in_arm[side]
    tool_orientation = _multiply_quaternions(
        current_pose[3:7],
        tool_pose_in_arm[3:7],
    )
    target = list(current_pose)
    if is_rotation:
        half_angle = math.radians(direction * ROTATION_STEP_DEGREES) / 2.0
        sine = math.sin(half_angle)
        delta_rotation = [
            math.cos(half_angle),
            sine if axis == "x" else 0.0,
            sine if axis == "y" else 0.0,
            sine if axis == "z" else 0.0,
        ]
        target_tool_orientation = _multiply_quaternions(
            tool_orientation,
            delta_rotation,
        )
        target_orientation = _multiply_quaternions(
            target_tool_orientation,
            _conjugate_quaternion(tool_pose_in_arm[3:7]),
        )
        target = _pose_rotated_around_tool_tip(
            current_pose,
            target_orientation,
            tool_pose_in_arm,
        )
    else:
        tool_axis = _rotate_vector_by_quaternion(
            TRANSLATION_AXES[axis],
            tool_orientation,
        )
        for index, component in enumerate(tool_axis):
            target[index] += direction * TRANSLATION_STEP_METERS * component

    start_received_at = node._arm_pose_received_at[side]
    if stop_event is not None and not stop_event.is_set():
        return
    node.publish_cartesian_target(side, target)
    deadline = time.monotonic() + MOTION_TIMEOUT
    while rclpy.ok() and time.monotonic() < deadline:
        current = node._arm_poses[side]
        received_at = node._arm_pose_received_at[side]
        if (
            current is None
            or received_at <= start_received_at
            or received_at < time.monotonic() - 1.0
        ):
            time.sleep(0.1)
            continue
        position_error = max(
            abs(current[index] - target[index]) for index in range(3)
        )
        orientation_reached = (
            not is_rotation
            or _quaternion_angular_error(current[3:7], target[3:7])
            <= CARTESIAN_ANGULAR_TOLERANCE
        )
        if position_error <= CARTESIAN_TOLERANCE and orientation_reached:
            return
        time.sleep(0.1)
    raise TimeoutError(
        f"La cible cartesienne du bras {node.setups[side].arm.label} "
        f"n'a pas été confirmée en {MOTION_TIMEOUT:.0f} s."
    )


class LaparoManipulationInterface:
    """Parcours de sélection, prise en main, choix d'action et guidage."""

    def __init__(
        self,
        robot_id: str,
        joint_topic: str,
        tf_topic: str,
        urdf_path: Path,
    ) -> None:
        self.robot_id = robot_id
        self.joint_topic = joint_topic
        self.tf_topic = tf_topic
        self.urdf_path = urdf_path
        self.root = tk.Tk()
        self.root.title("Manipulation des outils laparoscopiques")
        self.root.minsize(580, 420)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self._content = tk.Frame(self.root, padx=14, pady=14)
        self._content.pack(fill="both", expand=True)
        self._selection_vars: Dict[str, tk.StringVar] = {}
        self._active_tools: Dict[str, ToolSetup] = {}
        self._node: Optional[LaparoManipulationNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._results: Queue[Tuple[str, bool, str]] = Queue()
        self._busy = False
        self._busy_kind = ""
        self._workflow: List[WorkflowStep] = []
        self._workflow_kind = ""
        self._workflow_index = 0
        self._grasp_complete = False
        self._control_buttons: List[tk.Button] = []
        self._jog_stop_event: Optional[threading.Event] = None
        self._jog_lock = threading.Lock()
        self._jog_button: Optional[tk.Button] = None
        self._status_var: Optional[tk.StringVar] = None
        self._render_selection()
        self.root.after(GUI_POLL_PERIOD_MS, self._poll_results)

    def _clear_content(self) -> None:
        for child in self._content.winfo_children():
            child.destroy()

    def _render_selection(self, status: str = "") -> None:
        self._clear_content()
        self._control_buttons = []
        tk.Label(
            self._content,
            text="Choisir les outils et leur main",
            font=("TkDefaultFont", 16, "bold"),
        ).pack(pady=(0, 8))
        tk.Label(
            self._content,
            text=(
                "Choisissez aucun outil, un outil ou un outil par main. "
                "La préparation et chaque mouvement seront confirmés."
            ),
            wraplength=540,
            justify="left",
        ).pack(fill="x", pady=(0, 14))
        self._selection_vars = {}
        options = ("Aucun", "Endoscope", "Ciseaux")
        for side in ("right", "left"):
            arm = ARM_CONFIGS[side]
            row = tk.Frame(self._content)
            row.pack(fill="x", pady=6)
            tk.Label(row, text=f"Main {arm.label} :", width=18, anchor="w").pack(
                side="left"
            )
            default = tk.StringVar(master=self.root, value="Aucun")
            self._selection_vars[side] = default
            tk.OptionMenu(row, default, *options).pack(side="left")
        tk.Button(
            self._content,
            text="Confirmer le choix et démarrer la prise en main",
            command=self._confirm_selection,
            width=44,
        ).pack(pady=18)
        self._status_var = tk.StringVar(master=self.root, value=status)
        tk.Label(
            self._content,
            textvariable=self._status_var,
            wraplength=540,
            justify="left",
        ).pack(fill="x", pady=(8, 0))

    def _confirm_selection(self) -> None:
        chosen = {
            side: variable.get()
            for side, variable in self._selection_vars.items()
            if variable.get() != "Aucun"
        }
        if not chosen:
            messagebox.showwarning(
                "Aucun outil",
                "Sélectionnez au moins un outil.",
                parent=self.root,
            )
            return
        tool_keys = {
            "Endoscope": "endoscope",
            "Ciseaux": "ciseaux",
        }
        setups = {
            side: _make_tool_setup(side, tool_keys[tool])
            for side, tool in chosen.items()
        }
        summary = "\n".join(
            f"• {setup.tool_label.capitalize()} dans la main {setup.arm.label}"
            for setup in setups.values()
        )
        extra_warning = ""
        if any(setup.tool_key == "ciseaux" for setup in setups.values()):
            extra_warning = (
                "\n\nLes poses de ciseaux sont celles mesurées pour la main droite. "
                "La configuration gauche est à valider avant utilisation."
            )
        if not messagebox.askyesno(
            "Confirmer la préparation",
            "Dégagez la zone et positionnez les outils conformément à la séquence.\n\n"
            f"{summary}{extra_warning}\n\nDémarrer ?",
            parent=self.root,
        ):
            return
        try:
            self._node = LaparoManipulationNode(
                self.joint_topic,
                self.tf_topic,
                self.urdf_path,
                setups,
            )
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(
                target=self._executor.spin,
                daemon=True,
            )
            self._spin_thread.start()
        except (OSError, ValueError, RuntimeError, ET.ParseError) as error:
            self._stop_ros()
            messagebox.showerror(
                "Démarrage impossible",
                str(error),
                parent=self.root,
            )
            return
        self._active_tools = setups
        self._grasp_complete = False
        self._begin_grasp_workflow()

    def _build_grasp_workflow(self) -> List[WorkflowStep]:
        steps: List[WorkflowStep] = []
        for side, setup in self._active_tools.items():
            arm = setup.arm
            steps.append(
                WorkflowStep(
                    side,
                    "home",
                    f"Placer le bras {arm.label} en position home",
                    list(arm.home),
                )
            )
            steps.append(
                WorkflowStep(
                    side,
                    "open",
                    f"Ouvrir la main {arm.label} avant la saisie",
                    [0.0] * len(arm.hand_joints),
                )
            )
            for description, pose in setup.grasp_steps:
                steps.append(
                    WorkflowStep(
                        side,
                        "grasp",
                        f"{description.capitalize()} dans la main {arm.label} "
                        "(pose mesurée)",
                        list(pose),
                        allow_contact=setup.tool_key == "ciseaux",
                    )
                )
        return steps

    def _build_release_workflow(self) -> List[WorkflowStep]:
        steps: List[WorkflowStep] = []
        for side, setup in self._active_tools.items():
            arm = setup.arm
            steps.append(
                WorkflowStep(
                    side,
                    "home",
                    f"Revenir au home avec l'outil dans la main {arm.label}",
                    list(arm.home),
                )
            )
            steps.append(
                WorkflowStep(
                    side,
                    "open",
                    f"Relâcher l'outil en ouvrant la main {arm.label}",
                    [0.0] * len(arm.hand_joints),
                )
            )
        return steps

    def _build_home_workflow(self) -> List[WorkflowStep]:
        return [
            WorkflowStep(
                side,
                "home",
                f"Revenir au home en gardant l'outil dans la main "
                f"{setup.arm.label}",
                list(setup.arm.home),
            )
            for side, setup in self._active_tools.items()
        ]

    def _build_laparo_workflow(self) -> List[WorkflowStep]:
        return [
            WorkflowStep(
                side,
                "laparo",
                f"Placer le bras {setup.arm.label} en position laparo sur le phantom",
                list(LAPARO_POSITION[side]),
            )
            for side, setup in self._active_tools.items()
        ]

    def _begin_grasp_workflow(self) -> None:
        self._begin_workflow("grasp", self._build_grasp_workflow())

    def _begin_workflow(
        self,
        kind: str,
        steps: List[WorkflowStep],
        resume: bool = False,
    ) -> None:
        if self._busy:
            return
        self._workflow_kind = kind
        if not resume:
            self._workflow = steps
            self._workflow_index = 0
        self._render_workflow_progress()
        self._run_next_workflow_step()

    def _render_workflow_progress(self) -> None:
        self._clear_content()
        self._control_buttons = []
        tk.Label(
            self._content,
            text="Séquence guidée",
            font=("TkDefaultFont", 16, "bold"),
        ).pack(pady=(0, 10))
        for setup in self._active_tools.values():
            tk.Label(
                self._content,
                text=f"{setup.tool_label.capitalize()} — main {setup.arm.label}",
                anchor="w",
            ).pack(fill="x", pady=2)
        self._status_var = tk.StringVar(
            master=self.root,
            value="Suivez les confirmations pour chaque mouvement.",
        )
        tk.Label(
            self._content,
            textvariable=self._status_var,
            wraplength=540,
            justify="left",
        ).pack(fill="x", pady=(14, 0))

    def _run_next_workflow_step(self) -> None:
        if self._workflow_index >= len(self._workflow):
            self._finish_workflow()
            return
        step = self._workflow[self._workflow_index]
        if not messagebox.askyesno(
            "Confirmer le mouvement",
            f"{step.description} ?",
            parent=self.root,
        ):
            self._render_options("Séquence en pause. Reprenez-la ou choisissez une action.")
            return
        if self._workflow_kind == "release":
            self._grasp_complete = False
        self._busy = True
        self._busy_kind = "workflow"
        if self._status_var is not None:
            self._status_var.set(f"Mouvement en cours : {step.description}…")
        threading.Thread(
            target=self._run_workflow_step,
            args=(step,),
            daemon=True,
        ).start()

    def _run_workflow_step(self, step: WorkflowStep) -> None:
        try:
            self._execute_workflow_step(step)
        except (RuntimeError, TimeoutError, ValueError) as error:
            self._results.put(("workflow", False, str(error)))
        else:
            self._results.put(("workflow", True, step.description))

    def _execute_workflow_step(self, step: WorkflowStep) -> None:
        if self._node is None or step.target is None:
            raise RuntimeError("Le nœud ROS ou la cible du mouvement est absent.")
        setup = self._active_tools[step.side]
        if step.action in ("home", "laparo"):
            joints = setup.arm.arm_joints
            target_arm_index = setup.arm.arm_index
        elif step.action in ("open", "grasp"):
            joints = setup.arm.hand_joints
            target_arm_index = setup.arm.hand_index
        else:
            raise ValueError(f"Action de séquence inconnue : {step.action!r}.")
        if step.action == "grasp" and step.allow_contact:
            _move_scissors_grasp_step(self._node, step.side, step.target)
            return
        self._node.publish_joint_target(
            step.side,
            target_arm_index,
            step.target,
        )
        _wait_for_joint_target(self._node, step.side, joints, step.target)

    def _finish_workflow(self) -> None:
        kind = self._workflow_kind
        self._workflow_kind = ""
        self._workflow = []
        self._workflow_index = 0
        if kind == "release":
            self._active_tools = {}
            self._grasp_complete = False
            self._stop_ros()
            self._render_selection("Outils relâchés. Vous pouvez choisir une nouvelle configuration.")
        elif kind == "grasp":
            self._grasp_complete = True
            self._render_options("Prise en main terminée. Choisissez l'action suivante.")
        elif kind == "laparo":
            self._render_options("Position laparo atteinte. Choisissez l'action suivante.")
        else:
            self._render_options("Position home atteinte. Les outils restent saisis.")

    def _render_options(self, status: str = "") -> None:
        self._clear_content()
        self._control_buttons = []
        tk.Label(
            self._content,
            text="Que souhaitez-vous faire ?",
            font=("TkDefaultFont", 16, "bold"),
        ).pack(pady=(0, 12))
        tk.Label(
            self._content,
            text=(
                "Après la saisie, les bras sont au home. Le guidage démarre "
                "depuis la position actuelle; vous pouvez d'abord choisir "
                "la position laparo sur le phantom."
            ),
            wraplength=540,
            justify="left",
        ).pack(fill="x", pady=(0, 8))
        for setup in self._active_tools.values():
            tk.Label(
                self._content,
                text=f"{setup.tool_label.capitalize()} — main {setup.arm.label}",
                anchor="w",
            ).pack(fill="x", pady=2)
        if not self._grasp_complete:
            tk.Button(
                self._content,
                text="Reprendre la séquence de prise en main",
                command=self._resume_grasp_workflow,
                width=44,
            ).pack(pady=(16, 5))
        tk.Button(
            self._content,
            text="Relâcher l'outil ou les outils",
            command=self._release_tools,
            width=44,
        ).pack(pady=5)
        tk.Button(
            self._content,
            text="Revenir en position home (garder les outils)",
            command=self._return_home,
            width=44,
        ).pack(pady=5)
        laparo_button = tk.Button(
            self._content,
            text="Mettre en position laparo sur le phantom",
            command=self._move_to_laparo_position,
            width=44,
            state=tk.NORMAL if self._grasp_complete else tk.DISABLED,
        )
        laparo_button.pack(pady=5)
        control_button = tk.Button(
            self._content,
            text="Ouvrir le guidage depuis la position actuelle",
            command=self._open_guidance,
            width=44,
            state=tk.NORMAL if self._grasp_complete else tk.DISABLED,
        )
        control_button.pack(pady=5)
        self._status_var = tk.StringVar(master=self.root, value=status)
        tk.Label(
            self._content,
            textvariable=self._status_var,
            wraplength=540,
            justify="left",
        ).pack(fill="x", pady=(12, 0))

    def _resume_grasp_workflow(self) -> None:
        if self._workflow_kind == "grasp" and self._workflow:
            self._begin_workflow("grasp", self._workflow, resume=True)
        else:
            self._begin_grasp_workflow()

    def _release_tools(self) -> None:
        if not self._active_tools:
            return
        self._begin_workflow("release", self._build_release_workflow())

    def _return_home(self) -> None:
        self._begin_workflow("home", self._build_home_workflow())

    def _move_to_laparo_position(self) -> None:
        if not self._grasp_complete:
            return
        self._begin_workflow("laparo", self._build_laparo_workflow())

    def _open_guidance(self) -> None:
        if not self._grasp_complete:
            return
        self._render_guidance()

    def _render_guidance(self) -> None:
        self._clear_content()
        self._control_buttons = []
        tk.Label(
            self._content,
            text=(
                "Guidage — pas de 1 cm et rotations de 5°\n"
                "Maintenir un bouton pour jogger; le relâcher pour arrêter."
            ),
            font=("TkDefaultFont", 12, "bold"),
            justify="center",
        ).pack(pady=(0, 12))
        panels = tk.Frame(self._content)
        panels.pack(fill="x", expand=True)
        self._control_buttons = []
        for column, (side, setup) in enumerate(self._active_tools.items()):
            panel = tk.LabelFrame(
                panels,
                text=f"{setup.tool_label.capitalize()} — main {setup.arm.label}",
                padx=8,
                pady=8,
            )
            panel.grid(row=0, column=column, padx=6, pady=4, sticky="n")
            self._build_axis_controls(panel, side, is_rotation=False)
            self._build_axis_controls(panel, side, is_rotation=True)
        back_button = tk.Button(
            self._content,
            text="Sortie — revenir au choix (relâcher / home / guidage)",
            command=self._back_to_options,
        )
        back_button.pack(pady=(12, 4))
        self._control_buttons.append(back_button)
        self._status_var = tk.StringVar(
            master=self.root,
            value="Choisissez un axe. Une pression courte fait un pas; maintenir répète le jog.",
        )
        tk.Label(
            self._content,
            textvariable=self._status_var,
            wraplength=760,
            justify="left",
        ).pack(fill="x", pady=(6, 0))

    def _build_axis_controls(
        self,
        parent: tk.Widget,
        side: str,
        is_rotation: bool,
    ) -> None:
        title = "Rotations autour des axes de l'outil" if is_rotation else "Translations selon les axes de l'outil"
        frame = tk.LabelFrame(parent, text=title, padx=6, pady=5)
        frame.pack(fill="x", pady=5)
        for column, axis in enumerate(("x", "y", "z")):
            tk.Label(frame, text=axis.upper()).grid(
                row=0,
                column=column,
                padx=5,
                pady=(0, 4),
            )
            for row, direction in enumerate((-1.0, 1.0), start=1):
                sign = "-" if direction < 0 else "+"
                text = f"R{axis.upper()}{sign}" if is_rotation else f"{axis.upper()}{sign}"
                button = tk.Button(frame, text=text, width=7)
                button.bind(
                    "<ButtonPress-1>",
                    lambda event, s=side, a=axis, d=direction, r=is_rotation:
                    self._start_jog(s, a, d, r, event.widget),
                )
                button.bind("<ButtonRelease-1>", self._stop_jog)
                button.grid(row=row, column=column, padx=5, pady=3)
                self._control_buttons.append(button)

    def _start_jog(
        self,
        side: str,
        axis: str,
        direction: float,
        is_rotation: bool,
        button: tk.Button,
    ) -> None:
        if self._busy or self._node is None:
            return
        sign = "+" if direction > 0 else "-"
        description = (
            f"rotation {sign}{ROTATION_STEP_DEGREES:.0f}° autour de "
            f"{axis.upper()} — main {self._active_tools[side].arm.label}"
            if is_rotation
            else f"translation {sign}1 cm selon {axis.upper()} — "
            f"main {self._active_tools[side].arm.label}"
        )
        stop_event = threading.Event()
        stop_event.set()
        self._jog_stop_event = stop_event
        self._jog_button = button
        self._busy = True
        self._busy_kind = "jog"
        self._set_control_buttons_enabled(False, active_button=button)
        if self._status_var is not None:
            self._status_var.set(
                f"Jog en cours : {description}. Relâchez pour arrêter."
            )
        threading.Thread(
            target=self._run_jog,
            args=(side, axis, direction, is_rotation, description, stop_event),
            daemon=True,
        ).start()

    def _stop_jog(self, event) -> None:
        if event.widget is self._jog_button:
            with self._jog_lock:
                if self._jog_stop_event is not None:
                    self._jog_stop_event.clear()

    def _run_jog(
        self,
        side: str,
        axis: str,
        direction: float,
        is_rotation: bool,
        description: str,
        stop_event: threading.Event,
    ) -> None:
        try:
            while stop_event.is_set():
                if self._node is None:
                    raise RuntimeError("Le nœud ROS n'est plus disponible.")
                _move_tool_tip(
                    self._node,
                    side,
                    axis,
                    direction,
                    is_rotation,
                    stop_event,
                )
        except (RuntimeError, TimeoutError, ValueError) as error:
            with self._jog_lock:
                stop_event.clear()
            self._results.put(("jog", False, str(error)))
        else:
            self._results.put(("jog", True, f"Jog arrêté : {description}."))
        finally:
            with self._jog_lock:
                stop_event.clear()

    def _back_to_options(self) -> None:
        if self._busy:
            return
        self._render_options()

    def _set_control_buttons_enabled(
        self,
        enabled: bool,
        active_button: Optional[tk.Button] = None,
    ) -> None:
        for button in self._control_buttons:
            button.configure(
                state=(
                    tk.NORMAL
                    if enabled or button is active_button
                    else tk.DISABLED
                )
            )

    def _poll_results(self) -> None:
        try:
            kind, succeeded, message = self._results.get_nowait()
        except Empty:
            pass
        else:
            self._busy = False
            self._busy_kind = ""
            if self._jog_stop_event is not None:
                self._jog_stop_event.clear()
            self._jog_stop_event = None
            self._jog_button = None
            if kind == "workflow":
                if succeeded:
                    self._workflow_index += 1
                    self._run_next_workflow_step()
                else:
                    messagebox.showerror(
                        "Séquence interrompue",
                        message,
                        parent=self.root,
                    )
                    self._render_options(
                        "La séquence est en pause. Vous pouvez la reprendre "
                        "à l'étape en échec ou choisir home / relâcher."
                    )
            else:
                self._set_control_buttons_enabled(True)
                if self._status_var is not None:
                    self._status_var.set(message)
                if not succeeded:
                    messagebox.showerror(
                        "Déplacement impossible",
                        message,
                        parent=self.root,
                    )
        self.root.after(GUI_POLL_PERIOD_MS, self._poll_results)

    def _stop_ros(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self._spin_thread is not None:
            self._spin_thread.join()
            self._spin_thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None

    def _close(self) -> None:
        if self._busy:
            messagebox.showwarning(
                "Mouvement en cours",
                "Attendez la fin du mouvement avant de fermer l'interface.",
                parent=self.root,
            )
            return
        if self._active_tools:
            confirmed = messagebox.askyesno(
                "Quitter",
                "Les outils peuvent encore être tenus par le robot. "
                "Quitter sans mouvement de relâchement ?",
                parent=self.root,
            )
        else:
            confirmed = messagebox.askyesno(
                "Quitter",
                "Fermer l'interface ?",
                parent=self.root,
            )
        if confirmed:
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def _default_urdf_path() -> Path:
    urdf_name = "tora_double_one_x.urdf"
    candidates: List[Path] = []
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        package_share = None
    else:
        try:
            package_share = Path(
                get_package_share_directory("pointcloud2_perception")
            )
        except PackageNotFoundError:
            package_share = None
    if package_share is not None:
        candidates.extend(
            [
                package_share
                / "URDF"
                / "tora_double_one_x_description"
                / "urdf"
                / urdf_name,
                package_share / urdf_name,
            ]
        )

    script_directory = Path(__file__).resolve().parent
    candidates.extend(
        [
            script_directory.parent / "Others" / urdf_name,
            script_directory / "Others" / urdf_name,
        ]
    )
    return next((candidate for candidate in candidates if candidate.is_file()), candidates[0])


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Choisir, saisir et déplacer un ou deux outils laparoscopiques."
    )
    parser.add_argument("--robot-id", default=os.environ.get("ROBOT_ID", "ROBOT_10"))
    parser.add_argument("--joint-state-topic", default="")
    parser.add_argument("--arm-tf-topic", default="")
    parser.add_argument("--urdf-path", default=str(_default_urdf_path()))
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

    rclpy.init(args=args)
    interface: Optional[LaparoManipulationInterface] = None
    try:
        interface = LaparoManipulationInterface(
            options.robot_id,
            joint_topic,
            tf_topic,
            Path(options.urdf_path),
        )
        interface.run()
    finally:
        if interface is not None:
            interface._stop_ros()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""Interface graphique Tkinter pour déplacer un endoscope avec ROS 2
Main DROITE
Testé et fonctionnel avec jog plutôt propre. """

from dataclasses import dataclass
import argparse
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from queue import Empty, Queue
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import rclpy
import tkinter as tk
from tkinter import messagebox
from geometry_msgs.msg import TransformStamped
from px_mc_msgs.msg import (
    ArmConstraintParameters,
    OnlineMotionCommand,
    TrajectoryPoint,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ONLINE_MOTION_TOPIC = "/external/online_motion_command"
RIGHT_ARM_INDEX = 1
RIGHT_HAND_INDEX = 5
RIGHT_ARM_JOINTS = [f"right_arm_joint_{index}" for index in range(7)]
RIGHT_HAND_JOINTS = [
    f"right_{finger}_joint_{index}"
    for finger in ("index", "middle", "ring", "pinky", "thumb")
    for index in range(3)
]

# Pose home droite relevée dans les exemples du robot.
HOME_RIGHT_ARM = [0.0, -0.4, 0.0, -1.57, 0.0, 0.0, 0.0]
HAND_OPEN = [0.0] * len(RIGHT_HAND_JOINTS)

# Poses de saisie relevées dans Mathilda_tests/Drafts_and_others/poses_3_laparo_hold.
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

BASE_FRAME = "waist_link_0"
ARM_COMMAND_FRAME = "waist_link_4"
ARM_FRAME = "right_arm_link_6"
PALM_FRAME = "right_palm_link"
WAIST_JOINTS = [f"waist_joint_{index}" for index in range(1, 5)]
RIGHT_ARM_LINK_JOINTS = [
    f"right_arm_joint_{index}" for index in range(7)
]
RIGHT_ARM_FRAME_JOINTS = RIGHT_ARM_LINK_JOINTS + ["right_palm_joint"]
ARM_CHAIN_FRAMES = [f"right_arm_link_{index}" for index in range(7)] + [
    PALM_FRAME
]
TOOL_FRAME = "tool_tip"
TOOL_OFFSET_IN_PALM = (0.25, 0.01, 0.0)
TF_REPUBLISH_PERIOD = 0.1
JOINT_TOLERANCE = 0.10
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
    required_names = WAIST_JOINTS + RIGHT_ARM_FRAME_JOINTS
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
        (RIGHT_ARM_LINK_JOINTS[0], ARM_COMMAND_FRAME, ARM_CHAIN_FRAMES[0]),
    ]
    expected_chain.extend(
        (
            RIGHT_ARM_LINK_JOINTS[index],
            ARM_CHAIN_FRAMES[index - 1],
            ARM_CHAIN_FRAMES[index],
        )
        for index in range(1, len(RIGHT_ARM_LINK_JOINTS))
    )
    expected_chain.append(
        (RIGHT_ARM_FRAME_JOINTS[-1], ARM_FRAME, PALM_FRAME)
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
    tool_offset_in_arm: Sequence[float],
) -> List[float]:
    """Build an arm pose that rotates around the fixed tool-tip position."""
    current_tip_offset = _rotate_vector_by_quaternion(
        tool_offset_in_arm,
        pose[3:7],
    )
    tool_tip_position = [
        pose[index] + current_tip_offset[index] for index in range(3)
    ]
    target_tip_offset = _rotate_vector_by_quaternion(
        tool_offset_in_arm,
        target_orientation,
    )
    return [
        tool_tip_position[index] - target_tip_offset[index]
        for index in range(3)
    ] + list(target_orientation)


class EndoscopeManipulation(Node):
    """Envoie les commandes et publie les repères utiles à RViz."""

    def __init__(
        self,
        joint_topic: str,
        tf_topic: str,
        urdf_path: Path,
    ) -> None:
        super().__init__("endoscope_manipulation")
        self._motion_publisher = self.create_publisher(
            OnlineMotionCommand, ONLINE_MOTION_TOPIC, 10
        )
        self._joint_positions: Dict[str, float] = {}
        self._arm_pose: Optional[List[float]] = None
        self._arm_pose_received_at = 0.0
        self._urdf_joints = _load_urdf_joints(urdf_path)
        palm_joint_name = RIGHT_ARM_FRAME_JOINTS[-1]
        palm_pose = _joint_pose(
            self._urdf_joints[palm_joint_name],
            {},
            palm_joint_name,
        )
        tool_pose_in_palm = list(TOOL_OFFSET_IN_PALM) + [1.0, 0.0, 0.0, 0.0]
        self._tool_offset_in_arm = _compose_poses(
            palm_pose,
            tool_pose_in_palm,
        )[:3]
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
        required_joint_names = WAIST_JOINTS + RIGHT_ARM_FRAME_JOINTS[:-1]
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

        for joint_name in RIGHT_ARM_LINK_JOINTS:
            joint = self._urdf_joints[joint_name]
            transforms.append(
                self._transform_from_pose(
                    joint.parent,
                    joint.child,
                    _joint_pose(joint, self._joint_positions, joint_name),
                    now,
                )
            )

        palm_joint = self._urdf_joints[RIGHT_ARM_FRAME_JOINTS[-1]]
        transforms.append(
            self._transform_from_pose(
                palm_joint.parent,
                palm_joint.child,
                _joint_pose(
                    palm_joint,
                    self._joint_positions,
                    RIGHT_ARM_FRAME_JOINTS[-1],
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
        tool_tf.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(tool_tf)
        self.get_logger().info(
            f"TF statique publiee : {PALM_FRAME} -> {TOOL_FRAME}, "
            f"translation locale x={TOOL_OFFSET_IN_PALM[0]:.2f} m, "
            f"y={TOOL_OFFSET_IN_PALM[1]:.2f} m."
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
        command.target_arm_indices = [RIGHT_ARM_INDEX]
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
    node: EndoscopeManipulation,
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


def _wait_for_arm_pose(
    node: EndoscopeManipulation,
) -> Optional[List[float]]:
    if node._arm_pose is None and not _spin_until(
        lambda: node._arm_pose is not None, FEEDBACK_TIMEOUT
    ):
        return None
    return list(node._arm_pose) if node._arm_pose is not None else None




TRANSLATION_STEP_METERS = 0.01
ROTATION_STEP_DEGREES = 5.0
GUI_POLL_PERIOD_MS = 100


class EndoscopeManipulationInterface:
    """Affiche les commandes et exécute chaque mouvement hors du thread Tk."""

    def __init__(self, node: EndoscopeManipulation) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("Déplacement de l'endoscope")
        self.root.resizable(False, False)
        self._busy = False
        self._jog_stop_event: Optional[threading.Event] = None
        self._jog_lock = threading.Lock()
        self._jog_button: Optional[tk.Button] = None
        self._results: Queue[Tuple[bool, str]] = Queue()
        self._motion_buttons = []
        self._status = tk.StringVar(
            master=self.root,
            value="Endoscope saisi. Choisissez un déplacement.",
        )

        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(GUI_POLL_PERIOD_MS, self._poll_results)

    def _build_widgets(self) -> None:
        tk.Label(
            self.root,
            text=(
                "Pas de 1 cm et rotations de 5° "
                "(maintenir le bouton pour jogger)"
            ),
            font=("TkDefaultFont", 12, "bold"),
            padx=12,
            pady=12,
        ).pack()

        translation_frame = tk.LabelFrame(
            self.root,
            text="Translation selon les axes de tool_tip",
            padx=10,
            pady=8,
        )
        translation_frame.pack(fill="x", padx=12, pady=5)
        for column, axis in enumerate(("x", "y", "z")):
            tk.Label(translation_frame, text=axis.upper()).grid(
                row=0,
                column=column,
                padx=8,
                pady=(0, 4),
            )
            for row, direction in enumerate((-1.0, 1.0), start=1):
                sign = "-" if direction < 0 else "+"
                button = tk.Button(
                    translation_frame,
                    text=f"{axis.upper()}{sign}",
                    width=8,
                )
                button.bind(
                    "<ButtonPress-1>",
                    lambda event, a=axis, d=direction:
                    self._start_jog(a, d, False, event.widget),
                )
                button.bind("<ButtonRelease-1>", self._stop_jog)
                button.grid(row=row, column=column, padx=8, pady=3)
                self._motion_buttons.append(button)

        rotation_frame = tk.LabelFrame(
            self.root,
            text="Rotation autour des axes de tool_tip",
            padx=10,
            pady=8,
        )
        rotation_frame.pack(fill="x", padx=12, pady=5)
        for column, axis in enumerate(("x", "y", "z")):
            tk.Label(rotation_frame, text=axis.upper()).grid(
                row=0,
                column=column,
                padx=8,
                pady=(0, 4),
            )
            for row, direction in enumerate((-1.0, 1.0), start=1):
                sign = "-" if direction < 0 else "+"
                button = tk.Button(
                    rotation_frame,
                    text=f"R{axis.upper()}{sign}",
                    width=8,
                )
                button.bind(
                    "<ButtonPress-1>",
                    lambda event, a=axis, d=direction:
                    self._start_jog(a, d, True, event.widget),
                )
                button.bind("<ButtonRelease-1>", self._stop_jog)
                button.grid(row=row, column=column, padx=8, pady=3)
                self._motion_buttons.append(button)

        self._home_button = tk.Button(
            self.root,
            text="Retour au home (garder l'endoscope)",
            command=self._return_home,
        )
        self._home_button.pack(pady=(8, 4))
        self._motion_buttons.append(self._home_button)

        tk.Label(
            self.root,
            textvariable=self._status,
            wraplength=480,
            padx=12,
            pady=10,
        ).pack(fill="x")

    def _fresh_arm_pose(self) -> List[float]:
        pose = self.node._arm_pose
        if (
            pose is None
            or self.node._arm_pose_received_at < time.monotonic() - 1.0
        ):
            raise RuntimeError(
                "Aucune pose récente du bras n'est disponible. "
                "Vérifiez le topic TF du robot."
            )
        return list(pose)

    def _start_jog(
        self,
        axis: str,
        direction: float,
        is_rotation: bool,
        button: tk.Button,
    ) -> None:
        if self._busy:
            return

        sign = "+" if direction > 0 else "-"
        description = (
            f"rotation {sign}{ROTATION_STEP_DEGREES:.0f}° autour de "
            f"{axis.upper()} de {TOOL_FRAME}"
            if is_rotation
            else f"translation {sign}1 cm selon {axis.upper()} de {TOOL_FRAME}"
        )
        stop_event = threading.Event()
        stop_event.set()
        self._jog_stop_event = stop_event
        self._jog_button = button
        self._busy = True
        self._set_controls_enabled(False, active_button=button)
        self._status.set(f"Jog en cours : {description} — relâchez pour arrêter.")
        threading.Thread(
            target=self._run_jog,
            args=(axis, direction, is_rotation, description, stop_event),
            daemon=True,
        ).start()

    def _stop_jog(self, event) -> None:
        if event.widget is self._jog_button:
            with self._jog_lock:
                if self._jog_stop_event is not None:
                    self._jog_stop_event.clear()

    def _run_jog(
        self,
        axis: str,
        direction: float,
        is_rotation: bool,
        description: str,
        stop_event: threading.Event,
    ) -> None:
        result = (False, f"Erreur inattendue pendant : {description}.")
        try:
            while stop_event.is_set():
                current_pose = self._fresh_arm_pose()
                target = list(current_pose)
                if is_rotation:
                    half_angle = math.radians(
                        direction * ROTATION_STEP_DEGREES
                    ) / 2.0
                    sine = math.sin(half_angle)
                    delta_rotation = [
                        math.cos(half_angle),
                        sine if axis == "x" else 0.0,
                        sine if axis == "y" else 0.0,
                        sine if axis == "z" else 0.0,
                    ]
                    target_orientation = _multiply_quaternions(
                        current_pose[3:7],
                        delta_rotation,
                    )
                    target = _pose_rotated_around_tool_tip(
                        current_pose,
                        target_orientation,
                        self.node._tool_offset_in_arm,
                    )
                else:
                    tool_axis = _rotate_vector_by_quaternion(
                        TRANSLATION_AXES[axis],
                        current_pose[3:7],
                    )
                    for index, component in enumerate(tool_axis):
                        target[index] += (
                            direction * TRANSLATION_STEP_METERS * component
                        )

                with self._jog_lock:
                    if not stop_event.is_set():
                        break
                    start_received_at = self.node._arm_pose_received_at
                    self.node.publish_cartesian_target(target)
                self._wait_for_cartesian_target(
                    target,
                    description,
                    is_rotation,
                    start_received_at,
                )
        except (RuntimeError, TimeoutError, ValueError) as error:
            result = (False, str(error))
        else:
            result = (True, f"Jog arrêté : {description}.")
        finally:
            with self._jog_lock:
                stop_event.clear()
            self._results.put(result)

    def _return_home(self) -> None:
        if not messagebox.askyesno(
            "Confirmer le mouvement",
            "Ramener le bras droit au home en gardant l'endoscope ?",
            parent=self.root,
        ):
            return
        self._start_motion(
            "Retour au home terminé.",
            self._move_arm_home,
        )

    def _start_motion(self, description: str, action: Callable[[], None]) -> None:
        if self._busy:
            return
        self._busy = True
        self._set_controls_enabled(False)
        self._status.set(f"Mouvement en cours : {description}...")
        threading.Thread(
            target=self._run_motion,
            args=(description, action),
            daemon=True,
        ).start()

    def _run_motion(self, description: str, action: Callable[[], None]) -> None:
        result = (False, f"Erreur inattendue pendant : {description}.")
        try:
            action()
        except (RuntimeError, TimeoutError, ValueError) as error:
            result = (False, str(error))
        else:
            result = (True, f"Mouvement terminé : {description}.")
        finally:
            self._results.put(result)

    def _move_cartesian(
        self,
        target: Sequence[float],
        description: str,
        check_orientation: bool = False,
    ) -> None:
        start_received_at = self.node._arm_pose_received_at
        self.node.publish_cartesian_target(target)
        self._wait_for_cartesian_target(
            target,
            description,
            check_orientation,
            start_received_at,
        )

    def _wait_for_cartesian_target(
        self,
        target: Sequence[float],
        description: str,
        check_orientation: bool,
        start_received_at: float,
    ) -> None:
        deadline = time.monotonic() + MOTION_TIMEOUT

        while rclpy.ok() and time.monotonic() < deadline:
            current_pose = self.node._arm_pose
            received_at = self.node._arm_pose_received_at
            if (
                current_pose is None
                or received_at <= start_received_at
                or received_at < time.monotonic() - 1.0
            ):
                time.sleep(0.1)
                continue

            position_error = max(
                abs(current_pose[index] - target[index]) for index in range(3)
            )
            orientation_reached = (
                not check_orientation
                or _quaternion_angular_error(
                    current_pose[3:7],
                    target[3:7],
                )
                <= CARTESIAN_ANGULAR_TOLERANCE
            )
            if (
                position_error <= CARTESIAN_TOLERANCE
                and orientation_reached
            ):
                return
            time.sleep(0.1)

        raise TimeoutError(
            f"La pose cible n'a pas été confirmée en "
            f"{MOTION_TIMEOUT:.0f} s : {description}."
        )

    def _move_arm_home(self) -> None:
        self.node.publish_joint_target(
            RIGHT_ARM_INDEX,
            HOME_RIGHT_ARM,
        )

        def target_reached() -> bool:
            return all(
                name in self.node._joint_positions
                and abs(self.node._joint_positions[name] - value)
                <= JOINT_TOLERANCE
                for name, value in zip(
                    RIGHT_ARM_JOINTS,
                    HOME_RIGHT_ARM,
                )
            )

        if not _spin_until(target_reached, FEEDBACK_TIMEOUT):
            raise TimeoutError(
                "Le bras n'a pas atteint sa position home avant le délai imparti."
            )

    def _poll_results(self) -> None:
        try:
            succeeded, message = self._results.get_nowait()
        except Empty:
            pass
        else:
            with self._jog_lock:
                if self._jog_stop_event is not None:
                    self._jog_stop_event.clear()
            self._jog_stop_event = None
            self._jog_button = None
            self._busy = False
            self._set_controls_enabled(True)
            self._status.set(message)
            if not succeeded:
                messagebox.showerror(
                    "Mouvement impossible",
                    message,
                    parent=self.root,
                )
        self.root.after(GUI_POLL_PERIOD_MS, self._poll_results)

    def _set_controls_enabled(
        self,
        enabled: bool,
        active_button: Optional[tk.Button] = None,
    ) -> None:
        for button in self._motion_buttons:
            state = (
                tk.NORMAL
                if enabled or button is active_button
                else tk.DISABLED
            )
            button.configure(state=state)

    def _close(self) -> None:
        if self._busy:
            messagebox.showwarning(
                "Mouvement en cours",
                "Attendez la fin du mouvement avant de fermer l'interface.",
                parent=self.root,
            )
            return
        if messagebox.askyesno(
            "Quitter",
            "Quitter l'interface sans autre mouvement ?",
            parent=self.root,
        ):
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Saisir et déplacer un endoscope avec une interface graphique."
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
        "l'endoscope pour sa saisie."
    )
    if not _confirm(
        "Demarrer la sequence home puis saisie de l'endoscope"
    ):
        print("Sequence annulee.")
        return

    rclpy.init(args=args)
    node = None
    executor = None
    spin_thread = None
    try:
        node = EndoscopeManipulation(
            joint_topic,
            tf_topic,
            Path(options.urdf_path),
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        if not _move_joints(
            node,
            "Placer le bras droit en position home",
            RIGHT_ARM_INDEX,
            RIGHT_ARM_JOINTS,
            HOME_RIGHT_ARM,
        ):
            return
        if not _move_joints(
            node,
            "Ouvrir la main droite avant la saisie",
            RIGHT_HAND_INDEX,
            RIGHT_HAND_JOINTS,
            HAND_OPEN,
        ):
            return

        for description, pose in ENDOSCOPE_GRASP:
            if not _move_joints(
                node,
                f"{description.capitalize()} (pose mesuree)",
                RIGHT_HAND_INDEX,
                RIGHT_HAND_JOINTS,
                pose,
            ):
                print("Saisie interrompue par l'utilisateur.")
                return

        if _wait_for_arm_pose(node) is None:
            raise TimeoutError(
                f"TF {ARM_COMMAND_FRAME} -> {ARM_FRAME} "
                f"non recue en {FEEDBACK_TIMEOUT:.0f} s."
            )
        EndoscopeManipulationInterface(node).run()
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
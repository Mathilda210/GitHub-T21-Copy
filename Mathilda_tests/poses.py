#!/usr/bin/env python3
"""Choix interactif de poses mesurées pour la main droite.
A TESTER AVEC LA MAIN DROITE UNIQUEMENT, DANS UN ESPACE SÛR ET DÉGAGÉ."""

import argparse
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import rclpy
from px_mc_msgs.msg import (
    ArmConstraintParameters,
    OnlineMotionCommand,
    TrajectoryPoint,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


ONLINE_MOTION_TOPIC = "/external/online_motion_command"
FEEDBACK_TIMEOUT = 30.0
POSITION_TOLERANCE = 0.1
VELOCITY_TOLERANCE = 0.01
CONTACT_MIN_PROGRESS = 0.05
CONTACT_STALL_SECONDS = 1.5
COMPARISON_EPSILON = 1e-9
RIGHT_HAND_GROUP = 5
RIGHT_HAND_JOINTS = [
    "right_index_joint_0",
    "right_index_joint_1",
    "right_index_joint_2",
    "right_middle_joint_0",
    "right_middle_joint_1",
    "right_middle_joint_2",
    "right_ring_joint_0",
    "right_ring_joint_1",
    "right_ring_joint_2",
    "right_pinky_joint_0",
    "right_pinky_joint_1",
    "right_pinky_joint_2",
    "right_thumb_joint_0",
    "right_thumb_joint_1",
    "right_thumb_joint_2",
]

POSE_DEFINITIONS = [
    (
        "endoscope",
        "pouce referme",
        [
            0.003, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.023, 0.005, 0.0,
            0.004, 0.0, 0.0,
            1.534, 1.189, 1.686,
        ],
        True,
    ),
    (
        "endoscope",
        "index replie sur l'endoscope",
        [
            0.719, 1.049, 0.880,
            0.0, 0.0, 0.0,
            0.023, 0.005, 0.0,
            0.004, 0.0, 0.0,
            1.534, 1.120, 1.838,
        ],
        True,
    ),
    (
        "ciseaux",
        "enfilage des ciseaux entre pouce et index",
        [
            0.003, 0.0, 0.0,
            1.221, 0.017, 0.007,
            0.016, 0.003, 0.001,
            0.0, 0.0, 0.0,
            1.623, 0.757, 0.0,
        ],
        True,
    ),
    (
        "ciseaux",
        "doigts resserres sur les ciseaux",
        [
            0.003, 0.0, 0.0,
            1.193, 0.201, 0.113,
            0.016, 0.003, 0.001,
            0.0, 0.0, 0.0,
            1.623, 0.978, 0.0,
        ],
        True,
    ),
    (
        "ciseaux",
        "index en appui",
        [
            1.037, 0.247, 0.146,
            1.265, 0.103, 0.057,
            0.016, 0.003, 0.001,
            0.0, 0.0, 0.0,
            1.623, 0.924, 0.0,
        ],
        True,
    ),
    (
        "petite pince",
        "pouce replie",
        [
            0.028, 0.0, 0.0,
            0.003, 0.002, 0.002,
            0.014, 0.0, 0.003,
            0.004, 0.002, 0.005,
            1.675, 1.080, 1.830,
        ],
        True,
    ),
    (
        "petite pince",
        "annulaire et petit doigt en appui",
        [
            0.028, 0.0, 0.0,
            0.003, 0.002, 0.002,
            1.392, 1.208, 1.075,
            1.292, 1.161, 1.016,
            1.675, 1.086, 1.830,
        ],
        True,
    ),
    (
        "main",
        "ouvrir la main (pose home)",
        [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ],
        False,
    ),
]


@dataclass(frozen=True)
class NumberedPose:
    number: int
    instrument: str
    description: str
    positions: List[float]
    allow_contact: bool


def _build_pose_catalog() -> List[NumberedPose]:
    return [
        NumberedPose(
            number=number,
            instrument=instrument,
            description=description,
            positions=list(positions),
            allow_contact=allow_contact,
        )
        for number, (instrument, description, positions, allow_contact) in enumerate(
            POSE_DEFINITIONS,
            start=1,
        )
    ]


POSES = _build_pose_catalog()
POSES_BY_NUMBER = {pose.number: pose for pose in POSES}


class HandPoseSelector(Node):
    """Publie une pose de main et attend son retour articulaire."""

    def __init__(self, joint_state_topic: str) -> None:
        super().__init__("hand_pose_selector")
        self._publisher = self.create_publisher(
            OnlineMotionCommand,
            ONLINE_MOTION_TOPIC,
            10,
        )
        self._positions: Optional[Dict[str, float]] = None
        self._velocities: Optional[Dict[str, float]] = None
        self._command_id = 0
        self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            20,
        )
        self.get_logger().info(
            f"JointState de la main : {joint_state_topic}; "
            f"topic de commande : {ONLINE_MOTION_TOPIC}."
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        if (
            len(msg.name) != len(msg.position)
            or len(msg.name) != len(msg.velocity)
        ):
            self._positions = None
            self._velocities = None
            return

        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        missing = [
            name for name in RIGHT_HAND_JOINTS if name not in positions
        ]
        if missing:
            self._positions = None
            self._velocities = None
            return

        self._positions = {
            name: float(positions[name]) for name in RIGHT_HAND_JOINTS
        }
        self._velocities = {
            name: float(velocities[name]) for name in RIGHT_HAND_JOINTS
        }

    def has_hand_feedback(self) -> bool:
        return self._positions is not None and self._velocities is not None

    def move_to_pose(self, pose: NumberedPose) -> None:
        positions = self._positions
        velocities = self._velocities
        if positions is None or velocities is None:
            raise RuntimeError(
                "Les positions et vitesses articulaires de la main ne sont "
                "pas disponibles."
            )
        if self.count_subscribers(ONLINE_MOTION_TOPIC) == 0:
            raise RuntimeError(
                f"Aucun controleur n'est abonne a {ONLINE_MOTION_TOPIC}."
            )

        initial_error = sum(
            abs(positions[name] - target)
            for name, target in zip(RIGHT_HAND_JOINTS, pose.positions)
        )
        self._publish_target(pose.positions)
        progress_detected = False
        stall_since: Optional[float] = None
        deadline = time.monotonic() + FEEDBACK_TIMEOUT
        max_position_error = float("inf")
        max_velocity = float("inf")

        while rclpy.ok() and time.monotonic() < deadline:
            positions = self._positions
            velocities = self._velocities
            if positions is None or velocities is None:
                time.sleep(0.1)
                continue

            errors = [
                abs(positions[name] - target)
                for name, target in zip(RIGHT_HAND_JOINTS, pose.positions)
            ]
            max_position_error = max(errors)
            total_position_error = sum(errors)
            max_velocity = max(
                abs(velocities[name]) for name in RIGHT_HAND_JOINTS
            )

            if (
                max_position_error <= POSITION_TOLERANCE + COMPARISON_EPSILON
                and max_velocity <= VELOCITY_TOLERANCE + COMPARISON_EPSILON
            ):
                self.get_logger().info(
                    f"Pose #{pose.number} atteinte : {pose.instrument}, "
                    f"{pose.description}."
                )
                return

            if initial_error - total_position_error >= CONTACT_MIN_PROGRESS:
                progress_detected = True

            if (
                pose.allow_contact
                and progress_detected
                and max_velocity <= VELOCITY_TOLERANCE + COMPARISON_EPSILON
            ):
                now = time.monotonic()
                if stall_since is None:
                    stall_since = now
                elif now - stall_since >= CONTACT_STALL_SECONDS:
                    self.get_logger().info(
                        f"Contact stable détecté pour la pose #{pose.number} "
                        f"({pose.instrument}, {pose.description})."
                    )
                    return
            elif max_velocity > VELOCITY_TOLERANCE + COMPARISON_EPSILON:
                stall_since = None

            time.sleep(0.1)

        raise TimeoutError(
            f"La pose #{pose.number} n'a pas été atteinte avant le délai de "
            f"{FEEDBACK_TIMEOUT:.0f} s (erreur max={max_position_error:.3f}, "
            f"vitesse max={max_velocity:.3f})."
        )

    def _publish_target(self, positions: Sequence[float]) -> None:
        self._command_id += 1
        command = OnlineMotionCommand()
        command.type = OnlineMotionCommand.JOINT_TRAJECTORY
        command.target_arm_indices = [RIGHT_HAND_GROUP]

        point = TrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * len(positions)
        command.target_point = [point]
        command.planner = OnlineMotionCommand.PLANNER_RUCKIG
        command.max_velocity = [ArmConstraintParameters(values=[1.0])]
        command.max_acceleration = [ArmConstraintParameters(values=[0.8])]
        command.max_jerk = [ArmConstraintParameters(values=[5.0])]
        command.is_whole_body = False
        command.reference_frame = "world_frame"
        command.command_id = self._command_id
        command.need_interpolation = True
        command.is_urgent = False
        command.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(command)
        self.get_logger().info(
            f"Commande #{self._command_id} envoyée pour la pose de main."
        )


def _spin_until(predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.1)
    return False


def _confirm_pose(pose: NumberedPose) -> bool:
    print(
        f"\nPose #{pose.number} : {pose.instrument} — {pose.description}."
    )
    try:
        answer = input("Envoyer cette commande ? [o/n] : ").strip().lower()
    except EOFError:
        print("Aucune confirmation reçue ; commande annulée.")
        return False
    return answer in ("o", "oui")


def _display_pose_menu() -> None:
    print("\nPoses disponibles (main droite uniquement) :")
    for pose in POSES:
        print(
            f"  {pose.number}. {pose.instrument.capitalize()} — "
            f"{pose.description}"
        )
    print("  q. Quitter")


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Choisir une pose numérotée pour la main droite."
    )
    default_robot_id = os.environ.get("ROBOT_ID", "ROBOT_10")
    parser.add_argument("--robot-id", default=default_robot_id)
    parser.add_argument("--joint-state-topic", default="")
    options, _ = parser.parse_known_args(args=args)

    joint_state_topic = (
        options.joint_state_topic.strip()
        or f"/{options.robot_id}/mc/pr10_joint_states_topic"
    )
    if not joint_state_topic.startswith("/"):
        joint_state_topic = "/" + joint_state_topic

    print(
        "Ce script commande uniquement la main droite. Vérifiez que la zone "
        "de mouvement est dégagée."
    )
    rclpy.init(args=args)
    node = None
    executor = None
    spin_thread = None
    try:
        node = HandPoseSelector(joint_state_topic)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        print("Attente du retour articulaire de la main...")
        if not _spin_until(node.has_hand_feedback, FEEDBACK_TIMEOUT):
            raise TimeoutError(
                f"Aucun retour articulaire complet reçu en "
                f"{FEEDBACK_TIMEOUT:.0f} s sur {joint_state_topic}."
            )

        while rclpy.ok():
            _display_pose_menu()
            try:
                choice = input("Choisissez le numéro d'une pose : ").strip().lower()
            except EOFError:
                print("\nFin de saisie ; arrêt du sélecteur.")
                break

            if choice in ("q", "quitter"):
                break
            try:
                pose = POSES_BY_NUMBER[int(choice)]
            except (ValueError, KeyError):
                print("Choix invalide ; saisissez un numéro affiché ou q.")
                continue

            if not _confirm_pose(pose):
                continue
            node.move_to_pose(pose)
    except KeyboardInterrupt:
        print("\nArrêt demandé.")
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
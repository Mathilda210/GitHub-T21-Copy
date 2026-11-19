#!/usr/bin/env python3
"""Saisie interactive d'un endoscope et translations du bras droit.
Version demandé avec publication de TF pour rviz et commande de mouvement en cartésien."""

import argparse
import os
import time
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
ARM_FRAME = "right_arm_link_6"
TOOL_FRAME = "tool_tip"
TOOL_OFFSET_Z = 0.20
TRANSLATION_METERS = 0.10
JOINT_TOLERANCE = 0.10
CARTESIAN_TOLERANCE = 0.01
FEEDBACK_TIMEOUT = 30.0
MOTION_TIMEOUT = 60.0
TRANSLATION_AXES = {"x": 0, "y": 1, "z": 2}


class EndoscopeManipulation(Node):
    """Envoie les commandes et publie les repères utiles à RViz."""

    def __init__(self, joint_topic: str, tf_topic: str) -> None:
        super().__init__("endoscope_manipulation")
        self._motion_publisher = self.create_publisher(
            OnlineMotionCommand, ONLINE_MOTION_TOPIC, 10
        )
        self._joint_positions: Dict[str, float] = {}
        self._arm_pose: Optional[List[float]] = None
        self._arm_pose_received_at = 0.0
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
        self._publish_tool_transform()

    def _joint_state_cb(self, msg: JointState) -> None:
        self._joint_positions = dict(zip(msg.name, msg.position))

    def _tf_cb(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent != BASE_FRAME or child != ARM_FRAME:
                continue

            # La source robot est un topic dédié : on relaie cette TF sur /tf
            # pour que RViz voie la chaîne complète jusqu'à tool_tip.
            live_tf = TransformStamped()
            live_tf.header = transform.header
            live_tf.header.frame_id = parent
            live_tf.child_frame_id = child
            live_tf.transform = transform.transform
            self._tf_broadcaster.sendTransform(live_tf)

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

    def _publish_tool_transform(self) -> None:
        tool_tf = TransformStamped()
        tool_tf.header.stamp = self.get_clock().now().to_msg()
        tool_tf.header.frame_id = ARM_FRAME
        tool_tf.child_frame_id = TOOL_FRAME
        tool_tf.transform.translation.z = TOOL_OFFSET_Z
        tool_tf.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(tool_tf)
        self.get_logger().info(
            f"TF statique publiee : {ARM_FRAME} -> {TOOL_FRAME}, "
            f"translation locale z={TOOL_OFFSET_Z:.2f} m."
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
        command.reference_frame = BASE_FRAME
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
    executor: SingleThreadedExecutor,
    predicate: Callable[[], bool],
    timeout: float,
) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if predicate():
            return True
    return False


def _move_joints(
    node: EndoscopeManipulation,
    executor: SingleThreadedExecutor,
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

    if not _spin_until(executor, target_reached, FEEDBACK_TIMEOUT):
        raise TimeoutError(
            f"Retour articulaire non conforme apres {FEEDBACK_TIMEOUT:.0f} s "
            f"pour l'action : {description}."
        )
    print(f"Position confirmee : {description}.")
    return True


def _wait_for_arm_pose(
    node: EndoscopeManipulation,
    executor: SingleThreadedExecutor,
) -> Optional[List[float]]:
    if node._arm_pose is None and not _spin_until(
        executor, lambda: node._arm_pose is not None, FEEDBACK_TIMEOUT
    ):
        return None
    return list(node._arm_pose) if node._arm_pose is not None else None


def _move_cartesian(
    node: EndoscopeManipulation,
    executor: SingleThreadedExecutor,
    pose: Sequence[float],
) -> bool:
    if not _confirm(
        "Confirmer la translation du poignet droit de 10 cm "
        "dans le repere waist_link_0"
    ):
        return False

    start_received_at = node._arm_pose_received_at
    node.publish_cartesian_target(pose)
    deadline = time.monotonic() + MOTION_TIMEOUT
    while rclpy.ok() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        current = node._arm_pose
        if (
            current is None
            or node._arm_pose_received_at <= start_received_at
            or node._arm_pose_received_at < time.monotonic() - 1.0
        ):
            continue
        error = max(abs(current[index] - pose[index]) for index in range(3))
        if error <= CARTESIAN_TOLERANCE:
            print("Translation confirmee par la TF du bras.")
            return True
    raise TimeoutError(
        "La nouvelle pose du bras n'a pas confirme la cible cartesienne "
        f"en {MOTION_TIMEOUT:.0f} s. Aucun retour automatique n'est envoye."
    )


def _translation_menu(
    node: EndoscopeManipulation,
    executor: SingleThreadedExecutor,
) -> None:
    while rclpy.ok():
        print(
            "\nActions possibles :\n"
            "  x / y / z : translation +10 cm dans waist_link_0\n"
            "  h         : revenir au home du bras en gardant la saisie\n"
            "  r         : revenir au home puis relacher l'endoscope\n"
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
                executor,
                "Revenir au home du bras droit",
                RIGHT_ARM_INDEX,
                RIGHT_ARM_JOINTS,
                HOME_RIGHT_ARM,
            ):
                continue
            continue
        if choice in ("r", "relacher", "release"):
            returned_home = _move_joints(
                node,
                executor,
                "Revenir au home du bras droit en gardant l'endoscope",
                RIGHT_ARM_INDEX,
                RIGHT_ARM_JOINTS,
                HOME_RIGHT_ARM,
            )
            if not returned_home:
                continue
            opened = _move_joints(
                node,
                executor,
                "Relacher l'endoscope en ouvrant la main droite",
                RIGHT_HAND_INDEX,
                RIGHT_HAND_JOINTS,
                HAND_OPEN,
            )
            if opened:
                return
            continue
        if choice not in TRANSLATION_AXES:
            print("Choix invalide ; saisir x, y, z, h, r ou q.")
            continue

        current_pose = _wait_for_arm_pose(node, executor)
        if current_pose is None:
            raise TimeoutError(
                f"TF {BASE_FRAME} -> {ARM_FRAME} introuvable sur le topic robot."
            )
        target = list(current_pose)
        target[TRANSLATION_AXES[choice]] += TRANSLATION_METERS
        _move_cartesian(node, executor, target)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Saisir un endoscope et le deplacer avec validation utilisateur."
    )
    robot_id = os.environ.get("ROBOT_ID", "ROBOT_10")
    parser.add_argument("--robot-id", default=robot_id)
    parser.add_argument("--joint-state-topic", default="")
    parser.add_argument("--arm-tf-topic", default="")
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
    if not _confirm("Demarrer la sequence home puis saisie de l'endoscope"):
        print("Sequence annulee.")
        return

    rclpy.init(args=args)
    node = EndoscopeManipulation(joint_topic, tf_topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        if not _move_joints(
            node,
            executor,
            "Placer le bras droit en position home",
            RIGHT_ARM_INDEX,
            RIGHT_ARM_JOINTS,
            HOME_RIGHT_ARM,
        ):
            return
        if not _move_joints(
            node,
            executor,
            "Ouvrir la main droite avant la saisie",
            RIGHT_HAND_INDEX,
            RIGHT_HAND_JOINTS,
            HAND_OPEN,
        ):
            return

        for description, pose in ENDOSCOPE_GRASP:
            if not _move_joints(
                node,
                executor,
                f"{description.capitalize()} (pose mesuree)",
                RIGHT_HAND_INDEX,
                RIGHT_HAND_JOINTS,
                pose,
            ):
                print("Saisie interrompue par l'utilisateur.")
                return

        print(
            f"\nLe repere {TOOL_FRAME} est publie a {TOOL_OFFSET_Z:.2f} m "
            f"sur l'axe z local de {ARM_FRAME}."
        )
        print(
            f"En attente de la TF {BASE_FRAME} -> {ARM_FRAME}; "
            "verifiez que le topic TF du robot est disponible."
        )
        if _wait_for_arm_pose(node, executor) is None:
            raise TimeoutError(
                f"TF {BASE_FRAME} -> {ARM_FRAME} non recue en "
                f"{FEEDBACK_TIMEOUT:.0f} s."
            )
        _translation_menu(node, executor)
    except KeyboardInterrupt:
        print("\nArret demande ; aucun mouvement supplementaire ne sera envoye.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
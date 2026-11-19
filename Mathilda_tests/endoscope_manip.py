#!/usr/bin/env python3
"""Saisie interactive d'un endoscope et mouvements du bras droit."""

import argparse
import math
import os
import threading
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
ARM_COMMAND_FRAME = "waist_link_4"
ARM_FRAME = "right_arm_link_6"
TOOL_FRAME = "tool_tip"
TOOL_OFFSET_Z = 0.20
TF_REPUBLISH_PERIOD = 0.1
TRANSLATION_METERS = 0.10
ROTATION_DEGREES = 10.0
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
        self._arm_transforms: Dict[str, TransformStamped] = {}
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
        self.create_timer(TF_REPUBLISH_PERIOD, self._republish_arm_transform)
        self._publish_tool_transform()

    def _joint_state_cb(self, msg: JointState) -> None:
        self._joint_positions = dict(zip(msg.name, msg.position))

    def _tf_cb(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent not in (BASE_FRAME, ARM_COMMAND_FRAME) or child != ARM_FRAME:
                continue

            live_tf = TransformStamped()
            live_tf.header = transform.header
            live_tf.header.frame_id = parent
            live_tf.child_frame_id = child
            live_tf.transform = transform.transform
            self._arm_transforms[parent] = live_tf

            if parent != ARM_COMMAND_FRAME:
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

    def _republish_arm_transform(self) -> None:
        transform = self._arm_transforms.get(ARM_COMMAND_FRAME)
        if transform is None:
            transform = self._arm_transforms.get(BASE_FRAME)
        if transform is None:
            return

        live_tf = TransformStamped()
        live_tf.header.stamp = self.get_clock().now().to_msg()
        live_tf.header.frame_id = transform.header.frame_id
        live_tf.child_frame_id = transform.child_frame_id
        live_tf.transform = transform.transform
        self._tf_broadcaster.sendTransform(live_tf)

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


def _move_cartesian(
    node: EndoscopeManipulation,
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
    node: EndoscopeManipulation,
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
                "Revenir au home du bras droit en gardant l'endoscope",
                RIGHT_ARM_INDEX,
                RIGHT_ARM_JOINTS,
                HOME_RIGHT_ARM,
            )
            if not returned_home:
                continue
            opened = _move_joints(
                node,
                "Relacher l'endoscope en ouvrant la main droite",
                RIGHT_HAND_INDEX,
                RIGHT_HAND_JOINTS,
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
        if translation_axis is not None:
            tool_axis_in_command_frame = _rotate_vector_by_quaternion(
                TRANSLATION_AXES[translation_axis],
                current_pose[3:7],
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
            target[3:7] = _multiply_quaternions(current_pose[3:7], delta_rotation)
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
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
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

        print(
            f"\nLe repere {TOOL_FRAME} est publie a {TOOL_OFFSET_Z:.2f} m "
            f"sur l'axe z local de {ARM_FRAME}."
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
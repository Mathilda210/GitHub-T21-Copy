#!/usr/bin/env python3

"""Pick an object from the table, lift it 20 cm, put it back, and release it.
Tests pas concluants pour l'instant... """

import argparse
import math
import os
import time
from typing import Dict, List, Optional

import rclpy
from px_mc_msgs.msg import ArmConstraintParameters, OnlineMotionCommand, TrajectoryPoint
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


ONLINE_MOTION_TOPIC = "/external/online_motion_command"
RIGHT_ARM_GROUP = 1
RIGHT_HAND_GROUP = 5
RIGHT_ARM_JOINTS = [f"right_arm_joint_{index}" for index in range(7)]
RIGHT_HAND_JOINTS = [
    f"right_{finger}_joint_{index}"
    for finger in ("index", "middle", "ring", "pinky", "thumb")
    for index in range(3)
]
RIGHT_ARM_TF_PARENT = "waist_link_4"
RIGHT_ARM_TF_CHILD = "right_arm_link_6"

# Measured right-arm poses from Mathilda_tests/poses.txt.
HAND_OVER_TABLE = [-0.478, -0.228, 0.220, -0.814, 1.142, 0.152, -0.036]
HAND_ON_TABLE = [-0.605, -0.213, 0.269, -0.417, 1.251, 0.053, -0.028]

# Hand targets follow the open/closed poses in Finished_scripts/suman_pick_on_box.py.
OPEN_RIGHT_HAND = [
    0.791, 0.0, 0.004,
    0.694, 0.028, 0.015,
    0.698, 0.0, 0.0,
    0.692, 0.006, 0.006,
    1.349, 0.059, 1.079,
]
CLOSED_RIGHT_HAND = [
    1.3, 0.0, 0.0,
    1.3, 0.0, 0.0,
    1.3, 0.0, 0.0,
    1.3, 0.0, 0.0,
    1.5697, 0.6, 0.95,
]

LIFT_METERS = 0.20
JOINT_TIMEOUT_SECONDS = 45.0
CARTESIAN_TIMEOUT_SECONDS = 60.0
TF_REFRESH_TIMEOUT_SECONDS = 5.0
POSITION_TOLERANCE = 0.05
HAND_CLOSE_TOLERANCE = 0.1
VELOCITY_TOLERANCE = 0.01
HAND_CLOSE_MIN_PROGRESS = 0.05
HAND_CLOSE_STALL_SECONDS = 1.5
CARTESIAN_POSITION_TOLERANCE = 0.01
CARTESIAN_ORIENTATION_TOLERANCE = math.radians(1.0)
COMPARISON_EPSILON = 1e-9


def _confirm(prompt: str) -> bool:
    try:
        answer = input(f"{prompt} [o/n] : ").strip().lower()
    except EOFError:
        print("Aucune confirmation recue ; operation annulee.")
        return False
    return answer in ("o", "oui", "y", "yes")


class OrthoToolPick(Node):
    """Send measured joint poses and a feedback-based Cartesian lift."""

    def __init__(self, joint_state_topic: str, arm_tf_topic: str) -> None:
        super().__init__("pick_ortho_tools")
        self.publisher = self.create_publisher(
            OnlineMotionCommand, ONLINE_MOTION_TOPIC, 10
        )
        self.arm_tf_topic = arm_tf_topic
        self.joint_names = RIGHT_ARM_JOINTS + RIGHT_HAND_JOINTS
        self.positions: Dict[str, float] = {}
        self.velocities: Dict[str, float] = {}
        self.arm_pose: Optional[List[float]] = None
        self.arm_pose_received_at = 0.0
        self.command_id = 0
        self.create_subscription(
            JointState, joint_state_topic, self._joint_state_callback, 20
        )
        self.create_subscription(
            TFMessage,
            arm_tf_topic,
            self._arm_tf_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
            return
        self.positions = dict(zip(msg.name, map(float, msg.position)))
        self.velocities = dict(zip(msg.name, map(float, msg.velocity)))

    def _arm_tf_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if (
                transform.header.frame_id == RIGHT_ARM_TF_PARENT
                and transform.child_frame_id == RIGHT_ARM_TF_CHILD
            ):
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.arm_pose = [
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                    float(rotation.w),
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                ]
                self.arm_pose_received_at = time.monotonic()

    def publish_joint_target(
        self, targets: Dict[int, List[float]], groups: List[int]
    ) -> None:
        msg = OnlineMotionCommand()
        msg.type = OnlineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = groups
        msg.target_point = []
        for group in groups:
            point = TrajectoryPoint()
            point.positions = targets[group]
            point.velocities = [0.0] * len(targets[group])
            msg.target_point.append(point)
        msg.planner = OnlineMotionCommand.PLANNER_RUCKIG
        msg.max_velocity = []
        msg.max_acceleration = []
        msg.max_jerk = []
        for group in groups:
            if group == RIGHT_ARM_GROUP:
                msg.max_velocity.append(ArmConstraintParameters(values=[0.5]))
                msg.max_acceleration.append(ArmConstraintParameters(values=[0.5]))
                msg.max_jerk.append(ArmConstraintParameters(values=[5.0]))
            else:
                msg.max_velocity.append(ArmConstraintParameters(values=[1.0]))
                msg.max_acceleration.append(ArmConstraintParameters(values=[0.8]))
                msg.max_jerk.append(ArmConstraintParameters(values=[5.0]))
        msg.is_whole_body = False
        msg.reference_frame = "world_frame"
        self._publish(msg)

    def publish_cartesian_target(self, target: List[float]) -> None:
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
        msg.pos_tolerance = 0.0
        msg.vel_tolerance = 0.0
        self._publish(msg)

    def _publish(self, msg: OnlineMotionCommand) -> None:
        self.command_id += 1
        msg.command_id = self.command_id
        msg.need_interpolation = True
        msg.is_urgent = False
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Commande {self.command_id} publiee sur {ONLINE_MOTION_TOPIC}."
        )


def _spin_until_ready(node: OrthoToolPick, timeout: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (
            all(name in node.positions for name in node.joint_names)
            and node.arm_pose is not None
        ):
            return True
    return False


def _wait_for_joint_target(
    node: OrthoToolPick,
    targets: Dict[int, List[float]],
    groups: List[int],
    hand_close: bool = False,
) -> bool:
    group_joints = {
        RIGHT_ARM_GROUP: RIGHT_ARM_JOINTS,
        RIGHT_HAND_GROUP: RIGHT_HAND_JOINTS,
    }
    initial_error: Optional[float] = None
    progress_detected = False
    stall_since: Optional[float] = None
    deadline = time.monotonic() + JOINT_TIMEOUT_SECONDS
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        names = [name for group in groups for name in group_joints[group]]
        if not all(name in node.positions and name in node.velocities for name in names):
            continue
        errors = [
            abs(node.positions[name] - target)
            for group in groups
            for name, target in zip(group_joints[group], targets[group])
        ]
        max_velocity = max(abs(node.velocities[name]) for name in names)
        total_error = sum(errors)
        if hand_close:
            if initial_error is None:
                initial_error = total_error
            elif initial_error - total_error >= HAND_CLOSE_MIN_PROGRESS:
                progress_detected = True
            if max(errors) <= HAND_CLOSE_TOLERANCE and max_velocity <= VELOCITY_TOLERANCE:
                return True
            if progress_detected and max_velocity <= VELOCITY_TOLERANCE:
                if stall_since is None:
                    stall_since = time.monotonic()
                elif time.monotonic() - stall_since >= HAND_CLOSE_STALL_SECONDS:
                    node.get_logger().info(
                        "La main s'est immobilisee apres avoir progresse ; "
                        "contact considere comme une prise."
                    )
                    return True
            elif max_velocity > VELOCITY_TOLERANCE:
                stall_since = None
        elif (
            max(errors) <= POSITION_TOLERANCE
            and max_velocity <= VELOCITY_TOLERANCE
        ):
            return True
    return False


def _quaternion_error(first: List[float], second: List[float]) -> float:
    first_norm = math.hypot(*first)
    second_norm = math.hypot(*second)
    if first_norm <= COMPARISON_EPSILON or second_norm <= COMPARISON_EPSILON:
        return math.inf
    dot = abs(
        sum(
            (a / first_norm) * (b / second_norm)
            for a, b in zip(first, second)
        )
    )
    return 2.0 * math.acos(min(1.0, dot))


def _wait_for_cartesian_target(
    node: OrthoToolPick, target: List[float], sent_at: float
) -> bool:
    deadline = time.monotonic() + CARTESIAN_TIMEOUT_SECONDS
    settled_since: Optional[float] = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        pose = node.arm_pose
        if pose is None or node.arm_pose_received_at <= sent_at:
            continue
        position_error = math.sqrt(
            sum((pose[index] - target[index]) ** 2 for index in range(3))
        )
        orientation_error = _quaternion_error(pose[3:], target[3:])
        if (
            position_error <= CARTESIAN_POSITION_TOLERANCE
            and orientation_error <= CARTESIAN_ORIENTATION_TOLERANCE
        ):
            if settled_since is None:
                settled_since = time.monotonic()
            elif time.monotonic() - settled_since >= 0.5:
                return True
        else:
            settled_since = None
    return False


def _move_joints(
    node: OrthoToolPick,
    targets: Dict[int, List[float]],
    groups: List[int],
    label: str,
    hand_close: bool = False,
) -> bool:
    node.get_logger().info(f"Mouvement : {label}.")
    sent_at = time.monotonic()
    node.publish_joint_target(targets, groups)
    if not _wait_for_joint_target(node, targets, groups, hand_close):
        return False
    if RIGHT_ARM_GROUP not in groups:
        return True

    deadline = time.monotonic() + TF_REFRESH_TIMEOUT_SECONDS
    while rclpy.ok() and time.monotonic() < deadline:
        if node.arm_pose_received_at > sent_at:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


def _move_cartesian(
    node: OrthoToolPick, target: List[float], label: str
) -> bool:
    node.get_logger().info(
        f"Mouvement cartesien : {label} (z={target[2]:.3f} m)."
    )
    sent_at = time.monotonic()
    node.publish_cartesian_target(target)
    return _wait_for_cartesian_target(node, target, sent_at)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Prendre un objet, le lever de 20 cm, le reposer et le relacher."
    )
    robot_id = os.environ.get("ROBOT_ID", "ROBOT_10")
    parser.add_argument("--robot-id", default=robot_id)
    parser.add_argument("--joint-state-topic", default="")
    parser.add_argument("--arm-tf-topic", default="")
    parsed, ros_args = parser.parse_known_args(args=args)
    joint_state_topic = (
        parsed.joint_state_topic.strip()
        or f"/{parsed.robot_id}/mc/pr10_joint_states_topic"
    )
    arm_tf_topic = (
        parsed.arm_tf_topic.strip()
        or f"/{parsed.robot_id}/mc/pr10_arm_tf_topic"
    )
    if not joint_state_topic.startswith("/"):
        joint_state_topic = "/" + joint_state_topic
    if not arm_tf_topic.startswith("/"):
        arm_tf_topic = "/" + arm_tf_topic

    print(
        "\nSequence prevue : main droite au-dessus de la table, approche, "
        "prise, levee cartesienne de 20 cm, retour a la pose de prise, "
        "puis relachement.\n"
        "La zone doit etre degagee. Le robot ne bougera pas sans confirmation."
    )
    if not _confirm("Confirmer le lancement de toute la sequence"):
        print("Sequence annulee.")
        return

    rclpy.init(args=ros_args)
    node = OrthoToolPick(joint_state_topic, arm_tf_topic)
    try:
        print("Attente des retours articulaires et TF avant tout mouvement...")
        if not _spin_until_ready(node):
            raise RuntimeError(
                "Retours articulaires complets ou TF "
                f"{RIGHT_ARM_TF_PARENT}->{RIGHT_ARM_TF_CHILD} indisponibles."
            )
        if node.count_subscribers(ONLINE_MOTION_TOPIC) == 0:
            raise RuntimeError(
                f"Aucun abonne detecte sur {ONLINE_MOTION_TOPIC}; "
                "aucune commande envoyee."
            )

        if not _move_joints(
            node, {RIGHT_HAND_GROUP: OPEN_RIGHT_HAND}, [RIGHT_HAND_GROUP],
            "ouverture de la main droite",
        ):
            raise RuntimeError("Delai depasse pendant l'ouverture de la main.")
        if not _move_joints(
            node,
            {RIGHT_ARM_GROUP: HAND_OVER_TABLE},
            [RIGHT_ARM_GROUP],
            "position de la main au-dessus de la table",
        ):
            raise RuntimeError("Delai depasse pour la position au-dessus de table.")
        if not _move_joints(
            node,
            {RIGHT_ARM_GROUP: HAND_ON_TABLE},
            [RIGHT_ARM_GROUP],
            "approche de l'objet sur la table",
        ):
            raise RuntimeError("Delai depasse pendant l'approche de l'objet.")
        if not _move_joints(
            node,
            {RIGHT_HAND_GROUP: CLOSED_RIGHT_HAND},
            [RIGHT_HAND_GROUP],
            "fermeture de la main sur l'objet",
            hand_close=True,
        ):
            raise RuntimeError(
                "Prise non confirmee par le retour articulaire; "
                "aucune levee ni relachement automatique."
            )

        if not _confirm("Confirmer la levee verticale de l'objet de 20 cm"):
            print("Levee annulee ; l'objet reste dans la main, sans autre mouvement.")
            return
        table_pose = list(node.arm_pose) if node.arm_pose is not None else None
        if table_pose is None:
            raise RuntimeError("Pose TF absente apres la prise.")
        lifted_pose = list(table_pose)
        lifted_pose[2] += LIFT_METERS
        if not _move_cartesian(node, lifted_pose, "levee verticale de 20 cm"):
            raise RuntimeError(
                "La cible de levee n'a pas ete atteinte; "
                "aucun retour ni relachement automatique."
            )
        if not _move_cartesian(node, table_pose, "depose sur la table"):
            raise RuntimeError(
                "La pose de depose n'a pas ete atteinte; "
                "l'objet reste dans la main."
            )

        if not _confirm("Confirmer l'ouverture de la main et le relachement"):
            print("Relachement annule ; l'objet reste dans la main.")
            return
        if not _move_joints(
            node,
            {RIGHT_HAND_GROUP: OPEN_RIGHT_HAND},
            [RIGHT_HAND_GROUP],
            "relachement de l'objet",
        ):
            raise RuntimeError("Delai depasse pendant le relachement.")
        print("Sequence terminee.")
    except KeyboardInterrupt:
        print("\nInterruption utilisateur. Aucun mouvement de recuperation automatique.")
    except RuntimeError as error:
        node.get_logger().error(str(error))
        print(
            "Sequence arretee. Le robot reste dans sa position actuelle ; "
            "verifiez la situation avant toute action."
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
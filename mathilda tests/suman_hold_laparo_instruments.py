#!/usr/bin/env python3

"""Move the right hand through a selected laparoscopic instrument grasp."""

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


RIGHT_HAND_GROUP = 5
RIGHT_HAND_JOINTS = [
    f"right_{finger}_joint_{index}"
    for finger in ("index", "middle", "ring", "pinky", "thumb")
    for index in range(3)
]

POSE_PAUSE_SECONDS = 2.0
POSITION_TOLERANCE = 0.1
HAND_CLOSE_MIN_PROGRESS = 0.05
HAND_CLOSE_STALL_SECONDS = 1.5
VELOCITY_TOLERANCE = 0.01
COMPARISON_EPSILON = 1e-9

INSTRUMENTS: Dict[str, List[Tuple[str, List[float]]]] = {
    "endoscope": [
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
    ],
    "ciseaux": [
        (
            "prise initiale des ciseaux",
            [
                1.086, 0.0, 0.004,
                1.175, 0.019, 0.012,
                0.022, 0.0, 0.003,
                1.082, 0.016, 0.010,
                1.564, 0.637, 0.321,
            ],
        ),
        (
            "prise resserree des ciseaux",
            [
                1.210, 0.004, 0.004,
                1.172, 0.129, 0.071,
                0.022, 0.0, 0.003,
                1.114, 0.137, 0.078,
                1.564, 0.637, 0.321,
            ],
        ),
    ],
    "petite pince": [
        (
            "prise initiale de la petite pince",
            [
                1.318, 0.001, 0.0,
                1.336, 0.027, 0.015,
                1.344, 0.004, 0.002,
                0.004, 0.006, 0.005,
                1.371, 0.013, 0.026,
            ],
        ),
        (
            "doigts resserres",
            [
                1.393, 0.729, 0.536,
                1.391, 0.626, 0.436,
                1.392, 0.511, 0.341,
                0.004, 0.006, 0.005,
                1.371, 0.013, 0.026,
            ],
        ),
        (
            "pouce resserre",
            [
                1.392, 0.940, 0.756,
                1.391, 0.662, 0.467,
                1.392, 0.511, 0.341,
                0.026, 0.0, 0.005,
                1.371, 0.789, 1.324,
            ],
        ),
    ],
}

INSTRUMENT_CHOICES = {
    "1": "endoscope",
    "2": "ciseaux",
    "3": "petite pince",
    "endoscope": "endoscope",
    "ciseaux": "ciseaux",
    "petite pince": "petite pince",
}


class SumanLaparoHold(Node):
    """Apply one measured grasp sequence and keep its final pose."""

    def __init__(
        self,
        joint_state_topic: str,
        instrument: str,
    ) -> None:
        super().__init__("suman_hold_laparo_instruments")
        self.publisher = self.create_publisher(
            OnlineMotionCommand,
            "/external/online_motion_command",
            10,
        )
        self.steps = INSTRUMENTS[instrument]
        self.instrument = instrument
        self.step_index = 0
        self.phase = "send_step"
        self.next_action_time = time.monotonic()
        self.waiting_target: Optional[List[float]] = None
        self._latest_positions: Optional[Dict[str, float]] = None
        self._latest_velocities: Optional[Dict[str, float]] = None
        self._last_feedback_warning = 0.0
        self._hand_close_initial_error: Optional[float] = None
        self._hand_close_stall_since: Optional[float] = None
        self._hand_close_progress_detected = False
        self._hand_close_contact_detected = False
        self.cmd_id = 0

        self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            20,
        )
        self.timer = self.create_timer(0.1, self._advance)
        self.get_logger().info(
            f"Joint-state feedback: {joint_state_topic}; "
            f"selected instrument: {instrument}."
        )

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
                self.get_logger().info(
                    f"{self.instrument.capitalize()} is held in the final pose. "
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
            f"Published command {self.cmd_id} for the right hand."
        )


def _choose_instrument() -> Optional[str]:
    print("Quel instrument voulez-vous saisir ?")
    print("  1. Endoscope")
    print("  2. Ciseaux")
    print("  3. Petite pince")
    while True:
        try:
            choice = input("Votre choix (1-3, ou q pour quitter) : ").strip().lower()
        except EOFError:
            print("\nAucune saisie disponible ; lancement annule.")
            return None

        if choice in ("q", "quitter"):
            return None
        instrument = INSTRUMENT_CHOICES.get(choice)
        if instrument is not None:
            return instrument
        print("Choix invalide. Entrez 1, 2, 3 ou q.")


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Saisir un instrument laparo avec une pose mesuree."
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
    instrument = _choose_instrument()
    if instrument is None:
        print("Lancement annule.")
        return

    print(
        f"\nInstrument choisi : {instrument}.\n"
        "Consignes : placez l'instrument dans la main droite, degagez la zone "
        "autour de la main, puis appuyez sur Entree pour commencer. "
        "Appuyez sur Ctrl+C pour arreter."
    )
    try:
        if input("Pret ? [Entree/q] : ").strip().lower() in ("q", "quitter"):
            print("Lancement annule.")
            return
    except EOFError:
        print("\nAucune confirmation recue ; lancement annule.")
        return

    joint_state_topic = (
        parsed_args.joint_state_topic.strip()
        or f"/{parsed_args.robot_id}/mc/pr10_joint_states_topic"
    )
    if not joint_state_topic.startswith("/"):
        joint_state_topic = "/" + joint_state_topic

    rclpy.init(args=args)
    node = SumanLaparoHold(joint_state_topic, instrument)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
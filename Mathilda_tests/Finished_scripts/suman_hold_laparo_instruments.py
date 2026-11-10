
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
HOME_HAND_POSE = [0.0] * len(RIGHT_HAND_JOINTS)

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
            "enfilage des ciseaux entre pouce et index",
            [
                0.003, 0.0, 0.0,
                1.221, 0.017, 0.007,
                0.016, 0.003, 0.001,
                0.0, 0.0, 0.0,
                1.623, 0.757, 0.0,
            ],
        ),
        (
            "doigts resserres sur les ciseaux",
            [
                0.003, 0.0, 0.0,
                1.193, 0.201, 0.113,
                0.016, 0.003, 0.001,
                0.0, 0.0, 0.0,
                1.623, 0.978, 0.0,
            ],
        ),
        (
            "index en appui",
            [
                1.037, 0.247, 0.146,
                1.265, 0.103, 0.057,
                0.016, 0.003, 0.001,
                0.0, 0.0, 0.0,
                1.623, 0.924, 0.0,
            ],
        ),
    ],
    "petite pince": [
        (
            "pouce replie",
            [
                0.028, 0.0, 0.0,
                0.003, 0.002, 0.002,
                0.014, 0.0, 0.003,
                0.004, 0.002, 0.005,
                1.675, 1.080, 1.830,
            ],
        ),
        (
            "annulaire et petit doigt en appui",
            [
                0.028, 0.0, 0.0,
                0.003, 0.002, 0.002,
                1.392, 1.208, 1.075,
                1.292, 1.161, 1.016,
                1.675, 1.086, 1.830,
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
        self.home_reset = False
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

    def start_instrument(self, instrument: str) -> None:
        self.instrument = instrument
        self.steps = INSTRUMENTS[instrument]
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


def _confirm_instrument_ready(instrument: str) -> bool:
    print(
        f"\nInstrument choisi : {instrument}.\n"
        "Placez l'instrument dans la main droite et degagez la zone autour "
        "de la main."
    )
    try:
        return input("Pret ? [Entree/q] : ").strip().lower() not in ("q", "quitter")
    except EOFError:
        print("\nAucune confirmation recue ; lancement annule.")
        return False


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

    if not _confirm_instrument_ready(instrument):
        print("Lancement annule.")
        return

    print("Appuyez sur Ctrl+C pour arreter.")
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
            if node.phase != "done":
                continue

            if node.home_reset:
                instrument = _choose_instrument()
                if instrument is None:
                    break
                if not _confirm_instrument_ready(instrument):
                    print("Arret du test.")
                    break
                node.start_instrument(instrument)
                continue

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
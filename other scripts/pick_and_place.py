#!/usr/bin/env python3

"""Publish a sequence of joint positions for a pick-and-place task.

The node sends trajectory commands to the controller and monitors ``JointState``
feedback to advance through targets, optionally replay the sequence in reverse,
or pause interactively at selected points in preparation mode.
"""



# ----------------- IMPORTS ----------------- #
import argparse
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Custom imports 
from px_mc_msgs.msg import OnlineMotionCommand, TrajectoryPoint, ArmConstraintParameters
from sensor_msgs.msg import JointState




# ----------------- HELPER FUNCTIONS ----------------- #
def _hand_joints(side: str) -> List[str]:
    """Return the names of the 15 hand joints for the requested side.

    The order is index, middle, ring, pinky, and thumb, with three consecutive
    joints per finger. It must match the order of hand position values.

    Args:
        side: Side prefix used in joint names, such as ``"left"`` or ``"right"``.

    Returns:
        Joint names ordered by finger and then by joint index.
    """
    fingers = ("index", "middle", "ring", "pinky", "thumb")
    return [
        f"{side}_{finger}_joint_{index}"
        for finger in fingers
        for index in range(3)
    ]



# ----------------- CONSTANTS ----------------- #
TARGET_JOINT_GROUPS = {
    # Map OnlineMotionCommand group indices to joint names. The joint order must
    # match the order of position values in each trajectory target.
    0: [f"left_arm_joint_{index}" for index in range(7)],
    1: [f"right_arm_joint_{index}" for index in range(7)],
    2: [
        "right_front_leg_joint",
        "left_front_leg_joint",
        "waist_joint_1",
        "waist_joint_2",
        "waist_joint_3",
        "waist_joint_4",
    ],
    3: ["neck_joint", "head_joint_0"],
    4: _hand_joints("left"),
    5: _hand_joints("right"),
}

# Feedback thresholds required before a target is considered reached.
INITIAL_POSITION_TOLERANCE = 1.5
INITIAL_VELOCITY_TOLERANCE = 0.01
# Delay between completing forward playback and starting reverse playback.
REVERSE_WAIT_SECONDS = 5.0
# Numerical margin used when comparing timing and joint thresholds.
COMPARISON_EPSILON = 1e-9

# Right-hand preparation pose: five fingers with three joints each, in the
# order returned by _hand_joints("right").
PREPARE_RIGHT_HAND_POSITION = [
    0.5, 0.0, 0.0,
    0.5076, 0.0, 0.0,
    0.5008, 0.0, 0.0,
    0.5, 0.0, 0.0,
    1.5697, 0.0, 0.95,
]



# ----------------- NODE CLASS ----------------- #
class OnlineTrajectoryPublisher(Node):
    """ROS 2 node that publishes and orchestrates a joint-target sequence."""

    def __init__(self, loop_count=0, reverse=False, prepare=False, joint_state_topic=""):
        """Initialize targets, state feedback subscription, and command timer.

        Args:
            loop_count: Number of sequences to run; zero means run indefinitely.
            reverse: Replay each sequence in reverse after forward playback.
            prepare: Pause at grasp and placement targets and keep the right hand
                at its predefined preparation pose.
            joint_state_topic: ROS 2 topic providing measured joint positions and
                velocities.
        """
        super().__init__("online_trajectory_publisher")

        self.publisher = self.create_publisher(
            OnlineMotionCommand,
            "/external/online_motion_command",
            10,
        )

        self.arm_indices = [0, 1, 2, 3, 4, 5]

        # Each target maps a group index (see TARGET_JOINT_GROUPS) to joint
        # positions ordered to match that group's joint names.
        self.target_positions_a = {
            # arms: 7 joints
            0: [-0.0003, 0.4003, 0.0000, -1.7697, 0.0003, 0.0002, 0.0001],
            1: [-0.0003, -0.3997, 0.0003, -1.7697, 0.0003, 0.0003, 0.0003],
            # waist: 6 joints (including 2 front legs)
            2: [-0.0001, 0.0000, -0.5999, 1.1999, -0.5999, 0.0000],
            # head: 2 joints
            3: [0.0003, 0.0002],
            # Hands: five fingers with three joints each.
            4: [0.0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            5: [0.5, 0.0000, 0.0000, 0.5076, 0.0000, 0.0000, 0.5008, 0.0000, 0.0000, 0.5, 0.0, 0.0, 1.5697, 0.0000, 0.95],
        }

        # B: pre-position before approaching the object.
        self.target_positions_b = {
            0: [0.4911, 1.0769, 0.7641, -2.3363, -0.2115, 0.0098, -0.0001],
            1: [-0.0813, -0.8349, -0.2657, -2.3639, 0.6618, -0.0730, 0.0719],
            2: [0.0000, 0.0000, -0.7999, 1.5999, -0.1001, 0.0000],
            3: [-0.2, 0.003],
            4: [0.0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            5: [0.5, 0.0000, 0.0000, 0.5076, 0.0000, 0.0000, 0.5008, 0.0000, 0.0000, 0.5, 0.0, 0.0, 1.5697, 0.0000, 0.95],
        }

        # C: initial lift / approach position.
        self.target_positions_c = {
            0: [0.4911, 1.0769, 0.7641, -2.3363, -0.2115, 0.0098, -0.0001],
            1: [-0.9319, -1.0165, 0.2214, -1.6128, 0.4242, 0.2533, 0.1853],
            2: [0.0000, 0.0000, -0.7998, 1.5998, -0.1000, 0.0000],
            3: [-0.2, 0.003],
            4: [0.0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            5: [0.5, 0.0000, 0.0000, 0.5076, 0.0000, 0.0000, 0.5008, 0.0000, 0.0000, 0.5, 0.0, 0.0, 1.5697, 0.0000, 0.95],
        }

        # D: grasp position; preparation mode pauses here.
        self.target_positions_d = {
            0: [0.4911, 1.0769, 0.7641, -2.3363, -0.2115, 0.0098, -0.0001],
            1: [-1.1299, -0.6598, 0.1690, -1.3132, 0.8901, 0.6133, -0.0076],
            2: [0.0000, 0.0000, -0.7998, 1.5998, -0.1000, 0.0000],
            3: [-0.2, 0.003],
            4: [0.0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            5: [0.5, 0.0000, 0.0000, 0.5076, 0.0000, 0.0000, 0.5008, 0.0000, 0.0000, 0.5, 0.0, 0.0, 1.5697, 0.0000, 0.95],
        }

        # G: close the hand around the object.
        self.target_positions_g = {
            0: [0.4911, 1.0769, 0.7641, -2.3363, -0.2115, 0.0098, -0.0001],
            1: [-1.1299, -0.6598, 0.1690, -1.3132, 0.8901, 0.6133, -0.0076],
            2: [0.0000, 0.0000, -0.7998, 1.5998, -0.1000, 0.0000],
            3: [-0.2, 0.003],
            4: [0.0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            5: [1.3, 0.0, 0.0000, 1.3, 0.0, 0.0000, 1.3, 0.0, 0.0000, 1.3, 0.0, 0.0, 1.5697, 0.6, 0.95],
        }

        # E: lift the object after grasping it.
        self.target_positions_e = {
            0: [0.4911, 1.0769, 0.7641, -2.3363, -0.2115, 0.0098, -0.0001],
            1: [-1.1814, -0.6258, 0.3378, -1.8128, 0.6513, 0.6998, 0.2709],
            2: [0.0000, 0.0000, -0.7998, 1.5998, -0.1000, 0.0000],
            3: [-0.2, 0.003],
            4: [0.0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            5: [1.3, 0.0, 0.0000, 1.3, 0.0, 0.0000, 1.3, 0.0, 0.0000, 1.3, 0.0, 0.0, 1.5697, 0.6, 0.95],
        }

        # F: placement position; preparation mode pauses here.
        self.target_positions_f = {
            0: [0.4911, 1.0769, 0.7641, -2.3363, -0.2115, 0.0098, -0.0001],
            1: [-1.1794, -0.0167, 0.3428, -1.4834, 1.0275, 0.8408, 0.5877],
            2: [0.0000, 0.0000, -0.7998, 1.5998, -0.1000, 0.0000],
            3: [-0.2, 0.003],
            4: [0.0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            5: [1.3, 0.0, 0.0000, 1.3, 0.0, 0.0000, 1.3, 0.0, 0.0000, 1.3, 0.0, 0.0, 1.5697, 0.6, 0.95],
        }

        # H: open the hand to release the object.
        self.target_positions_h = {
            0: [0.4911, 1.0769, 0.7641, -2.3363, -0.2115, 0.0098, -0.0001],
            1: [-1.1794, -0.0167, 0.3428, -1.4834, 1.0275, 0.8408, 0.5877],
            2: [0.0000, 0.0000, -0.7998, 1.5998, -0.1000, 0.0000],
            3: [-0.2, 0.003],
            4: [0.0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            5: [0.7, 0.0000, 0.0000, 0.7, 0.0000, 0.0000, 0.7, 0.0000, 0.0000, 0.7, 0.0, 0.0, 1.5697, 0.0000, 0.45],
        }

        # I: return toward the previous position after placement.
        self.target_positions_i = {
            0: [0.4911, 1.0769, 0.7641, -2.3363, -0.2115, 0.0098, -0.0001],
            1: [-1.1814, -0.6258, 0.3378, -1.8128, 0.6513, 0.6998, 0.2709],
            2: [0.0000, 0.0000, -0.7998, 1.5998, -0.1000, 0.0000],
            3: [-0.2, 0.003],
            4: [0.0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000],
            5: [0.7, 0.0000, 0.0000, 0.7, 0.0000, 0.0000, 0.7, 0.0000, 0.0000, 0.7, 0.0, 0.0, 1.5697, 0.0000, 0.45],
        }

        self.targets = [
            # Forward playback order; the letters refer to the target dictionaries
            # defined above.
            self.target_positions_a,
            self.target_positions_b,
            self.target_positions_c,
            self.target_positions_d,
            self.target_positions_g,
            self.target_positions_e,
            self.target_positions_f,
            self.target_positions_h,
            self.target_positions_i,
        ]

        self.prepare_mode = prepare
        # Preparation mode runs one forward sequence and disables reverse playback.
        self.loop_count = 1 if prepare else loop_count
        self.reverse_enabled = False if prepare else reverse

        # Map the D and F target indices to the prompts shown during pauses.
        self.prepare_pause_prompts = {
            next(
                index for index, target in enumerate(self.targets)
                if target is self.target_positions_d
            ): 'Please place the object at the grasping position',
            next(
                index for index, target in enumerate(self.targets)
                if target is self.target_positions_f
            ): 'Please place the object at the placement position',
        }

        self.prepare_wait_index: Optional[int] = None
        self.completed_loops = 0
        self.finished = False

        # All joints whose feedback is required to verify target completion.
        self._joint_names = [
            joint_name
            for arm_index in self.arm_indices
            for joint_name in TARGET_JOINT_GROUPS[arm_index]
        ]

        self._latest_positions: Optional[Dict[str, float]] = None
        self._latest_velocities: Optional[Dict[str, float]] = None
        self._last_feedback_warning = 0.0

        self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            20,
        )

        # Initialize the phase machine. Delay the first command to allow the node
        # and ROS feedback stream to start.
        self.send_interval = 2.2
        self.phase = 'initial'
        self.initial_sent = False
        self.forward_index = 1
        self.reverse_index = len(self.targets) - 1
        self.next_action_time = time.monotonic() + self.send_interval

        self.cmd_id = 0
        self.timer = self.create_timer(0.1, self.publish_command)

        self.get_logger().info(
            f'Joint-state feedback: {joint_state_topic}; reverse: '
            f'{"enabled" if self.reverse_enabled else "disabled"}; prepare: '
            f'{"enabled" if self.prepare_mode else "disabled"}'
        )

    def _joint_state_callback(self, msg: JointState):
        """Store joint feedback when the message contains all required data.

        Invalid or incomplete messages are ignored and reported through a
        rate-limited warning.

        Args:
            msg: Incoming ROS joint-state message.
        """
        if len(msg.name) != len(msg.position):
            self._warn_feedback(
                'Waiting for valid JointState: name/position sizes differ.'
            )
            return

        if len(msg.velocity) != len(msg.name):
            self._warn_feedback(
                'Waiting for JointState velocities: velocity array is missing or incomplete.'
            )
            return

        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))

        missing = [name for name in self._joint_names if name not in positions]
        if missing:
            self._warn_feedback(
                'Waiting for complete JointState; missing: ' + ', '.join(missing)
            )
            return

        self._latest_positions = {
            name: float(positions[name]) for name in self._joint_names
        }
        self._latest_velocities = {
            name: float(velocities[name]) for name in self._joint_names
        }


    def _warn_feedback(self, message: str):
        """Log a feedback warning no more than once every three seconds.

        Args:
            message: Warning text to send to the node logger.
        """
        now = time.monotonic()
        if now - self._last_feedback_warning >= 3.0:
            self.get_logger().warning(message)
            self._last_feedback_warning = now


    def _effective_target(
        self,
        target_index: int,
    ) -> Dict[int, List[float]]:
        """Return the target to publish, applying the preparation hand override.

        In preparation mode, group 5 (the right hand) uses
        PREPARE_RIGHT_HAND_POSITION; all other groups retain the trajectory
        target's positions.

        Args:
            target_index: Zero-based index into ``self.targets``.

        Returns:
            Joint positions keyed by target group index.
        """
        target = self.targets[target_index]

        if not self.prepare_mode:
            return target

        return {
            arm_index: (
                PREPARE_RIGHT_HAND_POSITION
                if arm_index == 5
                else target[arm_index]
            )
            for arm_index in self.arm_indices
        }


    def _target_reached(self, target_index: int) -> Tuple[bool, float, float]:
        """Check whether all targeted joints are close enough and nearly still.

        Args:
            target_index: Zero-based index into ``self.targets``.

        Returns:
            A tuple of: whether the target is reached, maximum absolute position
            error, and maximum absolute joint velocity.
        """
        if (
            self._latest_positions is None
            or self._latest_velocities is None
        ):
            self._warn_feedback(
                'Waiting for complete joint position and velocity feedback.'
            )
            return False, float('inf'), float('inf')

        target = self._effective_target(target_index)

        max_position_error = 0.0
        max_velocity = 0.0

        for arm_index in self.arm_indices:
            for joint_name, target_position in zip(
                TARGET_JOINT_GROUPS[arm_index],
                target[arm_index],
            ):
                max_position_error = max(
                    max_position_error,
                    abs(
                        self._latest_positions[joint_name]
                        - target_position
                    ),
                )

                max_velocity = max(
                    max_velocity,
                    abs(self._latest_velocities[joint_name]),
                )

        reached = (
            max_position_error
            <= INITIAL_POSITION_TOLERANCE + COMPARISON_EPSILON
            and max_velocity
            <= INITIAL_VELOCITY_TOLERANCE + COMPARISON_EPSILON
        )

        if not reached:
            self._warn_feedback(
                f'Waiting at target point {target_index + 1}: '
                f'max position error {max_position_error:.4f} '
                f'(limit {INITIAL_POSITION_TOLERANCE:.2f}), '
                f'max velocity {max_velocity:.4f} '
                f'(limit {INITIAL_VELOCITY_TOLERANCE:.2f}).'
            )

        return reached, max_position_error, max_velocity


    def publish_command(self):
        """Advance the phase machine and publish commands when needed.

        The ROS timer calls this method periodically. Depending on the current
        phase, it sends the initial target, advances the sequence, handles
        preparation pauses, or starts reverse playback.
        """
        now = time.monotonic()

        if self.phase == 'initial':
            if not self.initial_sent:
                if now + COMPARISON_EPSILON < self.next_action_time:
                    return

                self._publish_target(0)
                self.initial_sent = True

                self.get_logger().info(
                    'Initial point sent; waiting for position and velocity tolerances.'
                )
                return

            reached, position_error, velocity = self._target_reached(0)
            if not reached:
                return

            self.get_logger().info(
                'Initial point reached: max position error '
                f'{position_error:.4f}, max velocity {velocity:.4f}.'
            )

            if len(self.targets) == 1:
                self._finish_forward(now)
            else:
                self.phase = 'forward'
                self.forward_index = 1
                self.next_action_time = now + self.send_interval

            return
        
        if self.phase == 'prepare_wait':
            reached, position_error, velocity = self._target_reached(
                self.prepare_wait_index
            )

            if not reached:
                return

            prompt = self.prepare_pause_prompts[self.prepare_wait_index]

            self.get_logger().info(
                f'Target point {self.prepare_wait_index + 1} reached: '
                f'max position error {position_error:.4f}, '
                f'max velocity {velocity:.4f}.'
            )

            try:
                input(f'{prompt}, press Enter to continue > ')
            except EOFError:
                self.get_logger().warning(
                    'Input closed; continuing prepare mode.'
                )

            self.prepare_wait_index = None
            self.phase = 'forward'
            self.next_action_time = time.monotonic() + self.send_interval

            return

        if now + COMPARISON_EPSILON < self.next_action_time:
            return

        if self.phase == 'forward':
            published_index = self.forward_index

            self._publish_target(published_index)
            self.forward_index += 1

            if self.forward_index >= len(self.targets):
                self._finish_forward(now)

            elif (
                self.prepare_mode
                and published_index in self.prepare_pause_prompts
            ):
                self.prepare_wait_index = published_index
                self.phase = 'prepare_wait'

                self.get_logger().info(
                    f'Target point {published_index + 1} sent; '
                    'waiting until reached.'
                )

            else:
                self.next_action_time = now + self.send_interval

            return

        if self.phase == 'reverse_wait':
            self.phase = 'reverse'
            self.reverse_index = len(self.targets) - 1

        if self.phase == 'reverse':
            self._publish_target(self.reverse_index)
            self.reverse_index -= 1

            if self.reverse_index < 0:
                self._complete_loop(now)
            else:
                self.next_action_time = now + self.send_interval


    def _finish_forward(self, now: float):
        """Finish forward playback by looping, waiting for reverse, or stopping.

        Args:
            now: Current monotonic time, used to schedule the next phase.
        """
        if self.prepare_mode:
            self._complete_loop(now)

        elif self.reverse_enabled:
            self.phase = 'reverse_wait'
            self.next_action_time = now + REVERSE_WAIT_SECONDS

            self.get_logger().info(
                f'Forward playback completed; '
                f'waiting {REVERSE_WAIT_SECONDS:.1f}s '
                'before reverse playback.'
            )

        else:
            self._complete_loop(now)


    def _complete_loop(self, now: float):
        """Count a completed loop and stop or schedule another forward sequence.

        Args:
            now: Current monotonic time, used to schedule the next sequence.
        """
        self.completed_loops += 1

        self.get_logger().info(
            f'Completed loop {self.completed_loops}'
            + (
                ' (forward + reverse).'
                if self.reverse_enabled
                else ' (forward only).'
            )
        )

        if (
            self.loop_count > 0
            and self.completed_loops >= self.loop_count
        ):
            self.finished = True
            self.timer.cancel()
            return

        self.phase = 'forward'
        self.forward_index = 0
        self.next_action_time = now + self.send_interval


    def _publish_target(self, target_index: int):
        """Build and publish the ROS command for one trajectory target.

        The command contains one trajectory point per target group, zero
        terminal velocities, and motion constraints for each group.

        Args:
            target_index: Zero-based index into ``self.targets``.
        """
        current_targets = self._effective_target(target_index)
        current_target_index = target_index + 1

        msg = OnlineMotionCommand()
        msg.type = OnlineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = self.arm_indices

        msg.target_arm_indices = self.arm_indices

        target_points = []

        arm_names = {
            0: "Left Arm",
            1: "Right Arm",
            2: "Waist Arm",
            3: "Head Arm",
            4: "Left Hand",
            5: "Right Hand",
        }

        for arm_idx in self.arm_indices:
            point = TrajectoryPoint()
            point.positions = current_targets[arm_idx]

            # Specify zero terminal velocity for this target point.
            point.velocities = [0.0] * len(point.positions)

            target_points.append(point)

            self.get_logger().info(
                f"{arm_names[arm_idx]} target positions: "
                f"{[f'{p:.3f}' for p in point.positions]}"
            )

        msg.target_point = target_points
        msg.planner = OnlineMotionCommand.PLANNER_RUCKIG

        # Configure velocity, acceleration, and jerk limits for each group.
        msg.max_velocity = []
        msg.max_acceleration = []
        msg.max_jerk = []

        for arm_idx in self.arm_indices:
            # Left or right arm (seven joints).
            if arm_idx in [0, 1]:
                # A single constraint value applies to all joints in the group.
                msg.max_velocity.append(
                    ArmConstraintParameters(values=[0.5])
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(values=[0.5])
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(values=[5.0])
                )

            # Waist group (six joints).
            elif arm_idx == 2:
                # These constraints specify one value per joint.
                msg.max_velocity.append(
                    ArmConstraintParameters(
                        values=[0.3, 0.3, 0.3, 0.3, 0.13, 0.14]
                    )
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(
                        values=[0.4, 0.5, 0.1, 0.4, 0.5, 0.3]
                    )
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(
                        values=[4.0, 5.0, 3.0, 4.0, 5.0, 3.0]
                    )
                )

            # Head group.
            elif arm_idx == 3:
                msg.max_velocity.append(
                    ArmConstraintParameters(values=[0.3])
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(values=[0.5])
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(values=[5.0])
                )

            # Left or right DH15 hand.
            else:
                msg.max_velocity.append(
                    ArmConstraintParameters(values=[1.0])
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(values=[0.8])
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(values=[5.0])
                )

        msg.is_whole_body = False
        msg.reference_frame = "world_frame"

        self.cmd_id += 1
        msg.command_id = self.cmd_id
        msg.need_interpolation = True
        msg.is_urgent = False
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Published command {self.cmd_id} "
            f"for target point {current_target_index} "
            f"to arms {self.arm_indices}"
        )


def main(args=None):
    """Parse command-line options, configure ROS 2, and run the node.

    Args:
        args: Optional argument list; ``None`` reads arguments from the process.
    """
    parser = argparse.ArgumentParser(
        description='Publish online target points at a fixed interval.'
    )

    parser.add_argument(
        '--loop-count',
        '--loops',
        type=int,
        default=0,
        help=(
            'Number of loops to execute; 0 means infinite. '
            'With --reverse, a forward and reverse sequence '
            'counts as one loop (default: 0).'
        ),
    )

    parser.add_argument(
        '--reverse',
        action='store_true',
        help=(
            'Wait 5 seconds after forward playback, then replay '
            'all points in reverse order.'
        ),
    )

    parser.add_argument(
        '--prepare',
        action='store_true',
        help=(
            'Run the A-to-I sequence once, pause at D and F '
            'waiting for Enter, and keep the right hand '
            'at the predefined preparation position.'
        ),
    )

    # ROBOT_ID selects the robot without requiring the full topic to be specified.
    default_robot_id = os.environ.get(
        'ROBOT_ID',
        'ROBOT_10',
    )

    parser.add_argument(
        '--robot-id',
        default=default_robot_id,
        help=(
            'Robot ID used to build the default JointState topic '
            f'(default: {default_robot_id}).'
        ),
    )

    parser.add_argument(
        '--joint-state-topic',
        default='',
        help='JointState feedback topic; overrides --robot-id.',
    )

    parsed_args, _ = parser.parse_known_args(args=args)

    if parsed_args.loop_count < 0:
        parser.error('--loop-count must be >= 0')

    # Prefer an explicit topic; otherwise construct it from the robot ID.
    joint_state_topic = (
        parsed_args.joint_state_topic.strip()
        or f'/{parsed_args.robot_id}/mc/pr10_joint_states_topic'
    )

    if not joint_state_topic.startswith('/'):
        joint_state_topic = '/' + joint_state_topic

    rclpy.init(args=args)

    node = OnlineTrajectoryPublisher(
        loop_count=parsed_args.loop_count,
        reverse=parsed_args.reverse,
        prepare=parsed_args.prepare,
        joint_state_topic=joint_state_topic,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

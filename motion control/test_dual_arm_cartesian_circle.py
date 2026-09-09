#!/usr/bin/env python3

"""
Dual-arm Cartesian circle demo in the zy plane (fixed x, fixed orientation).

Pose order: [x, y, z, qw, qx, qy, qz]

How to run
----------
1. Start arm_fsm (main_node) and motor driver. Ensure these are available:
   - /external/pose_demo
   - /external/online_motion_command
2. Put the system in MOVE state (e.g. HOLD first, then control_command command=3).
3. Source the ROS workspace, then:
   python3 test_dual_arm_cartesian_circle.py
4. Script sequence:
   a) Call /external/pose_demo (go_home); abort unless response.success is true.
   b) Publish dual-arm Cartesian start poses (right arm mirrored about xz).
   c) Wait --dwell-before-circle seconds (default 10).
   d) Loop forever: publish circle waypoints; Ctrl+C to stop.

Examples::
   python3 test_dual_arm_cartesian_circle.py
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import List, Sequence

import rclpy
from rclpy.node import Node

from px_mc_msgs.msg import (
    ArmConstraintParameters,
    OnlineMotionCommand,
    TrajectoryPoint,
)
from px_mc_msgs.srv import PoseDemo

# arm_fsm arm indices and Cartesian pose dimension
LEFT_ARM_IDX = 0
RIGHT_ARM_IDX = 1
POSE_SIZE = 7

# Left-arm circle start pose (theta=0 on the zy-plane circle);
# right arm is derived by mirror_pose_xz
DEFAULT_LEFT_START = [
    0.222486,
    0.318404,
    -0.004878,
    0.693407,
    0.140458,
    -0.692657,
    -0.140306,
]


def _fmt_pose(pose: Sequence[float]) -> str:
    return "[" + ", ".join(f"{v:.6f}" for v in pose) + "]"


def mirror_pose_xz(pose: Sequence[float]) -> List[float]:
    """Mirror pose about the xz plane (palm symmetry rule used in arm_fsm scripts)."""

    if POSE_SIZE:
        raise ValueError(f"pose must have {POSE_SIZE} elements")

    x, y, z, qw, qx, qy, qz = (float(v) for v in pose)

    # Position: flip Y
    # Orientation: (qw, qx, qy, qz) -> (qw, -qx, qy, -qz)
    return [x, -y, z, qw, -qx, qy, -qz]


def circle_pose_left(
    theta: float,
    x0: float,
    y0: float,
    z0: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
    radius: float,
) -> List[float]:
    """Left-arm waypoint on zy-plane circle; fixed x and quaternion."""

    # Center chosen so theta=0 lands on (y0, z0)
    # y(0)=y0, z(0er_y = y0 - radius
    center_z = z0

    y = center_y + radius * math.cos(theta)
    z = center_z + radius * math.sin(theta)

    return [x0, y, z, qw, qx, qy, qz]


def circle_pose_right(
    theta: float,
    x0: float,
    y0: float,
    z0: float,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
    radius: float,
) -> List[float]:
    return mirror_pose_xz(
        circle_pose_left(
            theta,
            x0,
            y0,
            z0,
            qw,
            qx,
            qy,
            qz,
adius,
        )
    )


class DualArmCirclePublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("dual_arm_cartesian_circle")

        self._args = args
        self._command_id = int(args.command_id)
        self._left_start = list(args.left_start_pose)

        # Cartesian motion ingress (non-whole-body dual-arm commands)
        self._motion_pub = self.create_publisher(
            OnlineMotionCommand,
            "/external/online_motion_command",
            10,
        )

        self._pose_demo_client = self.create_client(
            PoseDemo,
            "/external/pose_demo",
        )

    def call_go_home_pose_demo(self) -> bool:
        """Blocking call to /external/pose_demo; must return success before Cartesian motion."""

        if not self._pose_demo_client.wait_for_service(
            timeout_sec=float(
                self._args.pose_demo_wait_service_timeout
            )
        ):
            self.get_logger().error(
                "Service /external/pose_demo is not available"
            )
            return False

        request = PoseDemo.Request()
        request.stamp = self.get_clock().now().to_msg()
        request.command_id = int(self._args.pose_demo_command_id)
        request.demo_name = str(self._args.pose_demo_name)
        request.speed_scale = float(
            self._args.pose_demo_speed_scale
        )
        request.repeat_count = int(
            self._args.pose_demo_repeat_count
        )
        request.timeout_sec = float(
            self._args.pose_demo_timeout_sec
        )

        self.get_logger().info(
            "Calling /external/pose_demo: "
            f"demo_name={request.demo_name!r}, "
            f"command_id={request.command_id}, "
            f"speed_scale={request.speed_scale}, "
            f"repeat_count={request.repeat_count}, "
            f"timeout_sec={request.timeout_sec}"
        )

        future = self._pose_demo_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=float(
                self._args.pose_demo_timeout_sec
            ) + 30.0,
        )

        if future.result() is None:
            self.get_logger().error(
                f"PoseDemo service call failed: "
                f"{future.exception()}"
            )
            return False

        response = future.result()

        self.get_logger().info(
            f"PoseDemo result: success={response.success}, "
            f"result_code={response.result_code}, "
            f"message={response.message!r}"
        )

        return bool(response.success)

    @property
    def x0(self) -> float:
        return self._left_start[0]

    @property
    def y0(self) -> float:
        return self._left_start[1]

    @property
    def z0(self) -> float:
        return self._left_start[2]

    @property
    def orientation(
        self,
    ) -> tuple[float, float, float, float]:
        return (
            self._left_start[3],
            self._left_start[4],
            self._left_start[5],
            self._left_start[6],
        )

    def publish_dual_cartesian(
        self,
        left_pose: Sequence[float],
        right_pose: Sequence[float],
    ) -> int:
        """One OnlineMotionCommand for both arms."""

        msg = OnlineMotionCommand()
        msg.type = OnlineMotionCommand.CARTESIAN_TRAJECTORY
        msg.target_arm_indices = [
            LEFT_ARM_IDX,
            RIGHT_ARM_IDX,
        ]
        msg.target_point = []

        # target_point[i] aligns with target_arm_indices[i]
        for pose in (left_pose, right_pose):
            point = TrajectoryPoint()
            point.positions = [float(v) for v in pose]
            point.velocities = [0.0] * POSE_SIZE
            point.accelerations = [0.0] * POSE_SIZE
            msg.target_point.append(point)

        msg.max_velocity = [
            ArmConstraintParameters(
                values=[float(self._args.max_velocity)]
            ),
            ArmConstraintParameters(
                values=[float(self._args.max_velocity)]
            ),
        ]

        msg.max_acceleration = [
            ArmConstraintParameters(
                values=[float(self._args.max_acceleration)]
            ),
            ArmConstraintParameters(
                values=[float(self._args.max_acceleration)]
            ),
        ]

        msg.max_jerk = [
            ArmConstraintParameters(
                values=[float(self._args.max_jerk)]
            ),
            ArmConstraintParameters(
                values=[float(self._args.max_jerk)]
            ),
        ]


        msg.planner = OnlineMotionCommand.PLANNER_RUCKIG
        msg.reference_frame = self._args.reference_frame
        msg.command_id = self._command_id
        msg.need_interpolation = True  # Ruckig interpolates between successive targets at 1 kHz
        msg.is_whole_body = False
        msg.is_urgent = False
        msg.stamp = self.get_clock().now().to_msg()
        msg.pos_tolerance = float(self._args.pos_tolerance)
        msg.vel_tolerance = float(self._args.vel_tolerance)

        self._motion_pub.publish(msg)

        cmd_id = self._command_id
        self._command_id += 1

        return cmd_id

    def run(self) -> None:
        # Phase 1: homing via pose demo (must succeed before any Cartesian command)
        if not self._args.skip_pose_demo:
            if not self.call_go_home_pose_demo():
                raise RuntimeError(
                    "PoseDemo go_home failed or returned success=false"
                )
        else:
            self.get_logger().warning(
                "Skipping /external/pose_demo (--skip-pose-demo)"
            )

        qw, qx, qy, qz = self.orientation
        right_start = mirror_pose_xz(self._left_start)

        # Phase 2: move both arms to circle start poses (theta = 0)
        cmd_id = self.publish_dual_cartesian(
            self._left_start,
            right_start,
        )

        self.get_logger().info(
            f"Published start poses command_id={cmd_id}, "
            f"is_whole_body=false, targets=[0, 1]"
        )
        self.get_logger().info(
            f"  left:  {_fmt_pose(self._left_start)}"
        )
        self.get_logger().info(
            f"  right: {_fmt_pose(right_start)}"
        )

        # Phase 3: hold at start before tracing the circle
        dwell = float(self._args.dwell_before_circle)

        self.get_logger().info(
            f"Waiting {dwell:.1f}s before circle motion..."
        )

        time.sleep(dwell)

        n_points = int(self._args.points_per_rev)

        if n_points < 3:
            raise ValueError(
                "--points-per-rev must be >= 3"
            )

        radius = float(self._args.radius)
        interval = float(self._args.waypoint_interval)

        rev = 0
        wp_in_rev = 0

        self.get_logger().info(
            f"Starting infinite circle: radius={radius:.3f} m, "
            f"points_per_rev={n_points}, "
            f"waypoint_interval={interval:.3f} s"
        )

        # Phase 4: publish waypoints along the circle until interrupted
        try:
            while rclpy.ok():

                if wp_in_rev >= n_points:
                    rev += 1
                    wp_in_rev = 0
                    self.get_logger().info(
                        f"Completed revolution {rev}"
                    )

                # Uniform sampling in theta;
                # one OnlineMotionCommand per waypoint
                theta = (
                    2.0 * math.pi * wp_in_rev
                ) / n_points

                left_pose = circle_pose_left(
                    theta,
                    self.x0,
                    self.y0,
                    self.z0,
                    qw,
                    qx,
                    qy,
                    qz,
                    radius,
                )

                right_pose = circle_pose_right(
                    theta,
                    self.x0,
                    self.y0,
                    self.z0,
                    qw,
                    qx,
                    qy,
                    qz,
                    radius,
                )

                cmd_id = self.publish_dual_cartesian(
                    left_pose,
                    right_pose,
                )

                self.get_logger().info(
                    f"Published circle waypoint "
                    f"command_id={cmd_id} "
                    f"rev={rev} "
                    f"idx={wp_in_rev}/{n_points} "
                    f"theta={theta:.4f} rad"
                )

                self.get_logger().info(
                    f"  left:  {_fmt_pose(left_pose)}"
                )

                self.get_logger().info(
                    f"  right: {_fmt_pose(right_pose)}"
                )

                wp_in_rev += 1

                time.sleep(interval)

        except KeyboardInterrupt:
            self.get_logger().info(
                "Interrupted by user, stopping circle publisher"
            )


def _parse_pose7(
    values: Sequence[float],
    arg_name: str,
) -> Listif len(values) != POSE_SIZE:
        raise ValueError(
            f"{arg_name} requires {POSE_SIZE} numbers: "
            "x y z qw qx qy qz"
        )

    return [float(v) for v in values]


def parse_args() -> argparse.Namespace:

    parser = argparse.ArgumentParser(
        description=(
            "Publish dual-arm Cartesian circle motion via "
            "/external/online_motion_command. "
            "Pose order: x y z qw qx qy qz."
        )
    )

    parser.add_argument(
        "--left-start-pose",
        nargs=POSE_SIZE,
        type=float,
        default=DEFAULT_LEFT_START,
        metavar=("X", "Y", "Z", "QW", "QX", "QY", "QZ"),
        help="Left arm circle start pose on the zy plane.",
    )

    parser.add_argument(
        "--radius",
        type=float,
        default=0.1,
        help="Circle radius in meters.",
    )

    parser.add_argument(
        "--dwell-before-circle",
        type=float,
        default=10.0,
        help="Seconds to wait at start pose before circle motion.",
    )

    parser.add_argument(
        "--points-per-rev",
        type=int,
        default=300,
        help="Number of waypoints per full revolution.",
    )

    parser.add_argument(
        "--waypoint-interval",
        type=float,
        default=0.02,
        help="Seconds between successive circle waypoints.",
    )

    parser.add_argument(
        "--pose-demo-name",
        default="go_home",
        help="demo_name for /external/pose_demo before circle motion.",
    )

    parser.add_argument(
        "--pose-demo-command-id",
        type=int,
        default=42,
        help="command_id for /external/pose_demo.",
    )

    parser.add_argument(
        "--pose-demo-speed-scale",
        type=float,
        default=0.8,
        help="speed_scale for /external/pose_demo.",
    )

    parser.add_argument(
        "--pose-demo-repeat-count",
        type=int,
        default=1,
        help="repeat_count for /external/pose_demo.",
    )

    parser.add_argument(
        "--pose-demo-timeout-sec",
        type=float,
        default=120.0,
        help="timeout_sec for /external/pose_demo.",
    )

    parser.add_argument(
        "--pose-demo-wait-service-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for /external/pose_demo service.",
    )

    parser.add_argument(
        "--skip-pose-demo",
        action="store_true",
        help="Skip go_home PoseDemo call (debug only).",
    )

    parser.add_argument(
        "--reference-frame",
        default="world_frame"
    )

    parser.add_argument(
        "--command-id",
        type=int,
        default=4000
    )

    parser.add_argument(
        "--max-velocity",
        type=float,
        default=0.3
    )

    parser.add_argument(
        "--max-acceleration",
        type=float,
        default=0.6
    )

    parser.add_argument(
        "--max-jerk",
        type=float,
        default=5.0
    )

    parser.add_argument(
        "--pos-tolerance",
        type=float,
        default=0.0
    )

    parser.add_argument(
        "--vel-tolerance",
        type=float,
        default=0.0
    )

    args = parser.parse_args()

    args.left_start_pose = _parse_pose7(
        args.left_start_pose,
        "--left-start-pose"
    )

    if args.radius <= 0.0:
        parser.error("--radius must be > 0")

    if args.dwell_before_circle < 0.0:
        parser.error(
            "--dwell-before-circle must be >= 0"
        )

    if args.points_per_rev < 3:
        parser.error(
            "--points-per-rev must be >= 3"
        )

    if args.waypoint_interval <= 0.0:
        parser.error(
            "--waypoint-interval must be > 0"
        )

    if args.max_velocity <= 0.0:
        parser.error(
            "--max-velocity must be > 0"
        )

    if args.max_acceleration <= 0.0:
        parser.error(
            "--max-acceleration must be > 0"
        )

    if args.max_jerk <= 0.0:
        parser.error(
            "--max-jerk must be > 0"
        )

    if args.pose_demo_speed_scale <= 0.0:
        parser.error(
            "--pose-demo-speed-scale must be > 0"
        )

    if args.pose_demo_repeat_count < 1:
        parser.error(
            "--pose-demo-repeat-count must be >= 1"
        )

    if args.pose_demo_timeout_sec <= 0.0:
        parser.error(
            "--pose-demo-timeout-sec must be > 0"
        )

    return args


def main() -> int:
    args = parse_args()

    rclpy.init()

    node = DualArmCirclePublisher(args)

    try:
        # Brief delay so publishers / service clients are ready
        time.sleep(0.5)
        node.run()

    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
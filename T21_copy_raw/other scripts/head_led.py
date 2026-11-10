#!/usr/bin/env python3

import argparse
import sys
import time
from typing import List

import rclpy
from rclpy.node import Node

from px_mc_msgs.msg import HeadLightControl


MODE_MAP = {
    "off": HeadLightControl.MODE_ALWAYS_OFF,
    "on": HeadLightControl.MODE_ALWAYS_ON,
    "breathing": HeadLightControl.MODE_BREATHING,
}

DEFAULT_SEQUENCE = ["off", "on", "breathing"]


class HeadLightSdoSequenceTester(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("head_light_sdo_sequence_tester")

        self._args = args

        self._publisher = self.create_publisher(
            HeadLightControl,
            "/external/head_light_control",
            10,
        )

    def wait_for_subscriber(self) -> bool:
        if self._args.wait_subscriber_timeout <= 0.0:
            return True

        deadline = (
            time.monotonic()
            + self._args.wait_subscriber_timeout
        )

        while rclpy.ok() and time.monotonic() < deadline:
            if self._publisher.get_subscription_count() > 0:
                return True

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            "No subscriber matched for "
            "/external/head_light_control "
            f"within "
            f"{self._args.wait_subscriber_timeout:.1f}s. "
            "Make sure arm_fsm main_node is running "
            "and sourced from the same workspace."
        )

        return False

    def build_message(
        self,
        mode: str,
        command_id: int
    ) -> HeadLightControl:

        msg = HeadLightControl()

        msg.stamp = self.get_clock().now().to_msg()
        msg.command_id = command_id
        msg.mode = MODE_MAP[mode]

        if mode == "off":
            msg.has_brightness = False
            msg.brightness = 0.0

            msg.has_period_sec = False
            msg.period_sec = 0.0

            return msg

        msg.has_brightness = (
            self._args.brightness is not None
        )

        msg.brightness = (
            float(self._args.brightness)
            if msg.has_brightness
            else 0.0
        )

        msg.has_period_sec = (
            mode == "breathing"
            and self._args.period_sec is not None
        )

        msg.period_sec = (
            float(self._args.period_sec)
            if msg.has_period_sec
            else 0.0
        )

        return msg

    def publish_sequence(self) -> None:
        if not self.wait_for_subscriber():
            raise RuntimeError(
                "No /external/head_light_control "
                "subscriber matched"
            )

        command_id = self._args.command_id

        for cycle_index in range(self._args.repeat):
            for step_index, mode in enumerate(
                self._args.sequence
            ):
                msg = self.build_message(
                    mode,
                    command_id
                )

                self._publisher.publish(msg)

                self.get_logger().info(
                    "Published head light SDO test step: "
                    f"cycle={cycle_index + 1}/"
                    f"{self._args.repeat}, "
                    f"step={step_index + 1}/"
                    f"{len(self._args.sequence)}, "
                    f"command_id={msg.command_id}, "
                    f"mode={mode}, "
                    f"has_brightness={msg.has_brightness}, "
                    f"brightness={msg.brightness:.3f}, "
                    f"has_period_sec={msg.has_period_sec}, "
                    f"period_sec={msg.period_sec:.3f}"
                )

                command_id += 1

                rclpy.spin_once(
                    self,
                    timeout_sec=0.05
                )

                is_last_step = (
                    cycle_index + 1 == self._args.repeat
                    and step_index + 1
                    == len(self._args.sequence)
                )

                if not is_last_step:
                    time.sleep(self._args.interval)


def _positive_int(value: str) -> int:
    parsed = int(value)

    if parsed <= 0:
        raise argparse.ArgumentTypeError(
            "value must be > 0"
        )

    return parsed


def _non_negative_float(value: str) -> float:
    parsed = float(value)

    if parsed < 0.0:
        raise argparse.ArgumentTypeError(
            "value must be >= 0"
        )

    return parsed


def _parse_sequence(values: List[str]) -> Listsequence = []

    for value in values:
        sequence.extend(
            item.strip()
            for item in value.split(",")
            if item.strip()
        )

    invalid_modes = [
        mode
        for mode in sequence
        if mode not in MODE_MAP
    ]

    if invalid_modes:
        valid_modes = ", ".join(
            sorted(MODE_MAP.keys())
        )

        raise argparse.ArgumentTypeError(
            f"invalid mode(s): "
            f"{', '.join(invalid_modes)}. "
            f"Valid modes: {valid_modes}"
        )

    if not sequence:
        raise argparse.ArgumentTypeError(
            "sequence must contain at least one mode"
        )

    return sequence


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Publish a head light SDO control "
            "test sequence through "
            "/external/head_light_control."
        )
    )

    parser.add_argument(
        "--sequence",
        nargs="+",
        default=DEFAULT_SEQUENCE,
        help=(
            "Head light mode sequence. "
            "Accepts spaces or comma-separated values. "
            "Valid modes: off,on,breathing. "
            "Default: off on breathing off."
        ),
    )

    parser.add_argument(
        "--brightness",
        type=float,
        default=0.5,
        help=(
            "Brightness used for on/breathing modes. "
            "Use --no-brightness to omit it."
        ),
    )

    parser.add_argument(
        "--no-brightness",
        action="store_true",
        help=(
            "Do not set brightness for "
            "on/breathing modes."
        ),
    )

    parser.add_argument(
        "--period-sec",
        type=_non_negative_float,
        default=1.5,
        help=(
            "Breathing period in seconds. "
            "Use --no-period-sec to omit it."
        ),
    )

    parser.add_argument(
        "--no-period-sec",
        action="store_true",
        help=(
            "Do not set period_sec for "
            "breathing mode."
        ),
    )

    parser.add_argument(
        "--command-id",
        type=int,
        default=int(time.time()),
        help=(
            "First command id. "
            "Each step increments it by 1."
        ),
    )

    parser.add_argument(
        "--repeat",
        type=_positive_int,
        default=1,
        help="Number of times to repeat the sequence.",
    )

    parser.add_argument(
        "--interval",
        type=_non_negative_float,
        default=1.0,
        help="Seconds to wait between sequence steps.",
    )

    parser.add_argument(
        "--wait-subscriber-timeout",
        type=_non_negative_float,
        default=5.0,
        help=(
            "Seconds to wait for arm_fsm to subscribe "
            "before publishing. Use 0 to skip."
        ),
    )

    args = parser.parse_args()

    try:
        args.sequence = _parse_sequence(
            args.sequence
        )
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))

    if args.no_brightness:
        args.brightness = None

    if args.no_period_sec:
        args.period_sec = None

    rclpy.init()

    node = HeadLightSdoSequenceTester(args)

    try:
        node.publish_sequence()

    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        exit_code = 2

    else:
        exit_code = 0

    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
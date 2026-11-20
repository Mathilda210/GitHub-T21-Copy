#!/usr/bin/env python3
"""Interface graphique Tkinter pour déplacer un endoscope avec ROS 2."""

import argparse
import math
import os
import threading
import time
from pathlib import Path
from queue import Empty, Queue
from typing import Callable, List, Optional, Sequence, Tuple

import rclpy
import tkinter as tk
from tkinter import messagebox
from rclpy.executors import SingleThreadedExecutor

if __package__:
    from .Finished_scripts import endoscope_manip as controller
else:
    from Finished_scripts import endoscope_manip as controller


TRANSLATION_STEP_METERS = 0.03
ROTATION_STEP_DEGREES = 5.0
GUI_POLL_PERIOD_MS = 100


class EndoscopeManipulationInterface:
    """Affiche les commandes et exécute chaque mouvement hors du thread Tk."""

    def __init__(self, node: controller.EndoscopeManipulation) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("Déplacement de l'endoscope")
        self.root.resizable(False, False)
        self._busy = False
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
            text="Translations de 3 cm et rotations de 5°",
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
                    command=lambda a=axis, d=direction: self._translate(a, d),
                )
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
                    command=lambda a=axis, d=direction: self._rotate(a, d),
                )
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

    def _fresh_arm_pose(self) -> Optional[List[float]]:
        pose = self.node._arm_pose
        if (
            pose is None
            or self.node._arm_pose_received_at < time.monotonic() - 1.0
        ):
            messagebox.showwarning(
                "Retour robot indisponible",
                "Aucune pose récente du bras n'est disponible. "
                "Vérifiez le topic TF du robot.",
                parent=self.root,
            )
            return None
        return list(pose)

    def _translate(self, axis: str, direction: float) -> None:
        current_pose = self._fresh_arm_pose()
        if current_pose is None:
            return

        sign = "+" if direction > 0 else "-"
        description = (
            f"translation {sign}3 cm selon {axis.upper()} de "
            f"{controller.TOOL_FRAME}"
        )
        if not messagebox.askyesno(
            "Confirmer le mouvement",
            f"Confirmer la {description} ?",
            parent=self.root,
        ):
            return

        target = list(current_pose)
        tool_axis = controller._rotate_vector_by_quaternion(
            controller.TRANSLATION_AXES[axis],
            current_pose[3:7],
        )
        for index, component in enumerate(tool_axis):
            target[index] += direction * TRANSLATION_STEP_METERS * component
        self._start_motion(
            description,
            lambda: self._move_cartesian(target, description),
        )

    def _rotate(self, axis: str, direction: float) -> None:
        current_pose = self._fresh_arm_pose()
        if current_pose is None:
            return

        sign = "+" if direction > 0 else "-"
        description = (
            f"rotation {sign}{ROTATION_STEP_DEGREES:.0f}° autour de "
            f"{axis.upper()} de {controller.TOOL_FRAME}"
        )
        if not messagebox.askyesno(
            "Confirmer le mouvement",
            f"Confirmer la {description} ?",
            parent=self.root,
        ):
            return

        half_angle = math.radians(direction * ROTATION_STEP_DEGREES) / 2.0
        sine = math.sin(half_angle)
        delta_rotation = [
            math.cos(half_angle),
            sine if axis == "x" else 0.0,
            sine if axis == "y" else 0.0,
            sine if axis == "z" else 0.0,
        ]
        target_orientation = controller._multiply_quaternions(
            current_pose[3:7],
            delta_rotation,
        )
        target = controller._pose_rotated_around_tool_tip(
            current_pose,
            target_orientation,
            self.node._tool_offset_in_arm,
        )
        self._start_motion(
            description,
            lambda: self._move_cartesian(
                target,
                description,
                check_orientation=True,
            ),
        )

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
        deadline = time.monotonic() + controller.MOTION_TIMEOUT

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
                or controller._quaternion_angular_error(
                    current_pose[3:7],
                    target[3:7],
                )
                <= controller.CARTESIAN_ANGULAR_TOLERANCE
            )
            if (
                position_error <= controller.CARTESIAN_TOLERANCE
                and orientation_reached
            ):
                return
            time.sleep(0.1)

        raise TimeoutError(
            f"La pose cible n'a pas été confirmée en "
            f"{controller.MOTION_TIMEOUT:.0f} s : {description}."
        )

    def _move_arm_home(self) -> None:
        self.node.publish_joint_target(
            controller.RIGHT_ARM_INDEX,
            controller.HOME_RIGHT_ARM,
        )

        def target_reached() -> bool:
            return all(
                name in self.node._joint_positions
                and abs(self.node._joint_positions[name] - value)
                <= controller.JOINT_TOLERANCE
                for name, value in zip(
                    controller.RIGHT_ARM_JOINTS,
                    controller.HOME_RIGHT_ARM,
                )
            )

        if not controller._spin_until(target_reached, controller.FEEDBACK_TIMEOUT):
            raise TimeoutError(
                "Le bras n'a pas atteint sa position home avant le délai imparti."
            )

    def _poll_results(self) -> None:
        try:
            succeeded, message = self._results.get_nowait()
        except Empty:
            pass
        else:
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

    def _set_controls_enabled(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        for button in self._motion_buttons:
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
    if not controller._confirm(
        "Demarrer la sequence home puis saisie de l'endoscope"
    ):
        print("Sequence annulee.")
        return

    rclpy.init(args=args)
    node = None
    executor = None
    spin_thread = None
    try:
        node = controller.EndoscopeManipulation(
            joint_topic,
            tf_topic,
            Path(options.urdf_path),
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        if not controller._move_joints(
            node,
            "Placer le bras droit en position home",
            controller.RIGHT_ARM_INDEX,
            controller.RIGHT_ARM_JOINTS,
            controller.HOME_RIGHT_ARM,
        ):
            return
        if not controller._move_joints(
            node,
            "Ouvrir la main droite avant la saisie",
            controller.RIGHT_HAND_INDEX,
            controller.RIGHT_HAND_JOINTS,
            controller.HAND_OPEN,
        ):
            return

        for description, pose in controller.ENDOSCOPE_GRASP:
            if not controller._move_joints(
                node,
                f"{description.capitalize()} (pose mesuree)",
                controller.RIGHT_HAND_INDEX,
                controller.RIGHT_HAND_JOINTS,
                pose,
            ):
                print("Saisie interrompue par l'utilisateur.")
                return

        if controller._wait_for_arm_pose(node) is None:
            raise TimeoutError(
                f"TF {controller.ARM_COMMAND_FRAME} -> {controller.ARM_FRAME} "
                f"non recue en {controller.FEEDBACK_TIMEOUT:.0f} s."
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
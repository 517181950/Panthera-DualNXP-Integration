"""Joint-position control for the dual Panthera HT dishwasher MuJoCo scene."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Literal

import mujoco


Side = Literal["left", "right"]


class DishwasherSim:
    """Dishwasher scene controlled directly through joint position targets.

    Arm angles are radians unless ``degrees=True`` is supplied. Gripper values
    are single-finger displacements in metres and use ``set_gripper``.
    """

    ARM_JOINTS = (
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    )
    # Preserve the original Panthera URDF joint zero, with no pose offsets.
    INITIAL_ARM_ANGLES = (0.0,) * 6
    INITIAL_GRIPPER_OPENING = 0.04

    def __init__(self, scene_path: str | Path | None = None) -> None:
        if scene_path is None:
            scene_path = Path(__file__).parent / "assets" / "panthera" / "scene.xml"

        self.scene_path = Path(scene_path).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)
        self.reset()

    @staticmethod
    def _side(side: str) -> Side:
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        return side

    def _actuator_id(self, name: str) -> int:
        actuator_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
        )
        if actuator_id < 0:
            raise KeyError(f"unknown actuator: {name}")
        return actuator_id

    def _joint_id(self, name: str) -> int:
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise KeyError(f"unknown joint: {name}")
        return joint_id

    def _set_initial_joint_position(self, name: str, value: float) -> None:
        joint_id = self._joint_id(name)
        qpos_address = self.model.jnt_qposadr[joint_id]
        self.data.qpos[qpos_address] = value

    def _set_target(self, actuator_name: str, value: float) -> float:
        actuator_id = self._actuator_id(actuator_name)
        if self.model.actuator_ctrllimited[actuator_id]:
            lower, upper = self.model.actuator_ctrlrange[actuator_id]
            value = min(max(float(value), float(lower)), float(upper))
        self.data.ctrl[actuator_id] = value
        return value

    def reset(self) -> None:
        """Restore a deterministic scene without running any trajectory."""
        mujoco.mj_resetData(self.model, self.data)

        for side in ("left", "right"):
            for joint, angle in zip(self.ARM_JOINTS, self.INITIAL_ARM_ANGLES):
                name = f"{side}/{joint}"
                self._set_initial_joint_position(name, angle)
                self._set_target(name, angle)

            for finger, direction in (("gripper_L_joint", 1), ("gripper_R_joint", -1)):
                self._set_initial_joint_position(
                    f"{side}/{finger}", direction * self.INITIAL_GRIPPER_OPENING
                )
            self._set_target(f"{side}/gripper", self.INITIAL_GRIPPER_OPENING)

        # Prop poses and the rack's active weld are MJCF defaults, so the
        # native viewer's Reset (mj_resetData) restores the same arrangement.
        mujoco.mj_forward(self.model, self.data)

    def set_joint_angle(
        self, side: Side, joint: str, angle: float, *, degrees: bool = False
    ) -> float:
        """Set one arm joint target and return the clipped target in radians."""
        side = self._side(side)
        if joint not in self.ARM_JOINTS:
            raise ValueError(f"joint must be one of {self.ARM_JOINTS}, got {joint!r}")
        if degrees:
            angle = angle * mujoco.mjPI / 180.0
        return self._set_target(f"{side}/{joint}", angle)

    def set_arm_angles(
        self, side: Side, angles: Sequence[float], *, degrees: bool = False
    ) -> tuple[float, ...]:
        """Set all six arm joint targets in model order."""
        side = self._side(side)
        if len(angles) != len(self.ARM_JOINTS):
            raise ValueError(f"expected 6 angles, got {len(angles)}")
        return tuple(
            self.set_joint_angle(side, joint, angle, degrees=degrees)
            for joint, angle in zip(self.ARM_JOINTS, angles)
        )

    def set_gripper(self, side: Side, opening_metres: float) -> float:
        """Set single-finger travel, clipped to 0--0.04 m (opposed fingers)."""
        side = self._side(side)
        return self._set_target(f"{side}/gripper", opening_metres)

    def get_arm_angles(self, side: Side, *, degrees: bool = False) -> tuple[float, ...]:
        """Read the six current simulated joint positions."""
        side = self._side(side)
        scale = 180.0 / mujoco.mjPI if degrees else 1.0
        values = []
        for joint in self.ARM_JOINTS:
            joint_id = self._joint_id(f"{side}/{joint}")
            qpos_address = self.model.jnt_qposadr[joint_id]
            values.append(float(self.data.qpos[qpos_address]) * scale)
        return tuple(values)

    def step(self, steps: int = 1) -> None:
        """Advance physics without creating camera images."""
        if steps < 1:
            raise ValueError("steps must be at least 1")
        mujoco.mj_step(self.model, self.data, nstep=steps)

    def launch_viewer(self) -> None:
        """Open MuJoCo Simulate; use its Control panel to move actuators."""
        import mujoco.viewer

        mujoco.viewer.launch(
            model=self.model,
            data=self.data,
            show_left_ui=True,
            show_right_ui=True,
        )

    def launch_passive_viewer(self):
        """Open a non-blocking viewer for a future external control loop.

        The returned MuJoCo handle is a context manager. Call setters and
        ``step`` while it is open, then call ``handle.sync()``.
        """
        import mujoco.viewer

        return mujoco.viewer.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=True,
            show_right_ui=True,
        )

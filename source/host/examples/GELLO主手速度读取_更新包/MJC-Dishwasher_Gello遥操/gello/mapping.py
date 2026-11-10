"""Pure conversion from Gello snapshots to MuJoCo joint targets."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

from .config import ArmConfig, JOINT_NAMES
from .reader import GroupSnapshot


@dataclass(frozen=True)
class ArmTargets:
    """One arm's six joint targets in radians and gripper opening in metres."""

    joints: tuple[float, ...]
    gripper: float


def map_group_to_targets(
    snapshot: GroupSnapshot,
    config: ArmConfig,
    neutral_angles: Sequence[float],
) -> ArmTargets:
    """Map one complete, zeroed Gello group onto a MuJoCo neutral pose.

    ``snapshot.arm_radians`` already contains the servo direction and gear
    conversion used by the Gello reader.  The user mapping is then applied as
    a delta around ``neutral_angles``.
    """
    config.validate()
    if len(neutral_angles) != len(JOINT_NAMES):
        raise ValueError(f"expected {len(JOINT_NAMES)} neutral angles")
    if not snapshot.data_valid:
        raise ValueError(f"Gello group {snapshot.group} snapshot is incomplete")
    if not snapshot.zeroed:
        raise ValueError(f"Gello group {snapshot.group} has not been zeroed")
    if len(snapshot.arm_radians) != len(JOINT_NAMES):
        raise ValueError(
            f"Gello group {snapshot.group} must contain {len(JOINT_NAMES)} arm angles"
        )
    if len(snapshot.raw_positions) < len(JOINT_NAMES) + 1:
        raise ValueError(f"Gello group {snapshot.group} has no gripper position")

    joints: list[float] = []
    for neutral, radians, axis in zip(
        neutral_angles, snapshot.arm_radians, config.axes
    ):
        if radians is None or not math.isfinite(radians):
            raise ValueError(f"Gello group {snapshot.group} has an invalid arm angle")
        if not math.isfinite(neutral):
            raise ValueError("neutral angles must be finite")
        joints.append(float(neutral) + axis.apply(radians))

    gripper_normalized = snapshot.gripper_normalized
    if gripper_normalized is None or not math.isfinite(gripper_normalized):
        raise ValueError(f"Gello group {snapshot.group} gripper is not calibrated")

    return ArmTargets(
        tuple(joints), config.gripper.apply_fraction(gripper_normalized)
    )

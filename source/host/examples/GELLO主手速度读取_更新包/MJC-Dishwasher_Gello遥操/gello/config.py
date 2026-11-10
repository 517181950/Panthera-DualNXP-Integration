"""Validated JSON configuration for Gello acquisition and joint mapping."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
from typing import Any


JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)


@dataclass
class AxisConfig:
    sign: int = 1
    scale: float = 1.0
    offset_rad: float = 0.0

    def validate(self) -> None:
        if self.sign not in (-1, 1):
            raise ValueError("axis sign must be -1 or 1")
        if not math.isfinite(self.scale) or self.scale <= 0:
            raise ValueError("axis scale must be a positive finite number")
        if not math.isfinite(self.offset_rad):
            raise ValueError("axis offset_rad must be finite")

    def apply(self, radians: float) -> float:
        return self.sign * self.scale * radians + self.offset_rad

    def apply_velocity(self, radians_per_second: float) -> float:
        """Map angular velocity; position offsets do not affect velocity."""
        return self.sign * self.scale * radians_per_second


@dataclass
class GripperConfig:
    min_raw: int = 0
    max_raw: int = 4095
    inverted: bool = False
    output_min: float = 0.0
    output_max: float = 0.04

    def validate(self) -> None:
        if self.min_raw >= self.max_raw:
            raise ValueError("gripper unwrapped range must satisfy min < max")
        if not (
            math.isfinite(self.output_min)
            and math.isfinite(self.output_max)
            and self.output_min < self.output_max
        ):
            raise ValueError("gripper output range must be finite and increasing")

    def apply(self, unwrapped_ticks: int) -> float:
        fraction = (unwrapped_ticks - self.min_raw) / (
            self.max_raw - self.min_raw
        )
        return self.apply_fraction(fraction)

    def apply_fraction(self, fraction: float) -> float:
        if not math.isfinite(fraction):
            raise ValueError("gripper fraction must be finite")
        fraction = min(max(fraction, 0.0), 1.0)
        if self.inverted:
            fraction = 1.0 - fraction
        return self.output_min + fraction * (self.output_max - self.output_min)


def _default_axes() -> list[AxisConfig]:
    return [AxisConfig() for _ in JOINT_NAMES]


@dataclass
class ArmConfig:
    axes: list[AxisConfig] = field(default_factory=_default_axes)
    gripper: GripperConfig = field(default_factory=GripperConfig)

    def validate(self) -> None:
        if len(self.axes) != len(JOINT_NAMES):
            raise ValueError(f"each arm must define {len(JOINT_NAMES)} joint axes")
        for axis in self.axes:
            axis.validate()
        self.gripper.validate()


@dataclass
class SerialConfig:
    port: str = "/dev/ttyACM0"
    baud: int = 1_000_000
    group_mask: int = 0x3
    timeout_ms: float = 5.0
    scan_interval_ms: float = 0.0
    max_delta_ticks: int = 100
    auto_reconnect: bool = True
    link_loss_ms: float = 500.0
    reconnect_initial_ms: float = 250.0
    reconnect_max_ms: float = 2000.0
    read_velocity: bool = True
    velocity_rpm_per_unit: float = 0.732

    def validate(self) -> None:
        if not self.port:
            raise ValueError("serial port must not be empty")
        if not isinstance(self.read_velocity, bool):
            raise ValueError("serial read_velocity must be a boolean")
        if not math.isfinite(self.velocity_rpm_per_unit) or self.velocity_rpm_per_unit <= 0:
            raise ValueError("serial velocity_rpm_per_unit must be positive and finite")
        if self.baud <= 0:
            raise ValueError("serial baud must be positive")
        if self.group_mask not in (0x1, 0x2, 0x3):
            raise ValueError("serial group_mask must be 1, 2, or 3")
        if not math.isfinite(self.timeout_ms) or self.timeout_ms <= 0:
            raise ValueError("serial timeout_ms must be positive")
        if not math.isfinite(self.scan_interval_ms) or self.scan_interval_ms < 0:
            raise ValueError("serial scan_interval_ms must be non-negative")
        if not 0 < self.max_delta_ticks < 2048:
            raise ValueError("serial max_delta_ticks must be in the range 1..2047")
        if not math.isfinite(self.link_loss_ms) or self.link_loss_ms <= 0:
            raise ValueError("serial link_loss_ms must be positive")
        if (
            not math.isfinite(self.reconnect_initial_ms)
            or self.reconnect_initial_ms <= 0
        ):
            raise ValueError("serial reconnect_initial_ms must be positive")
        if (
            not math.isfinite(self.reconnect_max_ms)
            or self.reconnect_max_ms < self.reconnect_initial_ms
        ):
            raise ValueError(
                "serial reconnect_max_ms must be at least reconnect_initial_ms"
            )


@dataclass
class GelloConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)
    swap_arms: bool = False
    left: ArmConfig = field(default_factory=ArmConfig)
    right: ArmConfig = field(default_factory=ArmConfig)

    def validate(self) -> None:
        self.serial.validate()
        self.left.validate()
        self.right.validate()


def _axis_from_dict(value: dict[str, Any]) -> AxisConfig:
    return AxisConfig(
        sign=int(value.get("sign", 1)),
        scale=float(value.get("scale", 1.0)),
        offset_rad=float(value.get("offset_rad", 0.0)),
    )


def _gripper_from_dict(value: dict[str, Any]) -> GripperConfig:
    return GripperConfig(
        min_raw=int(value.get("min_raw", 0)),
        max_raw=int(value.get("max_raw", 4095)),
        inverted=bool(value.get("inverted", False)),
        output_min=float(value.get("output_min", 0.0)),
        output_max=float(value.get("output_max", 0.04)),
    )


def _arm_from_dict(value: dict[str, Any]) -> ArmConfig:
    axes_raw = value.get("axes", [{} for _ in JOINT_NAMES])
    return ArmConfig(
        axes=[_axis_from_dict(axis) for axis in axes_raw],
        gripper=_gripper_from_dict(value.get("gripper", {})),
    )


def config_from_dict(value: dict[str, Any]) -> GelloConfig:
    serial_raw = value.get("serial", {})
    config = GelloConfig(
        serial=SerialConfig(
            port=str(serial_raw.get("port", "/dev/ttyACM0")),
            baud=int(serial_raw.get("baud", 1_000_000)),
            group_mask=int(serial_raw.get("group_mask", 0x3)),
            timeout_ms=float(serial_raw.get("timeout_ms", 5.0)),
            scan_interval_ms=float(serial_raw.get("scan_interval_ms", 0.0)),
            max_delta_ticks=int(serial_raw.get("max_delta_ticks", 100)),
            auto_reconnect=bool(serial_raw.get("auto_reconnect", True)),
            link_loss_ms=float(serial_raw.get("link_loss_ms", 500.0)),
            reconnect_initial_ms=float(
                serial_raw.get("reconnect_initial_ms", 250.0)
            ),
            reconnect_max_ms=float(serial_raw.get("reconnect_max_ms", 2000.0)),
            read_velocity=serial_raw.get("read_velocity", True),
            velocity_rpm_per_unit=float(serial_raw.get("velocity_rpm_per_unit", 0.732)),
        ),
        swap_arms=bool(value.get("swap_arms", False)),
        left=_arm_from_dict(value.get("left", {})),
        right=_arm_from_dict(value.get("right", {})),
    )
    config.validate()
    return config


def load_config(path: str | Path) -> GelloConfig:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("Gello config root must be a JSON object")
    return config_from_dict(value)


def save_config(config: GelloConfig, path: str | Path) -> None:
    config.validate()
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        json.dumps(asdict(config), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)

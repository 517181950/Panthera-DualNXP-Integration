"""Dual-arm serial Gello acquisition and tuning support."""

from .config import (
    ArmConfig,
    AxisConfig,
    GelloConfig,
    GripperConfig,
    JOINT_NAMES,
    SerialConfig,
    load_config,
    save_config,
)
from .reader import (
    BusSilenceError,
    ConnectionState,
    ConnectionStatus,
    GROUP_SERVO_IDS,
    GelloReader,
    GroupSnapshot,
    ReaderSnapshot,
    ReaderStats,
    wrapped_tick_delta,
)
from .mapping import ArmTargets, map_group_to_targets
from .protocol import decode_signed_magnitude

__all__ = [
    "ArmConfig",
    "ArmTargets",
    "AxisConfig",
    "BusSilenceError",
    "ConnectionState",
    "ConnectionStatus",
    "GelloConfig",
    "GelloReader",
    "GROUP_SERVO_IDS",
    "GripperConfig",
    "GroupSnapshot",
    "JOINT_NAMES",
    "ReaderSnapshot",
    "ReaderStats",
    "SerialConfig",
    "load_config",
    "decode_signed_magnitude",
    "map_group_to_targets",
    "save_config",
    "wrapped_tick_delta",
]

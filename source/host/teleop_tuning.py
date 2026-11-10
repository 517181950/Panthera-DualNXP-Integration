"""Independent arm overrides and P1L mode-3 hand tuning for shared GELLO."""
from copy import deepcopy
from dataclasses import dataclass, field, asdict
import math
import numpy as np
from soft_limits import Limits, vector
from hand_slider_demo import gripper_targets_deg


@dataclass
class HandTuning:
    profile: str = 'five_finger'
    current_ma: int = 500
    grasp_scale: float = 1.0
    grasp_offset_percent: float = 0.0
    grasp_inverted: bool = False
    joint_scale: list = field(default_factory=lambda: [1.0]*6)
    joint_offset_deg: list = field(default_factory=lambda: [0.0]*6)
    max_target_speed_deg_s: list = None
    feedback_timeout_ms: float = 100.0

    def __post_init__(self):
        if self.profile not in ('five_finger', 'two_finger'):
            raise ValueError('Unknown hand profile')
        if type(self.current_ma) is not int or not 0 <= self.current_ma <= 65535:
            raise ValueError('current_ma must fit the unsigned 16-bit mode-3 field')
        if type(self.grasp_inverted) is not bool:
            raise ValueError('grasp_inverted must be a boolean')
        for name in ('grasp_scale', 'grasp_offset_percent', 'feedback_timeout_ms'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if self.grasp_scale <= 0 or self.feedback_timeout_ms <= 0:
            raise ValueError('grasp_scale and feedback_timeout_ms must be positive')
        self.joint_scale = vector(self.joint_scale, 'hand joint_scale').tolist()
        self.joint_offset_deg = vector(self.joint_offset_deg, 'hand joint_offset_deg').tolist()
        if self.max_target_speed_deg_s is not None:
            speed = vector(self.max_target_speed_deg_s, 'hand max_target_speed_deg_s')
            if np.any(speed <= 0):
                raise ValueError('Hand target speeds must be positive, or use null to disable')
            self.max_target_speed_deg_s = speed.tolist()
        # Each profile is piecewise linear. Checking every knot validates the
        # entire mapped travel against wire representability, not physical limits.
        for percent in (0, 20, 95, 100):
            self._scaled_target(percent)

    def _scaled_target(self, percent):
        q = np.deg2rad(np.asarray(gripper_targets_deg(percent, self.profile))*self.joint_scale+self.joint_offset_deg)
        if np.any(q < 0) or np.any(q > 65.535):
            raise ValueError('Mapped hand target outside unsigned milliradian representation')
        return q

    def target(self, percent):
        if not math.isfinite(percent) or not 0 <= percent <= 100:
            raise ValueError('Grasp input must be in [0,100]')
        if self.grasp_inverted:
            percent = 100-percent
        percent = float(np.clip(percent*self.grasp_scale+self.grasp_offset_percent, 0, 100))
        return self._scaled_target(percent)

    def document(self):
        return asdict(self)


class HandTargetFilter:
    """Optional per-channel command slew bound, not a physical finger NCBF."""
    def __init__(self, tuning):
        self.tuning = tuning
        self.previous = None
        self.updated = None

    def update(self, desired, actual, now):
        desired = vector(desired, 'hand desired')
        if not math.isfinite(now):
            raise ValueError('Nonfinite hand clock')
        if np.any(desired < 0) or np.any(desired > 65.535):
            raise ValueError('Hand target outside wire range')
        if self.tuning.max_target_speed_deg_s is None:
            self.previous, self.updated = desired.copy(), now
            return desired.tolist()
        if self.previous is None:
            self.previous = vector(actual, 'hand actual')
            if np.any(self.previous < 0) or np.any(self.previous > 65.535):
                raise ValueError('Hand initial position outside wire range')
            dt = .02  # First command is limited to one 50 Hz command interval.
        else:
            dt = now-self.updated
            if dt <= 0 or dt*1000 >= self.tuning.feedback_timeout_ms:
                raise RuntimeError('Hand target filter update timed out')
        step = np.deg2rad(self.tuning.max_target_speed_deg_s)*dt
        self.previous += np.clip(desired-self.previous, -step, step)
        self.updated = now
        return self.previous.tolist()


def resolve_tuning(config, shared_soft_limits):
    """Global soft-limit defaults + per-board overrides; hand defaults per side."""
    soft, hands = {}, {}
    for side, board in config['boards'].items():
        overrides = board.get('soft_limits', {})
        if not isinstance(overrides, dict):
            raise ValueError(f'{side}.soft_limits must be an object')
        merged = deepcopy(shared_soft_limits)
        merged.update(deepcopy(overrides))
        Limits(**merged)  # Reject typos and incompatible tuning before any I/O.
        soft[side] = merged
        hands[side] = HandTuning(**board.get('hand_control', {}))
    return soft, hands

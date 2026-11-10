"""Angle-based wall with bidirectional damping for manual guidance.

This is a host-side force cue, not a rigid constraint or collision detector.
There is no integral, fault latch at the boundary, or automatic homing.
Angles are logical encoder radians. Returned torque is the expected net wall
contribution at the sampled velocity. Send diag['feedforward_nm'] with zero
velocity reference and diag['kd'] to apply damping in the motor control loop.
The cap bounds feedforward and expected net torque at the sampled velocity;
the motor's actual instantaneous torque remains subject to its own limits.
"""
from dataclasses import dataclass
import math
import numpy as np
from soft_limits import vector


@dataclass
class WallSettings:
    damping_nm_s_rad: list
    max_torque_nm: list
    cushion_deg: list
    damping_zone_deg: list
    startup_ramp_s: float = 1.0
    low_speed_damping_ratio: float = 0.25
    full_damping_speed_deg_s: float = 20.0

    def __post_init__(self):
        for name in ('damping_nm_s_rad', 'max_torque_nm',
                     'cushion_deg', 'damping_zone_deg'):
            value = vector(getattr(self, name), name)
            if np.any(value <= 0):
                raise ValueError(f'{name} must be positive')
            setattr(self, name, value)
        for name in ('startup_ramp_s', 'full_damping_speed_deg_s'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(self.low_speed_damping_ratio) or not 0 < self.low_speed_damping_ratio <= 1:
            raise ValueError('low_speed_damping_ratio must be in (0, 1]')
        if np.any(self.damping_zone_deg <= self.cushion_deg):
            raise ValueError('Damping zone must exceed the spring cushion')


class GravityWall:
    def __init__(self, lower_deg, upper_deg, settings):
        self.lower = np.deg2rad(vector(lower_deg, 'lower_deg'))
        self.upper = np.deg2rad(vector(upper_deg, 'upper_deg'))
        self.settings = settings
        self.cushion = np.deg2rad(settings.cushion_deg)
        self.damping_zone = np.deg2rad(settings.damping_zone_deg)
        if np.any(self.upper-self.lower <= 2*self.damping_zone):
            raise ValueError('Each limit range must exceed twice its damping zone')

    @staticmethod
    def smooth(x):
        x = np.clip(x, 0, 1)
        return x*x*(3-2*x)

    def command(self, q, dq, elapsed_s, gravity=None):
        q = vector(q, 'actual position')
        dq = vector(dq, 'actual velocity')
        if not math.isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError('Elapsed time must be finite and nonnegative')
        c = self.settings
        gravity = np.zeros(6) if gravity is None else vector(gravity, 'gravity torque')
        baseline = np.clip(gravity, -c.max_torque_nm, c.max_torque_nm)
        spring = np.zeros(6)
        damping_weight = np.zeros(6)
        # Lower wall points positive, upper wall negative. Springs begin just
        # inside the configured boundary to develop force before that angle.
        for distance, inward_sign in ((q-self.lower, 1), (self.upper-q, -1)):
            # Fixed angular band, no time-to-boundary or per-contact timer.
            # The slope is zero at both ends of the angular transition.
            fraction = self.smooth(1-distance/self.cushion)
            # Reserve directional headroom INCLUDING gravity. Opposing gravity
            # must not reduce the requested full inward net effort at the wall.
            headroom = c.max_torque_nm-inward_sign*baseline
            spring += inward_sign*fraction*headroom
            # Full damping before the spring starts; brake both approach and
            # rebound. Position-only weighting is continuous at velocity zero.
            damping_weight = np.maximum(damping_weight, self.smooth(
                (self.damping_zone-distance)/(self.damping_zone-self.cushion)))
        ramp = self.smooth(elapsed_s/c.startup_ramp_s)
        undamped = np.clip(baseline+spring*ramp, -c.max_torque_nm, c.max_torque_nm)
        # Motor-local velocity feedback avoids a delayed host damping loop.
        # Keep both feedforward and predicted net effort inside the envelope:
        # -cap <= ff <= cap and -cap <= ff - kd*dq <= cap.
        # Smooth, nonnegative speed scheduling: light slow dragging, original
        # braking at higher speed in either direction. Damping still runs locally.
        speed_weight = self.smooth(np.abs(dq)/np.deg2rad(c.full_damping_speed_deg_s))
        speed_scale = c.low_speed_damping_ratio+(1-c.low_speed_damping_ratio)*speed_weight
        kd = np.minimum(c.damping_nm_s_rad*damping_weight*ramp*speed_scale,
                        2*c.max_torque_nm/np.maximum(np.abs(dq), 1e-12))
        damping_drop = np.clip(kd*dq, -2*c.max_torque_nm, 2*c.max_torque_nm)
        ff_lower = np.maximum(-c.max_torque_nm, -c.max_torque_nm+damping_drop)
        ff_upper = np.minimum(c.max_torque_nm, c.max_torque_nm+damping_drop)
        feedforward = np.clip(undamped, ff_lower, ff_upper)
        final = feedforward-damping_drop
        total = final-gravity
        diag = dict(active=np.abs(total)>1e-6,
                    command_nm=final,
                    feedforward_nm=feedforward,
                    kd=kd,
                    damping_nm=final-undamped,
                    outside=(q<self.lower)|(q>self.upper),
                    distance_deg=np.rad2deg(np.minimum(q-self.lower, self.upper-q)))
        return total, diag

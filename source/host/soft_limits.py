"""Position clipping with an optional NCBF trajectory filter for teleoperation.

Inspired by Lawan et al., eq. (9). This protects the commanded reference,
not a torque-controlled physical plant: no inertia, wrench, or DOB assumptions
are made. Boundary arrival blocks outward references without a fault latch;
inward references remain available. Invalid feedback and motor faults still
raise for the caller's soft-stop path. All internal positions are logical
encoder coordinates in radians.
"""
from dataclasses import dataclass
import math
import numpy as np


class SoftLimitFault(RuntimeError):
    pass


def vector(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name}: expected six finite numbers')
    return result.copy()


@dataclass
class Limits:
    lower_deg: list
    upper_deg: list
    velocity_deg_s: list
    acceleration_deg_s2: list
    # Legacy saved configurations may contain this field. It no longer gates
    # measured feedback; reference speed/acceleration apply only with the filter.
    measured_velocity_deg_s: list = None
    exponent: int = 6
    target_gain_s: float = 4.0
    feedback_age_ms: float = 20.0
    max_period_ms: float = 20.0
    # Accepted only for compatibility with saved configurations; not fault gates.
    position_tolerance_deg: float = None
    tracking_error_deg: float = None
    prediction_ms: float = 20.0
    reference_filter: bool = True  # False: position clipping only, original MIT dq=0.

    def __post_init__(self):
        if type(self.reference_filter) is not bool:
            raise ValueError('reference_filter must be a boolean')
        for name in ('lower_deg', 'upper_deg', 'velocity_deg_s', 'acceleration_deg_s2'):
            setattr(self, name, vector(getattr(self, name), name))
        self.lower = np.deg2rad(self.lower_deg)
        self.upper = np.deg2rad(self.upper_deg)
        self.speed = np.deg2rad(self.velocity_deg_s)
        self.acceleration = np.deg2rad(self.acceleration_deg_s2)
        self.center = (self.upper + self.lower) / 2
        self.radius = (self.upper - self.lower) / 2
        if np.any(self.radius <= 0) or np.any(self.speed <= 0) or np.any(self.acceleration <= 0):
            raise ValueError('Soft limits require lower < upper and positive speed/acceleration')
        if type(self.exponent) is not int or self.exponent < 2 or self.exponent % 2:
            raise ValueError('NCBF exponent must be an even integer >= 2')
        fields=['target_gain_s','feedback_age_ms','max_period_ms','prediction_ms']
        for name in fields:
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        # Sufficient conservative curvature condition for braking along the
        # polynomial envelope. Direct clipping does not use this envelope.
        if self.reference_filter and np.any(self.exponent * self.speed**2 / self.radius > self.acceleration):
            raise ValueError('NCBF curvature requires lower speed or higher reference acceleration')


class SoftLimitGovernor:
    def __init__(self, limits):
        self.limits = limits
        self.q = None
        self.v = np.zeros(6)
        self.updated = None
        self.fault = None

    def _fail(self, reason):
        self.fault = reason
        raise SoftLimitFault(reason)

    def check_feedback(self, state, now):
        if self.fault is not None:
            raise SoftLimitFault(self.fault)
        try:
            q = vector(state['q_rad'], 'actual position')
            v = vector(state['dq_rad_s'], 'actual velocity')
            age = vector(state['feedback_age_us'], 'feedback age')
            fault = vector(state['fault_code_raw'], 'motor faults')
            received = float(state['host_received_monotonic_s'])
        except (KeyError, TypeError, ValueError) as exc:
            self._fail(str(exc))
        c = self.limits
        if any(state.get(key) != 63 for key in ('online_mask', 'valid_q_mask', 'valid_dq_mask', 'motor_fault_valid_mask')):
            self._fail('Soft limit: incomplete feedback validity masks')
        if np.any(fault != 0):
            self._fail('Soft limit: motor reports a fault')
        host_age = now-received
        if (not math.isfinite(host_age) or host_age < 0 or np.any(age < 0)
                or np.any(age+host_age*1e6 >= c.feedback_age_ms*1000)):
            self._fail('Soft limit: missing/stale joint feedback')
        return q, v

    def _outward_speed(self, distance_from_center, remaining_distance):
        c = self.limits
        polynomial = c.speed * (1 - np.clip(distance_from_center/c.radius, 0, 1)**c.exponent)
        braking = np.sqrt(2*c.acceleration*np.maximum(remaining_distance, 0))
        return np.minimum(polynomial, braking)

    def barrier(self, q, v):
        c = self.limits
        x = (np.asarray(q)-c.center)/c.radius
        y = np.asarray(v)/c.speed
        return np.minimum(1-np.maximum(x, 0)**c.exponent-np.maximum(y, 0),
                          1-np.maximum(-x, 0)**c.exponent-np.maximum(-y, 0))

    def update(self, target, state, now):
        actual, actual_v = self.check_feedback(state, now)
        try:
            target = vector(target, 'target')
        except (TypeError, ValueError) as exc:
            self._fail(str(exc))
        if not math.isfinite(now):
            self._fail('Soft limit: nonfinite clock')
        c = self.limits
        clipped = np.clip(target, c.lower, c.upper)
        if not c.reference_filter:
            # Restore the original absolute-position teleoperation mapping.
            # No accumulated trajectory lag or velocity feedforward. Limits
            # only clip position; reversing inside the bounds takes one cycle.
            self.q = clipped
            self.v = np.zeros(6)
            self.updated = now
            return self.q.copy(), self.v.copy(), self._diagnostic(target, clipped)
        if self.q is None:
            self.q = np.clip(actual, c.lower, c.upper)
            self.updated = now
            return self.q.copy(), self.v.copy(), self._diagnostic(target, clipped)
        dt = now-self.updated
        if dt <= 0 or dt > c.max_period_ms/1000:
            # Re-anchor after scheduling gaps instead of integrating a large step.
            self.q = np.clip(actual, c.lower, c.upper)
            self.v = np.zeros(6)
            self.updated = now
            return self.q.copy(), self.v.copy(), self._diagnostic(target, clipped)

        # q_next=q+dt*(v+v_next)/2. Each outward NCBF condition is
        # monotone in v_next; bisection finds the feasible velocity interval.
        # Projection into its intersection solves the separable box QP
        # min ||v_next-v_nom||^2 (equivalently an acceleration QP).
        base = self.q + .5*dt*self.v
        lo = np.zeros((2, 6)); hi = np.tile(c.speed, (2, 1))
        centers = np.array([base-c.center, c.center-base])
        remaining = np.array([c.upper-base, base-c.lower])
        for _ in range(32):
            mid = (lo+hi)/2
            cap = self._outward_speed(centers+.5*dt*mid, remaining-.5*dt*mid)
            feasible = mid <= cap
            lo = np.where(feasible, mid, lo)
            hi = np.where(feasible, hi, mid)
        upper, negative_lower = lo

        # Also restrict outward commands using each arm's measured pose and
        # an outward-only constant-velocity prediction. This is a heuristic
        # feedback guard, not a certified physical stopping-distance model.
        delay = c.prediction_ms/1000
        upper = np.minimum(upper, self._outward_speed(
            actual-c.center+np.maximum(actual_v, 0)*delay,
            c.upper-actual-np.maximum(actual_v, 0)*delay))
        negative_lower = np.minimum(negative_lower, self._outward_speed(
            c.center-actual+np.maximum(-actual_v, 0)*delay,
            actual-c.lower-np.maximum(-actual_v, 0)*delay))
        lower = np.maximum.reduce([-negative_lower, self.v-c.acceleration*dt,
                                   2*(c.lower-self.q)/dt-self.v])
        upper = np.minimum.reduce([upper, self.v+c.acceleration*dt,
                                   2*(c.upper-self.q)/dt-self.v])
        blocked = lower > upper+1e-12
        if np.any(blocked):
            # A sudden measured boundary arrival can demand braking faster than
            # the reference acceleration limit. Stop only those axes at their
            # bounded measured pose; next cycle may immediately move inward.
            self.q[blocked] = np.clip(actual, c.lower, c.upper)[blocked]
            self.v[blocked] = 0
            lower[blocked] = upper[blocked] = 0
        error = clipped-self.q
        nominal = np.sign(error)*np.minimum(c.target_gain_s*np.abs(error),
                                            np.sqrt(2*c.acceleration*np.abs(error)))
        next_v = np.clip(nominal, lower, np.maximum(lower, upper))
        next_q = self.q + .5*dt*(self.v+next_v)
        self.q = np.clip(next_q, c.lower, c.upper)
        # Final projection handles rounding at the boundary without latching
        # a fault. Only outward velocity is suppressed; inward remains available.
        self.v = np.clip(next_v,
            -self._outward_speed(c.center-self.q, self.q-c.lower),
            self._outward_speed(self.q-c.center, c.upper-self.q))
        self.updated = now
        return self.q.copy(), self.v.copy(), self._diagnostic(target, clipped)

    def _diagnostic(self, target, clipped):
        return dict(target_clipped=bool(np.any(target != clipped)),
                    filtered=bool(np.max(np.abs(target-self.q)) > 1e-7),
                    min_barrier=float(np.min(self.barrier(self.q, self.v))))

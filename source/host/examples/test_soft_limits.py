"""Offline reference-invariant and dual-arm failure-path tests; no hardware."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch
import xml.etree.ElementTree as ET

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soft_limits import Limits, SoftLimitGovernor, SoftLimitFault

ROOT = Path(__file__).resolve().parents[3]


def config():
    data = json.loads((ROOT/'soft-limits.json').read_text())
    data['reference_filter'] = True  # Exercise the optional legacy trajectory filter.
    return data


def state(q, v, now):
    return dict(q_rad=list(q), dq_rad_s=list(v), feedback_age_us=[1000]*6,
                fault_code_raw=[0]*6, online_mask=63, valid_q_mask=63,
                valid_dq_mask=63, motor_fault_valid_mask=63,
                host_received_monotonic_s=now)


class Tests(unittest.TestCase):
    def test_narrow_direct_limits_do_not_require_inactive_filter_curvature(self):
        from teleop_tuning import resolve_tuning
        shared = json.loads((ROOT/'soft-limits.json').read_text())
        entry = {'boards': {'left': {'soft_limits': {
            'lower_deg': [-10]*6, 'upper_deg': [10]*6}}}}
        soft, _ = resolve_tuning(entry, shared)
        c = Limits(**soft['left'])
        g = SoftLimitGovernor(c)
        # Absolute mapping: reversing outside still clips; returning inside
        # follows immediately, with no fault latch or slew delay.
        for i, (requested, expected) in enumerate(((20, 10), (15, 10), (9, 9),
                                                  (-20, -10), (-15, -10), (-9, -9))):
            q, v, _ = g.update(np.deg2rad([requested]*6),
                               state(np.zeros(6), np.zeros(6), i*.002), i*.002)
            np.testing.assert_allclose(q, np.deg2rad([expected]*6))
            np.testing.assert_array_equal(v, np.zeros(6))
            self.assertIsNone(g.fault)
        with self.assertRaisesRegex(ValueError, 'curvature'):
            Limits(**dict(soft['left'], reference_filter=True))

    def test_active_entrypoints_direct_follow_clip_and_reverse_in_one_cycle(self):
        import runpy
        from teleop_tuning import resolve_tuning
        shared=json.loads((ROOT/'soft-limits.json').read_text())
        entrypoints=[json.loads((ROOT/'one-gello-two-arms.json').read_text()),
                     runpy.run_path(str(ROOT/'运行参数.py'))['TELEOP_CONFIG']]
        for entry in entrypoints:
            soft,_=resolve_tuning(entry,shared)
            for side,data in soft.items():
                with self.subTest(side=side):
                    c=Limits(**data); self.assertFalse(c.reference_filter)
                    g=SoftLimitGovernor(c); actual=c.center.copy()
                    # Large target changes must not take seconds to traverse.
                    targets=[c.center, c.center+np.deg2rad(30), c.upper+1,
                             c.upper-np.deg2rad(1), c.lower-1, c.lower+np.deg2rad(1)]
                    for i,target in enumerate(targets):
                        q,v,_=g.update(target,state(actual,np.zeros(6),i*.002),i*.002)
                        np.testing.assert_array_equal(q,np.clip(target,c.lower,c.upper))
                        np.testing.assert_array_equal(v,np.zeros(6))
                        self.assertIsNone(g.fault)
                    bad=state(actual,np.zeros(6),.02); bad['fault_code_raw'][0]=1
                    with self.assertRaisesRegex(SoftLimitFault,'motor reports'):
                        g.update(c.center,bad,.02)

    def test_default_bounds_match_model(self):
        c = Limits(**config())
        root = ET.parse(ROOT/'source/host/Assets/PantheraHT.urdf').getroot()
        limits = [j.find('limit') for j in root.findall('joint') if j.get('type') == 'revolute']
        np.testing.assert_allclose(c.lower, [float(x.get('lower')) for x in limits], atol=1e-11)
        np.testing.assert_allclose(c.upper, [float(x.get('upper')) for x in limits], atol=1e-11)

    def test_invalid_configuration_rejected(self):
        cases = [('lower_deg', [0]*5), ('upper_deg', [0]*6),
                 ('velocity_deg_s', [float('nan')]*6), ('acceleration_deg_s2', [-1]*6),
                 ('exponent', 3),
                 ('max_period_ms', 0), ('feedback_age_ms', float('inf')),
                 ('velocity_deg_s', [30]*6)]
        for key, value in cases:
            with self.subTest(key=key), self.assertRaises(ValueError):
                data = config(); data[key] = value; Limits(**data)

    def test_outward_speed_vanishes_at_boundary_inward_is_available(self):
        g = SoftLimitGovernor(Limits(**config())); c = g.limits
        for q in (c.lower, c.upper):
            np.testing.assert_allclose(g.barrier(q, np.zeros(6)), 0, atol=1e-15)
        self.assertTrue(np.all(g.barrier(c.upper, c.speed) < 0))
        self.assertTrue(np.all(g.barrier(c.upper, -c.speed) >= -1e-15))

    def test_extreme_targets_both_boundaries_reversal_and_jitter(self):
        g = SoftLimitGovernor(Limits(**config())); c = g.limits
        q = c.center.copy(); v = np.zeros(6); now = 0.
        g.update(q, state(q, v, now), now)
        minimum = q.copy(); maximum = q.copy()
        for i in range(9000):
            dt = (.009, .011, .01)[i % 3]; now += dt
            target = np.full(6, 100.) if i < 3000 else np.full(6, -100.) if i < 6000 else c.center
            old_q, old_v = q.copy(), v.copy()
            q, v, diag = g.update(target, state(q, v, now), now)
            self.assertTrue(np.all(q >= c.lower-1e-10) and np.all(q <= c.upper+1e-10))
            self.assertTrue(np.all(np.abs(v) <= c.speed+1e-10))
            self.assertTrue(np.all(np.abs(v-old_v) <= c.acceleration*dt+1e-10))
            np.testing.assert_allclose(q-old_q, .5*(v+old_v)*dt, atol=1e-12)
            self.assertGreaterEqual(diag['min_barrier'], -1e-9)
            minimum = np.minimum(minimum, q); maximum = np.maximum(maximum, q)
        self.assertTrue(np.all(maximum > c.upper-np.deg2rad(1)))
        self.assertTrue(np.all(minimum < c.lower+np.deg2rad(1)))
        np.testing.assert_allclose(q, c.center, atol=np.deg2rad(.1))

    def test_start_on_limit_can_retreat_without_reset(self):
        for side in ('lower', 'upper'):
            g = SoftLimitGovernor(Limits(**config())); c = g.limits
            q = getattr(c, side).copy(); v = np.zeros(6)
            g.update(q, state(q, v, 0), 0)
            for i in range(1, 101):
                q, v, _ = g.update(c.center, state(q, v, i*.01), i*.01)
            self.assertTrue(np.all(q > c.lower) and np.all(q < c.upper))

    def test_zero_branch_tolerance_does_not_expand_command_bounds(self):
        g = SoftLimitGovernor(Limits(**config()))
        q = np.zeros(6); q[1] = np.deg2rad(-1)
        out, v, _ = g.update(q, state(q, np.zeros(6), 0), 0)
        self.assertEqual(out[1], 0)
        np.testing.assert_array_equal(v, np.zeros(6))

    def test_bad_feedback_and_frozen_udp_latch_fault(self):
        c = Limits(**config())
        cases = [('q_rad', [float('nan')]*6), ('dq_rad_s', [float('nan')]*6),
                 ('feedback_age_us', [20000]*6), ('feedback_age_us', [-1]*6),
                 ('fault_code_raw', [0, 0, 1, 0, 0, 0]), ('valid_q_mask', 31),
                 ('host_received_monotonic_s', -.1)]
        for key, value in cases:
            with self.subTest(key=key):
                g = SoftLimitGovernor(c); s = state(c.center, np.zeros(6), 0); s[key] = value
                with self.assertRaises(SoftLimitFault): g.update(c.center, s, 0)
                with self.assertRaises(SoftLimitFault): g.update(c.center, state(c.center, np.zeros(6), 0), 0)

    def test_board_age_and_host_age_are_added(self):
        g = SoftLimitGovernor(Limits(**config())); q = g.limits.center
        s = state(q, np.zeros(6), 0); s['feedback_age_us'] = [15000]*6
        with self.assertRaisesRegex(SoftLimitFault, 'stale'): g.update(q, s, .006)

    def test_j5_33_47_does_not_abort_but_reference_limits_remain(self):
        data=config();data['measured_velocity_deg_s']=[30]*6  # Saved legacy config.
        c=Limits(**data);g=SoftLimitGovernor(c);q=c.center.copy()
        measured=np.zeros(6);measured[4]=np.deg2rad(33.47)
        g.update(q,state(q,measured,0),0)
        target=q+np.deg2rad(1)
        out,v,_=g.update(target,state(q,measured,.01),.01)
        self.assertIsNone(g.fault)
        self.assertTrue(np.all(out>=c.lower) and np.all(out<=c.upper))
        self.assertTrue(np.all(np.abs(v)<=c.speed))
        self.assertTrue(np.all(np.abs(v)<=c.acceleration*.01+1e-10))

    def test_measured_velocity_still_restricts_outward_boundary_motion(self):
        c=Limits(**config());g=SoftLimitGovernor(c);q=c.center.copy()
        q[4]=c.upper[4]-np.deg2rad(.1)
        measured=np.zeros(6);measured[4]=np.deg2rad(33.47)
        g.update(q,state(q,measured,0),0)
        _,v,_=g.update(c.upper,state(q,measured,.01),.01)
        self.assertAlmostEqual(v[4],0.)

    def test_legacy_tracking_threshold_does_not_abort_compliant_following(self):
        data=config(); data['tracking_error_deg']=5
        c=Limits(**data); g=SoftLimitGovernor(c); actual=c.center.copy()
        target=actual+np.deg2rad(8)
        for i in range(301):
            command,velocity,_=g.update(target,state(actual,np.zeros(6),i*.01),i*.01)
            self.assertTrue(np.all(command>=c.lower) and np.all(command<=c.upper))
        self.assertTrue(np.all(command-actual>np.deg2rad(5)))
        self.assertIsNone(g.fault)
        bad=state(actual,np.zeros(6),3.01); bad['fault_code_raw'][0]=1
        with self.assertRaisesRegex(SoftLimitFault,'motor reports'):
            g.update(target,bad,3.01)

    def test_boundary_hold_and_reverse_without_restart_including_outside_feedback(self):
        for direction in (-1,1):
            for overshoot in (0,3):
                with self.subTest(direction=direction,overshoot=overshoot):
                    c=Limits(**config()); g=SoftLimitGovernor(c)
                    edge=c.upper if direction==1 else c.lower
                    actual=edge+direction*np.deg2rad(overshoot)
                    outward=edge+direction*np.deg2rad(30)
                    for i in range(101):
                        q,v,_=g.update(outward,state(actual,np.zeros(6),i*.01),i*.01)
                        np.testing.assert_allclose(q,edge)
                        np.testing.assert_allclose(v,0)
                    q,v,_=g.update(c.center,state(actual,np.zeros(6),1.01),1.01)
                    self.assertTrue(np.all(direction*(q-edge)<0))
                    self.assertTrue(np.all(direction*v<0))
                    self.assertIsNone(g.fault)

    def test_stalled_clock_and_long_period_reanchor_without_exit(self):
        for now in (0,-.01,.021,1):
            g=SoftLimitGovernor(Limits(**config())); c=g.limits; q=c.center.copy()
            g.update(q,state(q,np.zeros(6),0),0)
            g.v[:]=c.speed
            actual=q+np.deg2rad(1)
            out,v,_=g.update(c.upper,state(actual,np.zeros(6),now),now)
            np.testing.assert_allclose(out,actual)
            np.testing.assert_allclose(v,0)
            self.assertIsNone(g.fault)

    def test_infeasible_boundary_braking_stops_only_affected_axis_then_reverses(self):
        g=SoftLimitGovernor(Limits(**config())); c=g.limits
        q=c.center.copy(); q[0]=c.upper[0]-np.deg2rad(.1)
        g.update(q,state(q,np.zeros(6),0),0)
        g.v[0]=c.speed[0]
        out,v,_=g.update(c.upper,state(q,np.zeros(6),.002),.002)
        self.assertEqual(v[0],0)
        self.assertTrue(np.all(v[1:]>0))
        target=c.upper.copy(); target[0]=c.center[0]
        out,v,_=g.update(target,state(out,v,.004),.004)
        self.assertLess(v[0],0)
        self.assertTrue(np.all(v[1:]>0))
        self.assertIsNone(g.fault)

    def test_each_arm_actual_pose_changes_outward_filter(self):
        c = Limits(**config()); q = c.upper-np.deg2rad(1)
        near, far = SoftLimitGovernor(c), SoftLimitGovernor(c)
        for g in (near, far): g.update(q, state(q, np.zeros(6), 0), 0)
        _, vnear, _ = near.update(c.upper, state(c.upper, np.zeros(6), .01), .01)
        _, vfar, _ = far.update(c.upper, state(q, np.zeros(6), .01), .01)
        np.testing.assert_allclose(vnear, 0)
        self.assertTrue(np.all(vfar > 0))


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        from test_one_gello_two_arms import bimanual
        self.launcher = bimanual
        self.c = Limits(**config())

    def test_second_arm_rejected_before_either_send(self):
        sessions = {side: MagicMock() for side in ('left', 'right')}
        guards = {side: SoftLimitGovernor(self.c) for side in sessions}
        states = {side: state(self.c.center, np.zeros(6), 0) for side in sessions}
        states['right']['feedback_age_us'] = [100000]*6
        with self.assertRaises(SoftLimitFault):
            self.launcher.send_pair(sessions, states, {s:self.c.center for s in sessions},
                                    0, True, governors=guards, now=0)
        for s in sessions.values(): s.step.assert_not_called()

    def test_actual_wire_commands_use_filtered_q_and_velocity_with_encoder_offset(self):
        import arm_lab
        args = arm_lab.parser('impedance').parse_args([]); args.no_log = True
        session = arm_lab.Session(args, lambda q: np.zeros(6))
        session.active = True; session.robot = MagicMock()
        session.robot.send_arm.return_value = SimpleNamespace(ok=True, sequence=1)
        session.encoder_zero_rad[1] = -2*np.pi
        session.last_print = float('inf')
        guards = {'left': SoftLimitGovernor(self.c)}
        q = self.c.center.copy(); v = np.zeros(6)
        for i in range(2):
            sample = state(q, v, i*.002); sample['header'] = {'sequence':i}
            commands = self.launcher.send_pair({'left':session}, {'left':sample}, {'left':self.c.upper+1},
                                               i, True, governors=guards, now=i*.002)
            q, v, _ = commands['left']
            sent = session.robot.send_arm.call_args.args
            np.testing.assert_allclose(sent[0], q+session.encoder_zero_rad)
            np.testing.assert_allclose(sent[1], v)
        self.assertTrue(np.all(v > 0))

    def test_pair_stop_precedes_cleanup_for_all_exit_reasons(self):
        from contextlib import ExitStack
        for error in (None, RuntimeError('fault'), KeyboardInterrupt()):
            events = []
            sessions = {}
            for side in ('left', 'right'):
                s = MagicMock(); s.robot.lease = 1
                s.stop_homing_fault.side_effect = lambda side=side: events.append('stop '+side)
                sessions[side] = s
            try:
                with ExitStack() as stack, self.launcher.stop_pair_on_exit(sessions):
                    stack.callback(lambda: events.append('cleanup'))
                    if error is not None: raise error
            except BaseException as exc:
                self.assertIs(exc, error)
            self.assertEqual(events, ['stop left', 'stop right', 'cleanup'])
            for s in sessions.values(): self.assertFalse(s.active)

    def test_failed_stop_still_attempts_other_arm_and_suppresses_hold(self):
        sessions = {side: MagicMock() for side in ('left', 'right')}
        sessions['left'].stop_homing_fault.side_effect = RuntimeError('lost UDP')
        with patch('builtins.print'), self.launcher.stop_pair_on_exit(sessions): pass
        for s in sessions.values():
            s.stop_homing_fault.assert_called_once()
            self.assertFalse(s.active)
        self.assertIn('paired_stop_error', sessions['left'].summary.__setitem__.call_args.args)


if __name__ == '__main__':
    unittest.main()

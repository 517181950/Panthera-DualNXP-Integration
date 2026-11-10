"""Offline unilateral-wall checks. No network or actuator access."""
from pathlib import Path
import copy
import runpy
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch, MagicMock
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'source'/'host'))
from gravity_wall import GravityWall, WallSettings
from 限位试验参数 import WALL_SETTINGS


def make_wall():
    return GravityWall([-50]*6, [50]*6, WallSettings(**WALL_SETTINGS['right']))


class Tests(unittest.TestCase):
    def test_interior_is_pure_gravity(self):
        wall = make_wall()
        for speed in (-300, 0, 300):
            tau, diag = wall.command(np.zeros(6), np.deg2rad([speed]*6), 2)
            np.testing.assert_array_equal(tau, np.zeros(6))
            self.assertFalse(diag['active'].any())

    def test_at_boundary_static_force_points_inward(self):
        wall = make_wall()
        for sign in (-1, 1):
            tau, diag = wall.command(np.deg2rad([50*sign]*6), np.zeros(6), 2)
            self.assertTrue(np.all(tau*sign < 0))
            np.testing.assert_allclose(tau, -sign*wall.settings.max_torque_nm)
            self.assertFalse(diag['outside'].any())

    def test_boundary_effort_includes_gravity_and_brakes_rebound(self):
        wall = make_wall()
        limit = wall.settings.max_torque_nm
        for sign in (-1, 1):
            for angle in (50, 50.001, 55):
                for speed in (-50, 0, 50):
                    for gravity in (-limit*.8, limit*.8):
                        extra, diag = wall.command(np.deg2rad([sign*angle]*6),
                                                   np.deg2rad([speed]*6), 2, gravity)
                        expected = np.clip(-sign*limit-wall.settings.damping_nm_s_rad*np.deg2rad(speed),
                                           -limit, limit)
                        np.testing.assert_allclose(gravity+extra, expected)
                        np.testing.assert_allclose(diag['command_nm'], expected)

    def test_total_effort_cap_including_gravity_and_startup(self):
        wall = make_wall(); rng = np.random.default_rng(73)
        limit = wall.settings.max_torque_nm
        for _ in range(500):
            q = np.deg2rad(rng.uniform(-80, 80, 6))
            gravity = rng.uniform(-2, 2, 6)*limit
            extra, diag = wall.command(q, rng.uniform(-10, 10, 6),
                                       rng.uniform(0, 2), gravity)
            self.assertTrue(np.all(np.abs(diag['command_nm']) <= limit+1e-12))
            self.assertTrue(np.all(np.abs(diag['feedforward_nm']) <= limit+1e-12))
            np.testing.assert_allclose(gravity+extra, diag['command_nm'], atol=1e-12)

    def test_interior_preserves_gravity_and_boundary_is_continuous(self):
        wall = make_wall(); gravity = np.array([0, -7, 4, 1, .2, 0])
        extra, _ = wall.command(np.zeros(6), np.zeros(6), 2, gravity)
        np.testing.assert_array_equal(extra, np.zeros(6))
        for sign in (-1, 1):
            a, _ = wall.command(np.deg2rad([sign*(50-1e-7)]*6), np.zeros(6), 2, gravity)
            b, _ = wall.command(np.deg2rad([sign*(50+1e-7)]*6), np.zeros(6), 2, gravity)
            np.testing.assert_allclose(a, b, atol=1e-8)

    def test_configured_caps_match_current_model_effort(self):
        import xml.etree.ElementTree as ET
        model = ET.parse(ROOT/'source/host/Assets/PantheraHT_v4_hand620g.urdf')
        limits = [float(j.find('limit').get('effort')) for j in model.findall('joint')
                  if j.get('type') == 'revolute']
        for side in ('left', 'right'):
            np.testing.assert_array_equal(WALL_SETTINGS[side]['max_torque_nm'], limits)

    def test_fixed_angle_braking_increases_with_speed(self):
        wall = make_wall()
        q = np.deg2rad([46]*6)
        slow, _ = wall.command(q, np.deg2rad([1]*6), 2)
        fast, _ = wall.command(q, np.deg2rad([100]*6), 2)
        self.assertTrue(np.all(slow < 0))
        self.assertTrue(np.all(fast < slow))
        outside_zone, _ = wall.command(np.deg2rad([43]*6), np.deg2rad([100]*6), 2)
        np.testing.assert_array_equal(outside_zone, np.zeros(6))

    def test_damping_brakes_return_motion_without_sign_switch(self):
        wall = make_wall()
        for sign in (-1, 1):
            q = np.deg2rad([49*sign]*6)
            rest, _ = wall.command(q, np.zeros(6), 2)
            tau, diag = wall.command(q, np.deg2rad([-100*sign]*6), 2)
            self.assertTrue(np.all((tau-rest)*sign > 0))
            self.assertTrue(np.all(diag['damping_nm']*sign > 0))

    def test_spring_helps_return_and_does_not_latch(self):
        wall = make_wall()
        for sign in (-1, 1):
            tau, diag = wall.command(np.deg2rad([51*sign]*6), np.deg2rad([-10*sign]*6), 2)
            self.assertTrue(np.all(tau*sign < 0))
            self.assertTrue(diag['outside'].all())
            tau, _ = wall.command(np.zeros(6), np.zeros(6), 2)
            np.testing.assert_array_equal(tau, np.zeros(6))

    def test_force_cap_and_outward_dissipation_random_states(self):
        wall = make_wall()
        rng = np.random.default_rng(12)
        for _ in range(1000):
            sign = rng.choice([-1, 1], 6)
            q = np.deg2rad(sign*rng.uniform(38, 80, 6))
            dq = np.deg2rad(sign*rng.uniform(0, 500, 6))
            tau, _ = wall.command(q, dq, 2)
            self.assertTrue(np.all(np.abs(tau) <= wall.settings.max_torque_nm))
            self.assertTrue(np.all(tau*dq <= 0))

    def test_startup_ramp_has_zero_initial_force(self):
        wall = make_wall(); q = np.deg2rad([51]*6)
        a, _ = wall.command(q, np.zeros(6), 0)
        b, _ = wall.command(q, np.zeros(6), .5)
        c, _ = wall.command(q, np.zeros(6), 1)
        np.testing.assert_array_equal(a, np.zeros(6))
        np.testing.assert_allclose(b, c*.5)

    def test_torque_continuous_at_cushion_start(self):
        wall = make_wall()
        a, _ = wall.command(np.deg2rad([47-1e-8]*6), np.zeros(6), 2)
        b, _ = wall.command(np.deg2rad([47+1e-8]*6), np.zeros(6), 2)
        np.testing.assert_allclose(a, b, atol=1e-7)

    def test_angle_ramp_and_elapsed_time_independence_after_startup(self):
        wall = make_wall(); limit = wall.settings.max_torque_nm
        for sign in (-1, 1):
            for distance, fraction in ((3, 0), (1.5, .5), (0, 1), (-1, 1)):
                q = np.deg2rad([sign*(50-distance)]*6)
                for elapsed in (1, 2, 10, 1000):
                    torque, _ = wall.command(q, np.zeros(6), elapsed)
                    np.testing.assert_allclose(torque, -sign*fraction*limit, atol=1e-12)

    def test_damping_dissipates_for_both_directions_even_when_saturated(self):
        wall = make_wall(); rng = np.random.default_rng(41)
        for _ in range(500):
            q = np.deg2rad(rng.uniform(-55, 55, 6))
            dq = rng.uniform(-5, 5, 6)
            gravity = rng.uniform(-1, 1, 6)*wall.settings.max_torque_nm
            _, diag = wall.command(q, dq, rng.uniform(0, 2), gravity)
            self.assertTrue(np.all(diag['damping_nm']*dq <= 1e-12))

    def test_no_torque_jump_across_damping_zone_or_velocity_zero(self):
        wall = make_wall()
        for sign in (-1, 1):
            for edge in (45.5, 47, 50):
                a, _ = wall.command(np.deg2rad([sign*(edge-1e-8)]*6), np.ones(6), 2)
                b, _ = wall.command(np.deg2rad([sign*(edge+1e-8)]*6), np.ones(6), 2)
                np.testing.assert_allclose(a, b, atol=1e-6)
            a, _ = wall.command(np.deg2rad([49*sign]*6), np.full(6,-1e-9), 2)
            b, _ = wall.command(np.deg2rad([49*sign]*6), np.full(6,1e-9), 2)
            np.testing.assert_allclose(a, b, atol=1e-7)

    def test_slow_drag_is_lighter_but_fast_braking_is_preserved(self):
        wall = make_wall()
        for side in (-1, 1):
            q = np.deg2rad([49*side]*6)
            for direction in (-1, 1):
                _, slow = wall.command(q, np.deg2rad([direction*2]*6), 2)
                _, fast = wall.command(q, np.deg2rad([direction*20]*6), 2)
                self.assertTrue(np.all(slow['kd'] < .3*wall.settings.damping_nm_s_rad))
                np.testing.assert_allclose(fast['kd'], wall.settings.damping_nm_s_rad)
                self.assertTrue(np.all(slow['damping_nm']*direction <= 0))
            # The old 6-degree band resisted here; the new band is released.
            tau, diag = wall.command(np.deg2rad([45*side]*6), np.ones(6), 2)
            np.testing.assert_array_equal(tau, np.zeros(6))
            np.testing.assert_array_equal(diag['kd'], np.zeros(6))

    def test_speed_schedule_is_continuous_at_full_damping(self):
        wall = make_wall()
        for sign in (-1, 1):
            a, da = wall.command(np.deg2rad([49]*6), np.deg2rad([sign*(20-1e-7)]*6), 2)
            b, db = wall.command(np.deg2rad([49]*6), np.deg2rad([sign*(20+1e-7)]*6), 2)
            np.testing.assert_allclose(a, b, atol=1e-6)
            np.testing.assert_allclose(da['kd'], db['kd'], atol=1e-8)

    def test_invalid_settings_and_feedback_rejected(self):
        for key, value in [('damping_nm_s_rad',[0]*6), ('cushion_deg',[float('nan')]*6),
                           ('max_torque_nm',[-1]*6), ('startup_ramp_s',float('inf')),
                           ('startup_ramp_s',0), ('damping_zone_deg',[3]*6),
                           ('low_speed_damping_ratio',0), ('low_speed_damping_ratio',1.1),
                           ('low_speed_damping_ratio',float('nan')), ('full_damping_speed_deg_s',0)]:
            data = copy.deepcopy(WALL_SETTINGS['right']); data[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError): WallSettings(**data)
        with self.assertRaises(ValueError):
            GravityWall([-4]*6,[4]*6,WallSettings(**WALL_SETTINGS['right']))
        for q, v, t in (([float('nan')]*6,[0]*6,1),([0]*6,[float('inf')]*6,1),([0]*6,[0]*6,-1)):
            with self.assertRaises(ValueError): make_wall().command(q,v,t)

    def test_entry_uses_selected_bounds_and_motor_damping_without_homing(self):
        import gravity_wall_entry as entry
        upper = entry.TELEOP_CONFIG['boards']['right']['soft_limits']['upper_deg'][0]
        with tempfile.TemporaryDirectory() as folder:
            session = MagicMock()
            session.directory = Path(folder)
            session.summary = {}
            session.__enter__.return_value = session
            session.__exit__.return_value = False
            session.configure_encoder_turns.return_value = {'joints':[2]}
            session.state.side_effect = [dict(q_rad=np.deg2rad([upper,90,120,0,0,0]).tolist(),dq_rad_s=[0]*6), KeyboardInterrupt()]
            with patch.object(sys,'argv',['原臂重力补偿限位.py']), \
                 patch.object(entry,'Session',return_value=session), \
                 patch.object(entry,'Gravity',return_value=MagicMock(return_value=np.array([3,-7,4,1,.2,0]))), patch.object(entry,'wait_for_local_home') as preflight, \
                 patch.object(entry.time,'perf_counter',side_effect=[10,11]):
                entry.run('right')
            self.assertEqual(list(preflight.call_args.args[0]['boards']), ['right'])
            self.assertEqual(session.summary['soft_limits']['upper_deg'][0],upper)
            self.assertEqual(session.configure_encoder_turns.call_args.args[0],[2])
            args, kwargs = session.step.call_args
            np.testing.assert_array_equal(args[4], np.zeros(6))
            self.assertEqual(args[5][0], WALL_SETTINGS['right']['damping_nm_s_rad'][0]*WALL_SETTINGS['right']['low_speed_damping_ratio'])
            self.assertLess(kwargs['extra'][0], 0)
            self.assertAlmostEqual(kwargs['extra'][0]+3, -WALL_SETTINGS['right']['max_torque_nm'][0])
            session.move.assert_not_called()
            session.robot.send_hand.assert_not_called()
            session.__exit__.assert_called_once()

    def test_wall_torque_is_added_to_existing_gravity_in_real_session(self):
        from arm_lab import Session, parser
        args=parser('gravity').parse_args([]); args.no_log=True
        model=MagicMock(return_value=np.array([0,-7,4,1,.2,0]))
        s=Session(args,model); s.active=True; s.robot=MagicMock()
        s.robot.send_arm.return_value=SimpleNamespace(ok=True,sequence=1)
        q=np.deg2rad([50,50,-50,0,0,0]); torque,_=make_wall().command(q,np.zeros(6),2,model.return_value)
        s.step('test','gravity_wall',q,np.zeros(6),np.zeros(6),np.zeros(6),
               extra=torque,state={'q_rad':q.tolist(),'header':{'sequence':1}},pace=False)
        sent=s.robot.send_arm.call_args.args
        np.testing.assert_allclose(sent[4],model.return_value+torque)
        np.testing.assert_allclose(sent[4][:3],[-21,-36,36])
        np.testing.assert_array_equal(sent[2],np.zeros(6))
        np.testing.assert_array_equal(sent[3],np.zeros(6))

    def test_native_damping_is_sent_once_and_reacts_to_current_motor_velocity(self):
        from arm_lab import Session, parser
        args=parser('gravity').parse_args([]); args.no_log=True
        gravity=np.array([0,-7,4,1,.2,0])
        s=Session(args,MagicMock(return_value=gravity)); s.active=True; s.robot=MagicMock()
        s.robot.send_arm.return_value=SimpleNamespace(ok=True,sequence=1)
        q=np.deg2rad([49]*6); dq=np.deg2rad([-10]*6)
        _,diag=make_wall().command(q,dq,2,gravity)
        s.step('test','gravity_wall',q,np.zeros(6),np.zeros(6),diag['kd'],
               extra=diag['feedforward_nm']-gravity,
               state={'q_rad':q.tolist(),'header':{'sequence':1}},pace=False)
        sent=s.robot.send_arm.call_args.args
        np.testing.assert_allclose(sent[4]-sent[3]*dq,diag['command_nm'])
        # At a faster rebound the motor adds braking without a host update.
        self.assertTrue(np.all(sent[4]-sent[3]*(dq-.1)>diag['command_nm']))


if __name__ == '__main__':
    unittest.main()

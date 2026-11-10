"""Independent tuning, real command arguments and passive feedback checks."""
import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teleop_tuning import HandTuning, HandTargetFilter, resolve_tuning
from hand_slider_demo import gripper_targets_deg
ROOT=Path(__file__).resolve().parents[3]


class Tests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads((ROOT/'one-gello-two-arms.json').read_text())
        self.shared=json.loads((ROOT/'soft-limits.json').read_text())

    def test_default_resolution_preserves_previous_settings(self):
        for board in self.config['boards'].values():
            board['soft_limits'] = {}
        soft,hands=resolve_tuning(self.config,self.shared)
        for side in ('left','right'):
            self.assertEqual(soft[side],self.shared)
            self.assertEqual(hands[side].current_ma,500)
            self.assertIsNone(hands[side].max_target_speed_deg_s)
            for percent in np.linspace(0,100,101):
                np.testing.assert_allclose(hands[side].target(percent),
                    np.deg2rad(gripper_targets_deg(percent,'five_finger')))

    def test_left_arm_override_does_not_change_right_or_shared(self):
        expected_right=resolve_tuning(self.config,self.shared)[0]['right']
        self.config['boards']['left']['soft_limits']={'velocity_deg_s':[10]*6,'prediction_ms':10}
        soft,_=resolve_tuning(self.config,self.shared)
        self.assertEqual(soft['left']['velocity_deg_s'],[10]*6)
        self.assertEqual(soft['left']['prediction_ms'],10)
        self.assertEqual(soft['right'],expected_right)
        soft['left']['upper_deg'][0]=0
        self.assertEqual(soft['right']['upper_deg'][0],expected_right['upper_deg'][0])
        self.assertEqual(self.shared['upper_deg'][0],155)

    def test_invalid_override_or_typo_rejected(self):
        for overrides in ({'exponent':3},{'velocty_deg_s':[10]*6},None):
            self.config['boards']['right']['soft_limits']=overrides
            with self.assertRaises((ValueError,TypeError)):resolve_tuning(self.config,self.shared)

    def test_legacy_config_inherits_defaults(self):
        for board in self.config['boards'].values():
            board.pop('soft_limits',None);board.pop('hand_control',None)
        soft,hands=resolve_tuning(self.config,self.shared)
        self.assertEqual(soft['left'],soft['right'])
        self.assertEqual(hands['left'].document(),HandTuning().document())

    def test_hand_mapping_and_current_are_independent(self):
        self.config['boards']['left']['hand_control'].update(current_ma=300,joint_scale=[.5]*6)
        _,hands=resolve_tuning(self.config,self.shared)
        self.assertEqual(hands['left'].current_ma,300)
        self.assertEqual(hands['right'].current_ma,500)
        np.testing.assert_allclose(hands['left'].target(100),hands['right'].target(100)/2)

    def test_grasp_transform_order_and_joint_offset(self):
        tuning=HandTuning(grasp_inverted=True,grasp_scale=.5,grasp_offset_percent=10,
                          joint_scale=[.5]*6,joint_offset_deg=[1]*6)
        # 20 -> inverted 80 -> scaled 40 -> offset 50 -> joint transform.
        np.testing.assert_allclose(np.rad2deg(tuning.target(20)),
                                   np.asarray(gripper_targets_deg(50,'five_finger'))*.5+1)

    def test_invalid_hand_settings_rejected(self):
        for params in ({'current_ma':-1},{'current_ma':65536},{'current_ma':1.5},
                       {'profile':'unknown'},{'grasp_scale':0},{'grasp_inverted':1},
                       {'joint_scale':[1]*5},{'joint_offset_deg':[-1]*6},
                       {'max_target_speed_deg_s':[0]*6},{'feedback_timeout_ms':float('nan')}):
            with self.subTest(params=params),self.assertRaises(ValueError):HandTuning(**params)

    def test_optional_speed_limits_are_per_hand_and_per_channel(self):
        slow=HandTargetFilter(HandTuning(max_target_speed_deg_s=[10,20,30,40,50,60]))
        free=HandTargetFilter(HandTuning())
        desired=np.deg2rad([60]*6); actual=np.zeros(6)
        np.testing.assert_allclose(free.update(desired,actual,1),desired)
        first=np.array(slow.update(desired,actual,1))
        np.testing.assert_allclose(np.rad2deg(first),np.array([10,20,30,40,50,60])*.02)
        second=np.array(slow.update(desired,actual,1.04))
        np.testing.assert_allclose(np.rad2deg(second-first),np.array([10,20,30,40,50,60])*.04)
        reverse=np.array(slow.update(np.zeros(6),actual,1.06))
        self.assertTrue(np.all(reverse<second))

    def test_limiter_timeout_rejects_a_large_restart_jump(self):
        filt=HandTargetFilter(HandTuning(max_target_speed_deg_s=[10]*6))
        filt.update([1]*6,[0]*6,0)
        with self.assertRaisesRegex(RuntimeError,'timed out'):filt.update([1]*6,[0]*6,1)


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        from test_one_gello_two_arms import bimanual
        self.launcher=bimanual

    def test_current_reaches_each_boards_configuration(self):
        for current in (300,500):
            session=MagicMock()
            session.robot.start_state_stream.return_value={'status':0}
            session.robot.prepare_hand_mode.return_value={'status':0}
            session.robot.configure.return_value={'status':0}
            with patch.object(self.launcher,'checked_hand_feedback'):
                self.launcher.prepare_hand(session,HandTuning(current_ma=current))
            kwargs=session.robot.configure.call_args.kwargs
            self.assertEqual(kwargs['hand_mode'],3)
            self.assertEqual(kwargs['hand_soft_current_ma'],[current]*6)
            self.assertEqual(session.robot.start_state_stream.call_args.kwargs['motor_query_hz'],500)

    def test_distinct_hand_targets_reach_real_send_api(self):
        sessions={side:MagicMock() for side in ('left','right')}
        tunings={'left':HandTuning(joint_scale=[.5]*6), 'right':HandTuning()}
        desired={side:t.target(100) for side,t in tunings.items()}
        feedback={side:{'q_rad':[0]*6} for side in sessions}
        filters={side:HandTargetFilter(t) for side,t in tunings.items()}
        self.launcher.send_hand_pair(sessions,feedback,desired,filters,0)
        for side,s in sessions.items():
            np.testing.assert_allclose(s.robot.send_hand.call_args.args[0],desired[side])

    def test_second_hand_filter_failure_sends_neither_hand(self):
        sessions={side:MagicMock() for side in ('left','right')}
        filters={side:HandTargetFilter(HandTuning()) for side in sessions}
        feedback={side:{'q_rad':[0]*6} for side in sessions}
        with self.assertRaises(ValueError):
            self.launcher.send_hand_pair(sessions,feedback,{'left':[0]*6,'right':[float('nan')]*6},filters,0)
        for s in sessions.values():s.robot.send_hand.assert_not_called()

    def test_hand_feedback_counts_both_host_and_board_age(self):
        session=MagicMock()
        raw=dict(joint_q_rad=[0]*11,valid_fields=2,online=True,
                 field_age_us=[0,70000,0,0,0],host_received_monotonic_s=1)
        session.robot.snapshot.return_value={'HandState':raw}
        with patch.object(self.launcher.time,'perf_counter',return_value=1.04):
            with self.assertRaisesRegex(RuntimeError,'stale'):
                self.launcher.checked_hand_feedback(session,HandTuning(feedback_timeout_ms=100))
            h=self.launcher.checked_hand_feedback(session,HandTuning(feedback_timeout_ms=120))
        self.assertAlmostEqual(h['angle_age_ms'],110)

    def test_mapping_never_applied_to_feedback_hold(self):
        # On master loss, the launcher's stored actual-pose target enters the
        # slew filter directly; per-finger offset must not be applied a second time.
        tuning=HandTuning(joint_offset_deg=[10]*6)
        actual=np.deg2rad([15]*6)
        output=HandTargetFilter(tuning).update(actual,actual,1)
        np.testing.assert_allclose(output,actual)


if __name__=='__main__':unittest.main()

"""Shared-master fanout uses one clock, independent gains and local feedback."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import threading
import time
from unittest.mock import MagicMock,patch
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('bimanual',ROOT/'run-one-gello-two-arms.py')
bimanual=importlib.util.module_from_spec(spec);spec.loader.exec_module(bimanual)
import arm_lab
from test_gello_home_zero import Clock,Session as HomeSession


class Tests(unittest.TestCase):
    def homing_sessions(self):
        sessions={side:MagicMock() for side in ('left','right')}
        for s in sessions.values():
            s.args.kp=[4]*6;s.args.kd=[.5]*6
            s.state.return_value={'control_owner':0,'q_rad':[0]*6}
        config={'boards':{side:{'homing_gains':'follow'} for side in sessions}}
        return sessions,config

    def test_first_homed_arm_is_serviced_while_peer_homes(self):
        sessions,config=self.homing_sessions();holding=threading.Event()
        sessions['left'].step.side_effect=lambda *args,**kwargs:holding.set()
        def home(proxy,gains):
            if proxy.session is sessions['right']:
                if not holding.wait(1):raise RuntimeError('first arm was not serviced')
        with patch.object(bimanual.teleop,'home_zero',side_effect=home):
            bimanual.home_sessions(sessions,config)
        self.assertTrue(holding.is_set())
        self.assertEqual(sessions['left'].step.call_args.args[1],'wait_peer_hold')

    def test_homing_failure_cancels_peer_and_joins_workers(self):
        sessions,config=self.homing_sessions();both=threading.Barrier(2)
        def home(proxy,gains):
            both.wait(timeout=1)
            if proxy.session is sessions['left']:raise RuntimeError('collision tracking')
            while True:
                proxy.step('home','raise',[0]*6,[0]*6,[1]*6,[1]*6)
                time.sleep(.001)
        with patch.object(bimanual.teleop,'home_zero',side_effect=home):
            with self.assertRaisesRegex(RuntimeError,'left: collision tracking'):
                bimanual.home_sessions(sessions,config)
        self.assertFalse(any(t.name in ('home-left','home-right') for t in threading.enumerate()))

    def test_local_home_owner_ends_active_host_session(self):
        from types import SimpleNamespace
        from arm_lab import Session
        s=Session.__new__(Session);s.active=True
        s.robot=SimpleNamespace(lease=123,snapshot=lambda *names:{'ArmState':{'control_owner':2}})
        with self.assertRaisesRegex(RuntimeError,'NXP'):s.state()
        self.assertFalse(s.active);self.assertEqual(s.robot.lease,0)

    def test_same_target_distinct_gains_local_gravity_no_per_arm_sleep(self):
        sessions={};states={};target=np.array([.1,.2,.3,.4,.5,.6])
        for side,q,gain in [('left',.25,2.),('right',.5,12.24)]:
            args=arm_lab.parser('impedance').parse_args([]);args.no_log=True;args.kp=[gain]*6;args.kd=[gain/10]*6
            model=MagicMock(side_effect=lambda pose:np.array(pose)*3)
            s=arm_lab.Session(args,model);s.active=True;s.robot=MagicMock()
            s.robot.send_arm.return_value=SimpleNamespace(ok=True,sequence=123)
            state=dict(q_rad=[q]*6,header={'sequence':1});sessions[side]=s;states[side]=state
        with patch.object(arm_lab.time,'sleep') as sleep:
            bimanual.send_pair(sessions,states,{n:target for n in sessions},1,True)
            sleep.assert_not_called()
        for side,s in sessions.items():
            sent=s.robot.send_arm.call_args.args
            np.testing.assert_allclose(sent[0],target)
            self.assertEqual(sent[2],s.args.kp);self.assertEqual(sent[3],s.args.kd)
            np.testing.assert_allclose(sent[4],np.array(states[side]['q_rad'])*3)
            self.assertEqual(s.last_command_sequence,123)

    def test_master_loss_holds_each_arm_own_actual_position(self):
        states={'left':{'q_rad':[.1]*6},'right':{'q_rad':[.8]*6}}
        holds={n:bimanual.teleop.HoldOnLoss() for n in states}
        targets=bimanual.targets_for(None,states,holds)
        np.testing.assert_allclose(targets['left'],[.1]*6);np.testing.assert_allclose(targets['right'],[.8]*6)
        changed={'left':{'q_rad':[.3]*6},'right':{'q_rad':[.9]*6}}
        targets=bimanual.targets_for(None,changed,holds)
        np.testing.assert_allclose(targets['left'],[.1]*6);np.testing.assert_allclose(targets['right'],[.8]*6)
        desired=np.array([.2]*6);targets=bimanual.targets_for(desired,changed,holds)
        for target in targets.values():np.testing.assert_allclose(target,desired)

    def test_new_arm_homing_never_uses_old_high_gains(self):
        clock=Clock();s=HomeSession(clock)
        with patch.object(bimanual.teleop,'time',clock):
            bimanual.teleop.home_zero(s,gains=(s.args.kp,s.args.kd))
        self.assertEqual(s.summary['homing']['status'],'complete')
        for command in s.commands:np.testing.assert_allclose(command[3],s.args.kp)

    def test_hand_mode_failure_restores_arm_queries_and_propagates(self):
        s=MagicMock();s.robot.start_state_stream.return_value={'status':0}
        s.robot.prepare_hand_mode.return_value={'status':-110}
        with self.assertRaises(RuntimeError):bimanual.prepare_hand(s)
        self.assertEqual([c.kwargs['motor_query_hz'] for c in s.robot.start_state_stream.call_args_list],[500,500])
        s.robot.send_arm.assert_not_called();s.robot.send_hand.assert_not_called()

    def test_missing_hand_aborts_before_mode_or_arm_takeover_without_stop(self):
        sessions={side:MagicMock(active=False,sent=0) for side in ('left','right')}
        config={'boards':{side:{'hand':True} for side in sessions}}
        with patch.object(bimanual,'checked_hand_feedback',side_effect=RuntimeError('no angles')), \
                patch.object(bimanual,'prepare_hand') as prepare:
            with self.assertRaisesRegex(RuntimeError,'left:.*right:'):
                with bimanual.stop_pair_on_exit(sessions):
                    bimanual.prepare_startup_hands(sessions,config,dict.fromkeys(sessions),timeout=0)
        prepare.assert_not_called()
        for s in sessions.values():
            s.step.assert_not_called();s.stop_homing_fault.assert_not_called()

    def test_second_hand_mode_failure_does_not_stop_either_arm(self):
        sessions={side:MagicMock(active=False,sent=0) for side in ('left','right')}
        config={'boards':{side:{'hand':True} for side in sessions}}
        with patch.object(bimanual,'checked_hand_feedback'), \
                patch.object(bimanual,'prepare_hand',side_effect=[None,RuntimeError('mode timeout')]):
            with self.assertRaisesRegex(RuntimeError,'right:.*尚未接管'):
                with bimanual.stop_pair_on_exit(sessions):
                    bimanual.prepare_startup_hands(sessions,config,dict.fromkeys(sessions))
        for s in sessions.values():
            s.step.assert_not_called();s.stop_homing_fault.assert_not_called()

    def optional_hands(self):
        sessions={side:MagicMock(active=False,sent=0) for side in ('left','right')}
        for s in sessions.values():s.robot.operation.return_value={'status':0}
        config={'require_hands':False,'boards':{side:{'hand':True} for side in sessions}}
        return sessions,config

    def test_both_missing_hands_skip_without_blocking_arms(self):
        sessions,config=self.optional_hands()
        with patch.object(bimanual,'checked_hand_feedback',side_effect=RuntimeError('offline')), \
                patch.object(bimanual,'prepare_hand') as prepare:
            bimanual.prepare_startup_hands(sessions,config,dict.fromkeys(sessions),timeout=0)
        prepare.assert_not_called()
        for side,s in sessions.items():
            self.assertFalse(config['boards'][side]['hand'])
            self.assertTrue(s.args.no_hand)
            s.robot.operation.assert_called_once_with(16,target_mask=0)
            s.step.assert_not_called();s.robot.send_hand.assert_not_called()

    def test_one_missing_hand_leaves_other_hand_enabled(self):
        sessions,config=self.optional_hands()
        def feedback(s,tuning):
            if s is sessions['left']:raise RuntimeError('offline')
            return {}
        with patch.object(bimanual,'checked_hand_feedback',side_effect=feedback), \
                patch.object(bimanual,'prepare_hand') as prepare:
            bimanual.prepare_startup_hands(sessions,config,dict.fromkeys(sessions),timeout=0)
        self.assertFalse(config['boards']['left']['hand'])
        self.assertTrue(config['boards']['right']['hand'])
        prepare.assert_called_once_with(sessions['right'],None)
        sessions['right'].robot.operation.assert_not_called()

    def test_mode_timeout_skips_only_that_hand(self):
        sessions,config=self.optional_hands()
        with patch.object(bimanual,'checked_hand_feedback'), \
                patch.object(bimanual,'prepare_hand',side_effect=[None,RuntimeError('mode timeout')]):
            bimanual.prepare_startup_hands(sessions,config,dict.fromkeys(sessions),timeout=0)
        self.assertTrue(config['boards']['left']['hand'])
        self.assertFalse(config['boards']['right']['hand'])
        for s in sessions.values():s.step.assert_not_called()

    def test_skip_rejects_failed_bus_disable_or_bad_arm_feedback(self):
        for failure in ('disable','arm'):
            sessions,config=self.optional_hands()
            if failure=='disable':sessions['left'].robot.operation.return_value={'status':-1}
            else:sessions['left'].state.side_effect=RuntimeError('arm stale')
            with patch.object(bimanual,'checked_hand_feedback',side_effect=RuntimeError('offline')):
                with self.assertRaises(RuntimeError):
                    bimanual.prepare_startup_hands(sessions,config,dict.fromkeys(sessions),timeout=0)
            for s in sessions.values():s.step.assert_not_called()

    def test_sent_target_before_activation_failure_still_stops_both(self):
        sessions={side:MagicMock(active=False,sent=0) for side in ('left','right')}
        sessions['left'].sent=1
        with bimanual.stop_pair_on_exit(sessions):pass
        for s in sessions.values():s.stop_homing_fault.assert_called_once()

    def test_failed_preflight_release_does_not_send_arm_or_stop_operations(self):
        s,_=self.make_encoder_session(0)
        s.robot.lease=1
        with self.assertRaisesRegex(RuntimeError,'preflight'):
            try:
                with bimanual.stop_pair_on_exit({'left':s}):
                    raise RuntimeError('preflight')
            finally:
                s.__exit__(RuntimeError,RuntimeError('preflight'),None)
        self.assertEqual([call.args[0] for call in s.robot.operation.call_args_list],[4,2])
        s.robot.send_arm.assert_not_called();s.robot.send_hand.assert_not_called()

    def make_encoder_session(self,raw_j2_deg):
        import copy
        a=arm_lab.parser('impedance').parse_args([]);a.no_log=True
        s=arm_lab.Session(a,lambda q:np.zeros(6));s.robot=MagicMock()
        state=dict(online_mask=63,valid_q_mask=63,valid_dq_mask=63,valid_tau_mask=63,
                   feedback_age_us=[0]*6,q_rad=[0,np.deg2rad(raw_j2_deg),0,0,0,0],
                   dq_rad_s=[0]*6,tau_measured_nm=[0]*6,header={'sequence':1})
        s.robot.snapshot.side_effect=lambda *names:{'ArmState':copy.deepcopy(state)}
        s.robot.send_arm.return_value=SimpleNamespace(ok=True,sequence=1)
        s.robot.operation.return_value={'status':0};s.robot.stop_state_stream.return_value={'status':0}
        return s,state

    def test_minus_361_homes_one_degree_not_361_degrees(self):
        s,state=self.make_encoder_session(-361)
        ref=s.configure_encoder_turns([2])
        self.assertAlmostEqual(ref['logical_start_deg'][1],-1)
        self.assertAlmostEqual(ref['raw_zero_deg'][1],-360)
        with patch.object(arm_lab.time,'sleep'):
            s.step('test','home_zero',np.zeros(6),np.zeros(6),s.args.kp,s.args.kd,pace=False)
        self.assertAlmostEqual(s.robot.send_arm.call_args.args[0][1],-2*np.pi)

    def test_freshness_includes_host_receipt_age_and_motor_faults(self):
        _,state=self.make_encoder_session(0)
        with patch.object(arm_lab.time,'perf_counter',return_value=1000.):
            state['host_received_monotonic_s']=999.990
            self.assertTrue(arm_lab.fresh(state))
            state['host_received_monotonic_s']=999.979
            self.assertFalse(arm_lab.fresh(state))
            state['host_received_monotonic_s']=1000.001
            self.assertFalse(arm_lab.fresh(state))
            state['host_received_monotonic_s']=1000.
            state['fault_code_raw']=[0,1,0,0,0,0]
            self.assertFalse(arm_lab.fresh(state))

    def test_turn_branch_is_independent_for_the_two_boards(self):
        left,_=self.make_encoder_session(-270);right,_=self.make_encoder_session(5)
        left.configure_encoder_turns([2]);right.configure_encoder_turns([2])
        self.assertAlmostEqual(np.rad2deg(left.state()['q_rad'][1]),90)
        self.assertAlmostEqual(np.rad2deg(right.state()['q_rad'][1]),5)
        self.assertAlmostEqual(left.encoder_zero_rad[1],-2*np.pi)
        self.assertEqual(right.encoder_zero_rad[1],0)

    def test_exit_keeps_motor_feedback_for_board_soft_hold(self):
        s,state=self.make_encoder_session(0);s.active=True;s.robot.lease=1
        s.__exit__(None,None,None)
        s.robot.operation.assert_called_once_with(18)
        s.robot.stop_state_stream.assert_not_called()
        s.robot.close.assert_called_once()
        self.assertTrue(s.summary['motor_feedback_queries_preserved'])

    def test_midrun_turn_jump_aborts_and_defers_hold_to_board(self):
        s,state=self.make_encoder_session(-361);s.configure_encoder_turns([2]);s.active=True;s.robot.lease=1
        state['q_rad'][1]=np.deg2rad(-1)
        with self.assertRaisesRegex(RuntimeError,'Encoder turn jumped'):s.state()
        with patch.object(arm_lab.time,'sleep'):s.__exit__(RuntimeError,RuntimeError('turn jump'),None)
        s.robot.send_arm.assert_not_called()
        s.robot.operation.assert_called_once_with(18)
        self.assertEqual(s.summary['cleanup_errors'],[])

    def test_other_joint_turns_are_accepted_and_rebased_after_homing(self):
        s,state=self.make_encoder_session(0);state['q_rad'][0]=2*np.pi
        s.configure_encoder_turns([2])
        s.rebase_home_zero([2*np.pi,0,0,0,0,0])
        self.assertAlmostEqual(s.state()['q_rad'][0],0.)
        s.active=True
        with patch.object(arm_lab.time,'sleep'):
            s.step('test','follow_zero',np.zeros(6),np.zeros(6),s.args.kp,s.args.kd,pace=False)
        self.assertAlmostEqual(s.robot.send_arm.call_args.args[0][0],2*np.pi)

    def test_board_arguments_keep_models_and_ports_separate(self):
        config=bimanual.read_json(ROOT/'one-gello-two-arms.json')
        a=bimanual.session_args(config,'left',Path('/tmp/offline'))
        b=bimanual.session_args(config,'right',Path('/tmp/offline'))
        self.assertEqual(a.kp,[4,10,10,2,2,1]);self.assertEqual(b.kp,[24.48,61.2,61.2,12.24,12.24,12.24])
        self.assertEqual(a.board_node,11);self.assertEqual(b.board_node,12)
        self.assertTrue(set(a.ports).isdisjoint(b.ports))
        self.assertNotEqual(a.output,b.output);self.assertFalse(a.no_hand);self.assertFalse(b.no_hand)

if __name__=='__main__':unittest.main()

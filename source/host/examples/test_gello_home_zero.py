"""Offline startup/motion tests. No hardware or serial access."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch, MagicMock
import numpy as np

spec=importlib.util.spec_from_file_location('gello_teleop',Path(__file__).with_name('18_gello-low-impedance.py'))
teleop=importlib.util.module_from_spec(spec)
spec.loader.exec_module(teleop)


class Clock:
    def __init__(self):self.t=0.
    def perf_counter(self):return self.t
    def sleep(self,seconds):self.t+=seconds


class Session:
    def __init__(self,clock,stuck=False):
        self.clock=clock;self.stuck=stuck
        self.q=np.deg2rad([20,40,60,-20,10,-90]);self.v=np.zeros(6)
        self.args=SimpleNamespace(kp=[4,10,10,2,2,1],kd=[.5,.8,.8,.2,.2,.1])
        self.summary={};self.commands=[]
    def __enter__(self):return self
    def __exit__(self,*args):pass
    def stop_homing_fault(self):self.summary['stop_requested']=True
    def rebase_home_zero(self,shift):self.summary["zero_shift"]=np.asarray(shift).tolist();self.q=self.q-shift
    def state(self):return dict(q_rad=self.q.tolist(),dq_rad_s=self.v.tolist())
    def step(self,point,phase,q,v,kp,kd,**kwargs):
        self.commands.append((phase,np.array(q),np.array(v),np.array(kp)))
        if not self.stuck:self.q=np.array(q);self.v=np.array(v)
        self.clock.t+=.002
        return self.state()


class Tests(unittest.TestCase):
    def test_single_tracking_threshold_and_joint_specific_range(self):
        check=teleop.home_path.Tracking()
        for now in (0.,.1,1.):
            self.assertTrue(check.update(np.deg2rad([0,0,4,0,0,0]),np.zeros(6),now))
        self.assertFalse(check.update(np.deg2rad([0,0,5.1,0,0,0]),np.zeros(6),1.002))
        self.assertTrue(teleop.home_path.start_ok([0,0,4,0,0,0]))
        self.assertTrue(teleop.home_path.start_ok([0,0,4.3,0,0,0]))
        self.assertFalse(teleop.home_path.start_ok([0,0,float("nan"),0,0,0]))

    def test_board_fault_stop_releases_lease_and_suppresses_exit_hold(self):
        session=teleop.Session.__new__(teleop.Session)
        session.robot=MagicMock();session.robot.operation.return_value={'status':0}
        session.active=True;session.summary={}
        session.stop_homing_fault()
        session.robot.operation.assert_called_once_with(18)
        self.assertFalse(session.active)
        self.assertEqual(session.robot.lease,0)
        session.robot.send_arm.assert_not_called()

    def test_failed_soft_hold_does_not_send_exit_targets(self):
        session=teleop.Session.__new__(teleop.Session)
        session.robot=MagicMock();session.robot.operation.side_effect=[{'status':-5},{'status':0}]
        session.active=True;session.summary={}
        with self.assertRaisesRegex(RuntimeError,'柔顺保持未确认'):session.stop_homing_fault()
        self.assertEqual([c.args[0] for c in session.robot.operation.call_args_list],[18])
        self.assertEqual(session.robot.lease,0);self.assertFalse(session.active)
        session.robot.send_arm.assert_not_called()

    def test_negative_J2_branch_homes_to_nearest_upright_then_rebases(self):
        clock=Clock();session=Session(clock);session.q[1]=np.deg2rad(-164.18)
        session.q[3]=np.deg2rad(93.47)
        with patch.object(teleop,'time',clock):teleop.home_zero(session)
        raising=[r for r in session.commands if r[0]=='home_raise']
        self.assertAlmostEqual(raising[0][1][1],np.deg2rad(-164.18))
        self.assertAlmostEqual(raising[-1][1][1],np.deg2rad(-270))
        self.assertAlmostEqual(session.summary['zero_shift'][1],-2*np.pi)
        np.testing.assert_allclose(session.q,np.zeros(6),atol=1e-12)
        self.assertEqual(session.summary['homing']['status'],'complete')

    def test_nonfinite_start_rejected_before_command(self):
        clock=Clock();session=Session(clock);session.q[1]=np.nan
        with patch.object(teleop,'time',clock),self.assertRaisesRegex(RuntimeError,'有效位置'):
            teleop.home_zero(session)
        self.assertEqual(session.commands,[])
        self.assertTrue(session.summary['stop_requested'])

    def test_saved_zero_does_not_rezero_a_bent_startup_pose(self):
        config=dict(zero_raw_ticks=[1000]*6,axes=[dict(sign=1,scale=1,offset_rad=0)]*6)
        mapping=teleop.FixedGelloZero(config)
        expected=np.array(teleop.SERVO_SIGNS[:6])*200*2*np.pi/4096
        np.testing.assert_allclose(mapping.target([1200]*6),expected)
        np.testing.assert_allclose(mapping.target([1000]*6),np.zeros(6))
        before=mapping.previous_delta.copy()
        self.assertIsNone(mapping.target([np.nan]*6))
        np.testing.assert_array_equal(mapping.previous_delta,before)

    def test_saved_zero_preserves_turns_after_wrap_and_reader_restart(self):
        config=dict(zero_raw_ticks=[4000]*6,axes=[dict(sign=1,scale=1,offset_rad=0)]*6)
        mapping=teleop.FixedGelloZero(config)
        for delta in (0,200,1000,2000,2500,3000,2500,2000,1000,200,0):
            ticks=[(4000+delta)%4096]*6
            np.testing.assert_allclose(mapping.target(ticks),np.array(teleop.SERVO_SIGNS[:6])*delta*2*np.pi/4096)

    def test_fixed_zero_waits_for_reference_before_following(self):
        clock=Clock();session=Session(clock);origin=session.q.copy();reader=MagicMock()
        reader.latest.return_value=None;reader.is_connected=True
        config=teleop.read_json(teleop.HERE/'profiles/gello_selected.json')
        config['left']['zero_raw_ticks']=[1000]*6
        calls=[1200,1000,900,900,900,900]
        def values(*args,**kwargs):
            tick=calls.pop(0) if calls else 900
            return dict(reader_deg=np.zeros(6),ticks=np.full(6,tick),raw_deg=np.zeros(6),velocity_deg_s=np.zeros(6))
        argv=['teleop','--execute','--group','0','--no-hand','--seconds','.01']
        with patch.object(teleop,'time',clock),patch.object(teleop.sys,'argv',argv),patch.object(teleop,'Session',return_value=session),patch.object(teleop,'GelloReader',return_value=reader),patch.object(teleop,'gello_values',side_effect=values),patch.object(teleop,'read_json',return_value=config):
            self.assertEqual(teleop.main(),0)
        self.assertEqual(session.commands[0][0],'gello_lost_hold')
        np.testing.assert_allclose(session.commands[0][1],origin)
        self.assertEqual(session.commands[1][0],'gello_follow')
        np.testing.assert_allclose(session.commands[1][1],np.zeros(6))
        self.assertGreater(np.max(np.abs(session.commands[2][1])),0)

    def test_trigger_rollover_at_either_starting_endpoint(self):
        # A trigger travel can cross 4095/0. Store continuous endpoints,
        # and align that interval to the reader's initial encoder turn.
        for initial in (-900,110):
            reader=teleop.GelloReader('offline',group_mask=1)
            with patch.object(reader,'_read_position',return_value=initial%4096):
                reader._read_stable_position(0,6)
            reader.set_gripper_calibration(0,-900,110)
            reader._apply_commands(reader._drain_commands(),{0:[0]*6+[initial%4096]})
            reader._sample_times[7]=1.
            for tick in (-900,-500,-1,0,50,110,-1,-900):
                raw=tick%4096
                with patch.object(reader,'_read_position',return_value=raw):
                    reader._read_stable_position(0,6)
                group=reader._capture_group(0,[0]*6+[raw],1.)
                self.assertAlmostEqual(teleop.grasp_percent(group,1.01,False),100*(110-tick)/1010)

    def test_calibrated_trigger_endpoints_and_five_finger_curve(self):
        for normalized,expected in [(1.,0.),(.5,50.),(0.,100.)]:
            group=SimpleNamespace(servo_captured_at=[1.]*7,gripper_normalized=normalized)
            self.assertEqual(teleop.grasp_percent(group,1.01,inverted=False),expected)
            self.assertIsNone(teleop.grasp_percent(group,2.,inverted=False))
        np.testing.assert_allclose(teleop.gripper_targets_deg(0,'five_finger'),[0]*6)
        np.testing.assert_allclose(teleop.gripper_targets_deg(20,'five_finger'),[100,0,0,0,0,0])
        np.testing.assert_allclose(teleop.gripper_targets_deg(95,'five_finger'),[100,13,53,56,55,52])
        np.testing.assert_allclose(teleop.gripper_targets_deg(100,'five_finger'),[100,18,58,61,60,57])

    def test_required_hand_initialization_and_trigger_target(self):
        clock=Clock();session=Session(clock);reader=MagicMock();robot=MagicMock();session.robot=robot
        robot.start_state_stream.return_value={'status':0}
        robot.prepare_hand_mode.side_effect=lambda mode: ({'status':0} if not session.commands else self.fail('mode set after arm motion'))
        robot.configure.return_value={'status':0}
        robot.snapshot.return_value={'HandState':dict(online=True,valid_fields=10,
            joint_q_rad=[0]*11,motor_current_ma=[0]*6,field_age_us=[0]*5,mode_raw=3)}
        robot.send_hand.return_value=SimpleNamespace(ok=True,sequence=1)
        reader.latest.return_value=SimpleNamespace(servo_captured_at=[0.]*7,
            gripper_normalized=.5,raw_positions=[0]*6+[1152],unwrapped_ticks=[0]*6+[1152])
        reader.is_connected=True
        values=dict(reader_deg=np.zeros(6),ticks=np.zeros(6),raw_deg=np.zeros(6),velocity_deg_s=np.zeros(6))
        argv=['teleop','--execute','--group','0','--require-hand','--seconds','.01']
        # The selected historical left profile is inverted; midpoint is 50% either way.
        with patch.object(teleop,'time',clock),patch.object(teleop.sys,'argv',argv),patch.object(teleop,'Session',return_value=session),patch.object(teleop,'GelloReader',return_value=reader),patch.object(teleop,'gello_values',return_value=values):
            self.assertEqual(teleop.main(),0)
        robot.prepare_hand_mode.assert_called_once_with(3)
        self.assertEqual([c.kwargs['motor_query_hz'] for c in robot.start_state_stream.call_args_list],[0,500])
        self.assertFalse(robot.start_state_stream.call_args.kwargs['hand_current_fast'])
        self.assertTrue(session.summary['hand_enabled'])
        np.testing.assert_allclose(robot.send_hand.call_args.args[0],np.deg2rad(teleop.gripper_targets_deg(50,'five_finger')))

    def test_required_hand_failure_prevents_follow(self):
        clock=Clock();session=Session(clock);session.robot=MagicMock();reader=MagicMock()
        session.robot.start_state_stream.return_value={'status':0}
        session.robot.prepare_hand_mode.return_value={'status':-110,'reason':'mode timeout'}
        with patch.object(teleop,'time',clock),patch.object(teleop.sys,'argv',['teleop','--execute','--home-zero','--require-hand']),patch.object(teleop,'Session',return_value=session),patch.object(teleop,'GelloReader',return_value=reader),self.assertRaises(RuntimeError):
            teleop.main()
        self.assertFalse(session.commands)
        session.robot.send_hand.assert_not_called()
        reader.stop.assert_called_once()

    def test_required_hand_feedback_loss_stops_sending_targets(self):
        clock=Clock();session=Session(clock);robot=MagicMock();session.robot=robot;reader=MagicMock()
        for method in (robot.start_state_stream,robot.prepare_hand_mode,robot.configure):method.return_value={'status':0}
        fresh={'HandState':dict(online=True,valid_fields=10,joint_q_rad=[0]*11,
            motor_current_ma=[0]*6,field_age_us=[0]*5,mode_raw=3)}
        stale={'HandState':dict(fresh['HandState'],field_age_us=[0,110000,0,110000,0])}
        robot.snapshot.side_effect=[fresh,stale]
        reader.latest.return_value=SimpleNamespace(servo_captured_at=[0.]*7,gripper_normalized=.5,
            raw_positions=[0]*7,unwrapped_ticks=[0]*7)
        values=dict(reader_deg=np.zeros(6),ticks=np.zeros(6),raw_deg=np.zeros(6),velocity_deg_s=np.zeros(6))
        with patch.object(teleop,'time',clock),patch.object(teleop.sys,'argv',['teleop','--execute','--require-hand']),patch.object(teleop,'Session',return_value=session),patch.object(teleop,'GelloReader',return_value=reader),patch.object(teleop,'gello_values',return_value=values),self.assertRaisesRegex(RuntimeError,'Hand feedback lost'):
            teleop.main()
        robot.send_hand.assert_not_called()
        self.assertEqual(session.summary['hand_feedback_error']['angle_age_ms'],110.)
        reader.stop.assert_called_once()

    def test_smooth_home_to_zero_and_restore_follow_gains(self):
        clock=Clock();session=Session(clock);origin=session.q.copy()
        with patch.object(teleop,'time',clock):teleop.home_zero(session)
        np.testing.assert_allclose(session.commands[0][1],origin)
        np.testing.assert_allclose(session.q,np.zeros(6),atol=1e-12)
        np.testing.assert_allclose(session.commands[-1][3],session.args.kp)
        self.assertEqual(session.summary['homing']['status'],'complete')
        self.assertLessEqual(max(np.max(np.abs(x[2])) for x in session.commands),np.deg2rad(30.)+1e-10)

    def test_upright_before_yaw_and_before_lowering(self):
        clock=Clock();session=Session(clock);origin=session.q.copy()
        with patch.object(teleop,'time',clock):teleop.home_zero(session)
        raising=[r for r in session.commands if r[0]=='home_raise']
        aligning=[r for r in session.commands if r[0]=='home_align']
        lowering=[r for r in session.commands if r[0]=='home_lower']
        self.assertTrue(raising and aligning and lowering)
        for r in raising:self.assertEqual(r[1][0],origin[0]);self.assertEqual(r[2][0],0)
        for r in aligning:np.testing.assert_allclose(r[1][1:],teleop.home_path.UPRIGHT[1:])
        np.testing.assert_allclose(lowering[0][1],teleop.home_path.UPRIGHT)
        for stage in ('raise','align','lower'):
            rows=[r for r in session.commands if r[0]=='home_'+stage+'_settle']
            self.assertGreaterEqual(len(rows)*.002,.5)

    def test_failed_raise_never_aligns_or_lowers(self):
        clock=Clock();session=Session(clock,stuck=True)
        with patch.object(teleop,'time',clock),self.assertRaisesRegex(RuntimeError,'raise'):
            teleop.home_zero(session)
        self.assertFalse(any(r[0].startswith(('home_align','home_lower')) for r in session.commands))
        self.assertEqual(session.summary['homing']['stage'],'raise')
        self.assertTrue(session.summary['stop_requested'])

    def test_no_follow_when_home_does_not_reach_zero(self):
        clock=Clock();session=Session(clock,stuck=True)
        with patch.object(teleop,'time',clock),self.assertRaisesRegex(RuntimeError,'回零未到位'):
            teleop.home_zero(session)
        self.assertEqual(session.summary['homing']['status'],'failed')

    def test_reader_starts_after_home_and_targets_have_no_old_pose_offset(self):
        clock=Clock();session=Session(clock);reader=MagicMock()
        reader.latest.return_value=None;reader.is_connected=True
        reader.start.side_effect=lambda:self.assertEqual(session.summary['homing']['status'],'complete')
        values=dict(reader_deg=np.zeros(6),ticks=np.zeros(6),raw_deg=np.zeros(6),velocity_deg_s=np.zeros(6))
        argv=['teleop','--execute','--home-zero','--no-hand','--seconds','.01']
        with patch.object(teleop,'time',clock),patch.object(teleop.sys,'argv',argv),patch.object(teleop,'Session',return_value=session),patch.object(teleop,'GelloReader',return_value=reader),patch.object(teleop,'gello_values',return_value=values):
            self.assertEqual(teleop.main(),0)
        reader.start.assert_called_once();reader.stop.assert_called_once()
        follow=[x for x in session.commands if x[0]=='gello_follow']
        self.assertTrue(follow)
        for row in follow:np.testing.assert_array_equal(row[1],np.zeros(6))

    def test_home_failure_does_not_start_reader(self):
        clock=Clock();session=Session(clock,stuck=True);reader=MagicMock()
        with patch.object(teleop,'time',clock),patch.object(teleop.sys,'argv',['teleop','--execute','--home-zero','--no-hand']),patch.object(teleop,'Session',return_value=session),patch.object(teleop,'GelloReader',return_value=reader),self.assertRaisesRegex(RuntimeError,'回零未到位'):
            teleop.main()
        reader.start.assert_not_called();reader.stop.assert_called_once()


if __name__=='__main__':unittest.main()

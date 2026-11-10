"""Offline independent-master routing and lifecycle tests; no hardware."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import MagicMock,patch
import numpy as np
from test_one_gello_two_arms import bimanual,ROOT
from gello_inputs import GelloInputs,make_reader
sys.path.insert(0,str(ROOT))
from 运行参数 import DUAL_GELLO_CONFIG
from test_board_gello import reply
from board_gello import BoardGelloReader,decode


class Tests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.root=Path(self.folder.name)
        self.config=copy.deepcopy(DUAL_GELLO_CONFIG)
        self.config['gellos']['left'].update(board='192.168.10.11',node=11,group=0)
        base=json.loads((ROOT/'gello-single-arm.json').read_text())
        for side,zero in [('left',1000),('right',2000)]:
            profile=copy.deepcopy(base);profile['left']['zero_raw_ticks']=[zero]*6
            (self.root/f'{side}.json').write_text(json.dumps(profile))
            self.config['gellos'][side]['config']=f'{side}.json'
        self.readers={side:MagicMock(is_connected=True) for side in ('left','right')}
        self.factory=MagicMock(side_effect=lambda spec,*args:self.readers['left' if spec['node']==11 else 'right'])
        self.inputs=GelloInputs(self.config,self.root,bimanual.teleop,self.factory)

    def sample(self,side,delta=0,grasp=10,now=100):
        zero=1000 if side=='left' else 2000
        r=BoardGelloReader();r.set_gripper_calibration(0,-977,110)
        r._accept(decode(reply(ticks=[zero+delta]*6+[grasp]),123),now,now+.001)
        self.readers[side].latest.return_value=r.latest(0)

    def test_independent_zero_gates_and_targets(self):
        self.sample('left');self.sample('right',delta=100)
        values=self.inputs.poll(100.01)
        np.testing.assert_array_equal(values['left'][0],np.zeros(6))
        self.assertIsNone(values['right'][0])
        self.sample('right');self.inputs.poll(100.01)
        self.sample('left',100);self.sample('right',-200)
        values=self.inputs.poll(100.01)
        self.assertFalse(np.allclose(values['left'][0],values['right'][0]))
        np.testing.assert_allclose(values['right'][0],-2*values['left'][0])

    def test_grippers_never_cross_sides(self):
        self.sample('left',grasp=-977);self.sample('right',grasp=110)
        values=self.inputs.poll(100.01)
        self.assertEqual(values['left'][1],0);self.assertEqual(values['right'][1],100)

    def test_one_loss_holds_only_its_measured_position(self):
        for side in self.readers:self.sample(side)
        self.inputs.poll(100.01);self.readers['left'].is_connected=False
        self.sample('right',100)
        desired={side:value[0] for side,value in self.inputs.poll(100.01).items()}
        states={'left':{'q_rad':[.4]*6},'right':{'q_rad':[.8]*6}}
        holds={side:bimanual.teleop.HoldOnLoss() for side in states}
        targets=bimanual.targets_for(desired,states,holds)
        np.testing.assert_allclose(targets['left'],[.4]*6)
        np.testing.assert_allclose(targets['right'],desired['right'])
        states['left']['q_rad']=[.6]*6
        np.testing.assert_allclose(bimanual.targets_for(desired,states,holds)['left'],[.4]*6)

    def test_stale_samples_do_not_follow(self):
        self.sample('left',now=99);self.sample('right',now=100)
        result=self.inputs.poll(100.01)
        self.assertIsNone(result['left'][0]);self.assertIsNotNone(result['right'][0])

    def test_source_identity_reaches_board_reader(self):
        with patch('gello_inputs.BoardGelloReader') as reader:
            make_reader(self.config['gellos']['left'],{},'192.168.10.1',bimanual.teleop)
        reader.assert_called_once_with(board='192.168.10.11',local='192.168.10.1',node=11,port=5021,group_mask=1)

    def test_missing_calibration_precedes_all_reader_creation(self):
        (self.root/'right.json').unlink();factory=MagicMock()
        with self.assertRaisesRegex(ValueError,'独立标定'):
            GelloInputs(self.config,self.root,bimanual.teleop,factory)
        factory.assert_not_called()

    def test_duplicate_physical_source_rejected(self):
        self.config['gellos']['left']['board']='192.168.10.12'
        with self.assertRaisesRegex(ValueError,'同一个输入'):
            GelloInputs(self.config,self.root,bimanual.teleop,self.factory)

    def test_shared_bus_opens_one_reader_for_two_distinct_groups(self):
        self.config['gellos']['left'].update(board='192.168.10.12',node=12,group=1)
        factory=MagicMock(return_value=MagicMock())
        inputs=GelloInputs(self.config,self.root,bimanual.teleop,factory)
        factory.assert_called_once();self.assertEqual(factory.call_args.args[0]['group_mask'],3)
        self.assertIs(inputs.readers['left'],inputs.readers['right'])
        self.assertEqual([c.args[0] for c in factory.return_value.set_gripper_calibration.call_args_list],[1,0])
        inputs.start();inputs.stop()
        factory.return_value.start.assert_called_once();factory.return_value.stop.assert_called_once()

    def test_shared_bus_groups_route_to_their_own_arm_and_trigger(self):
        from test_dual_gello_wire import dual_reply
        from board_gello import NODE,decode_dual
        self.config['gellos']['left'].update(board='192.168.10.12',node=12,group=1)
        reader=BoardGelloReader(group_mask=3)
        inputs=GelloInputs(self.config,self.root,bimanual.teleop,lambda *args:reader)
        def accept(delta,stamp):
            data=dual_reply(stamp=stamp)
            for group,zero,trigger in [(0,2000,110),(1,1000,-977)]:
                values=[zero+(delta if group==1 else 0)]*6+[trigger]
                for i,tick in enumerate(values):NODE.pack_into(data,36+12*(group*7+i),tick%4096,0,tick,1000)
            reader._accept(decode_dual(data,123),100,100.001)
        with patch('board_gello.time.perf_counter',return_value=100.01):
            accept(0,10000);inputs.poll(100.01)
            accept(100,20000);out=inputs.poll(100.01)
        np.testing.assert_allclose(out['right'][0],np.zeros(6))
        self.assertTrue(np.any(np.abs(out['left'][0])>0))
        self.assertEqual(out['left'][1],0);self.assertEqual(out['right'][1],100)

    def test_shared_usb_bus_uses_one_serial_reader(self):
        for side,group in [('left',1),('right',0)]:
            self.config['gellos'][side].update(transport='usb',port='/dev/test-gello',group=group,node=12)
        factory=MagicMock(return_value=MagicMock())
        inputs=GelloInputs(self.config,self.root,bimanual.teleop,factory)
        factory.assert_called_once();self.assertEqual(factory.call_args.args[0]['group_mask'],3)

    def test_second_start_failure_closes_both(self):
        self.readers['right'].start.side_effect=RuntimeError('reader failed')
        with self.assertRaisesRegex(RuntimeError,'reader failed'):self.inputs.start()
        for reader in self.readers.values():reader.stop.assert_called_once()

    def test_preflight_requires_seven_fresh_nodes_on_both(self):
        for side in self.readers:self.sample(side)
        self.readers['left'].latest.return_value=None
        with patch('gello_inputs.time.perf_counter',return_value=100.01):
            with self.assertRaisesRegex(RuntimeError,'left'):self.inputs.wait_ready(timeout=0)

    def test_shared_mode_still_fans_out_one_input(self):
        config=copy.deepcopy(self.config);config['gello']=config.pop('gellos')['right']
        inputs=GelloInputs(config,self.root,bimanual.teleop,self.factory)
        self.sample('right');out=inputs.poll(100.01)
        self.assertEqual(len(inputs.readers),1)
        np.testing.assert_array_equal(out['left'][0],out['right'][0])

    def test_phase_and_command_are_per_side(self):
        sessions={side:MagicMock() for side in self.readers}
        targets={'left':np.full(6,.1),'right':np.full(6,.8)}
        bimanual.send_pair(sessions,{side:{'q_rad':[0]*6} for side in sessions},targets,1,{'left':False,'right':True})
        for side in sessions:np.testing.assert_allclose(sessions[side].step.call_args.args[2],targets[side])
        self.assertEqual(sessions['left'].step.call_args.args[1],'gello_lost_hold')
        self.assertEqual(sessions['right'].step.call_args.args[1],'gello_follow')

    def test_trigger_loss_latches_actual_hand_at_hand_tick(self):
        tunings={side:bimanual.HandTuning() for side in ('left','right')}
        targets={};following={};feedback={side:{'q_rad':[.2]*6} for side in tunings}
        bimanual.update_hand_targets(feedback,{'left':100,'right':0},tunings,targets,following)
        bimanual.update_hand_targets(feedback,{'left':None,'right':100},tunings,targets,following)
        np.testing.assert_allclose(targets['left'],[.2]*6)
        np.testing.assert_allclose(targets['right'],tunings['right'].target(100))
        feedback['left']['q_rad']=[.5]*6
        bimanual.update_hand_targets(feedback,{'left':None,'right':0},tunings,targets,following)
        np.testing.assert_allclose(targets['left'],[.2]*6)

    def test_full_scheduler_routes_two_targets_and_grasps(self):
        import time,csv
        inputs=MagicMock(shared=False,specs=self.config['gellos'])
        inputs.poll.return_value={'left':(np.array([.1,.2,.3,0,0,0]),0),
                                  'right':(np.array([-.1,.4,.6,0,0,0]),100)}
        sessions={}
        def make_session(args,model):
            side='left' if args.board_node==11 else 'right'
            s=MagicMock();s.args=args;s.active=False;s.sent=0;s.robot.lease=0
            s.summary={};s.__enter__.return_value=s
            s.configure_encoder_turns.return_value={}
            s.state.side_effect=lambda:dict(q_rad=[0]*6,dq_rad_s=[0]*6,
                feedback_age_us=[0]*6,fault_code_raw=[0]*6,online_mask=63,
                valid_q_mask=63,valid_dq_mask=63,motor_fault_valid_mask=63,
                host_received_monotonic_s=time.perf_counter())
            s.robot.snapshot.return_value={'Diagnostics':{'hand_control_ack_latest_status':0}}
            s.last_command_sequence=123;sessions[side]=s
            return s
        out=self.root/'run'
        with patch.object(bimanual,'GelloInputs',return_value=inputs),patch.object(bimanual,'wait_for_local_home'), \
             patch.object(bimanual,'Session',side_effect=make_session),patch.object(bimanual,'Gravity'), \
             patch.object(bimanual,'prepare_startup_hands'),patch.object(bimanual,'home_sessions'), \
             patch.object(bimanual.threading,'Thread'),patch.object(bimanual,'checked_hand_feedback',return_value={'q_rad':[0]*6,'mode':3,'angle_age_ms':0}):
            bimanual.run(self.config,out,.012)
        for side,s in sessions.items():
            self.assertTrue(s.step.called)
            np.testing.assert_allclose(s.step.call_args.args[2],inputs.poll.return_value[side][0])
            expected=bimanual.HandTuning().target(inputs.poll.return_value[side][1])
            np.testing.assert_allclose(s.robot.send_hand.call_args.args[0],expected)
        with (out/'dual_targets.csv').open() as stream:rows=list(csv.DictReader(stream))
        self.assertEqual({r['side'] for r in rows},{'left','right'})
        self.assertEqual({r['grasp_percent'] for r in rows},{'0','100'})
        inputs.stop.assert_called_once()

    def test_run_preflight_failure_stops_inputs_without_creating_session(self):
        inputs=MagicMock();inputs.wait_ready.side_effect=RuntimeError('not ready')
        with patch.object(bimanual,'GelloInputs',return_value=inputs),patch.object(bimanual,'wait_for_local_home'),patch.object(bimanual,'Session') as session:
            with self.assertRaisesRegex(RuntimeError,'not ready'):
                bimanual.run(self.config,self.root/'out',.01)
        session.assert_not_called();inputs.stop.assert_called_once()


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('calibrate_gello_test',ROOT/'tools/calibrate-gello.py')
        cls.tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.tool)

    def test_zero_requires_directions_and_preserves_other_profile(self):
        data={'right':{'sentinel':True}}
        with self.assertRaises(ValueError):self.tool.apply_capture(data,'left','zero',[100]*7)
        self.tool.apply_capture(data,'left','zero',[4200]*7,[-1,1,-1,1,-1,1])
        self.assertEqual(data['left']['zero_raw_ticks'],[104]*6);self.assertEqual(data['right'],{'sentinel':True})

    def test_trigger_crossing_wrap_maps_open_closed_in_both_directions(self):
        for opened,closed in [(3119,110),(110,3119)]:
            p={};self.tool.apply_capture(p,'left','open',[0]*6+[opened])
            self.tool.apply_capture(p,'left','closed',[0]*6+[closed]);g=p['left']['gripper']
            r=BoardGelloReader();r.set_gripper_calibration(0,g['min_raw'],g['max_raw'])
            for i,(tick,expected) in enumerate([(opened,0),(closed,100)]):
                unwrapped=opened+(tick-opened+2048)%4096-2048
                r._accept(decode(reply(stamp=10000+i,ticks=[0]*6+[unwrapped]),123),100,100.001)
                self.assertAlmostEqual(bimanual.teleop.grasp_percent(r.latest(0),100.01,g['inverted']),expected)

    def test_moving_or_insufficient_samples_rejected(self):
        with self.assertRaises(ValueError):self.tool.stable_ticks([[0]*7]*9)
        with self.assertRaises(ValueError):self.tool.stable_ticks([[i*10]*7 for i in range(10)])
        self.assertEqual(self.tool.stable_ticks([[100]*7]*10),[100]*7)

if __name__=='__main__':unittest.main()

"""Hardware-free checks of the combined GUI controller and hand channel mapping."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import arm_runtime
import numpy as np
import arm_lab
from arm_dynamics import Gravity,read_json
from robot_sdk import SendResult
from test_arm_examples import FakeRobot
from hand_slider_demo import Shared,Friction,hand_feedback,run_control,HAND_INDICES,EXAMPLES,gripper_targets_deg


class HandRobot(FakeRobot):
    fail_hand=False
    fail_mode=False
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self._lock=threading.RLock();self.count={};self.hand_commands=[];self.stream_calls=[]
        self.initial=[.2,.1,.3,.4,.5,.6];self.hand_q=[0.]*11
        for i,j in enumerate(HAND_INDICES):self.hand_q[j]=self.initial[i]
    def start_state_stream(self,**kwargs):self.stream_calls.append(kwargs);return {'status':0}
    def prepare_hand_mode(self,mode):self.mode=mode;return {'status':1 if self.fail_mode else 0}
    def configure(self,**kwargs):self.config=kwargs;return {'status':0}
    def send_hand(self,q,duration):
        if self.fail_hand:return SendResult(False,0,'synthetic hand send failure')
        self.hand_commands.append(list(q))
        for i,j in enumerate(HAND_INDICES):self.hand_q[j]=q[i]
        return SendResult(True,len(self.hand_commands))
    def snapshot(self, *names):
        state=super().snapshot()
        state['HandState']=dict(header=dict(sequence=self.seq),online=True,valid_fields=10,
            joint_q_rad=self.hand_q.copy(),motor_current_ma=[10,20,30,40,50,60],
            field_age_us=[0]*5,mode_raw=getattr(self,'mode',3))
        return state


def arguments(output):
    args=arm_lab.parser('gravity_friction').parse_args([])
    args.identified=str(EXAMPLES/'profiles/gravity_selected.json')
    args.friction=str(EXAMPLES/'profiles/friction_selected.json');args.friction_scale=.6
    args.hand_mode=3;args.hand_current_ma=500;args.hand_duration_ms=20
    args.output=output;args.seconds=.2
    return args


class Tests(unittest.TestCase):
    def test_five_finger_linear_path_and_profile_isolation(self):
        cases=[(0,[0,0,0,0,0,0]),(10,[50,0,0,0,0,0]),(20,[100,0,0,0,0,0]),
               (57.5,[100,6.5,26.5,28,27.5,26]),(95,[100,13,53,56,55,52]),
               (97.5,[100,15.5,55.5,58.5,57.5,54.5]),(100,[100,18,58,61,60,57])]
        new=Shared('five_finger');old=Shared()
        new.initialize([0.]*6);old.initialize([0.]*6)
        new.set_individual(False);old.set_individual(False)
        for p,target in cases+list(reversed(cases)):
            np.testing.assert_allclose(gripper_targets_deg(p,'five_finger'),target)
            new.set_gripper(p)
            np.testing.assert_allclose(new.targets(),np.deg2rad(target))
        for boundary in (20,95):
            np.testing.assert_allclose(gripper_targets_deg(boundary-1e-7,'five_finger'),
                                       gripper_targets_deg(boundary+1e-7,'five_finger'),atol=1.1e-6)
        old.set_gripper(100);new.set_gripper(100)
        np.testing.assert_allclose(np.rad2deg(old.targets()),[75,20,55,90,90,90])
        np.testing.assert_allclose(np.rad2deg(new.targets()),[100,18,58,61,60,57])
        before=new.targets();new.set_individual(True)
        self.assertEqual(before,new.targets())
        self.assertAlmostEqual(np.rad2deg(new.hand_max_rad[0]),100)

    def test_gripper_linear_segments_and_boundaries(self):
        for percent,expected in [(0,[0,0,0]),(10,[37.5,0,0]),(20,[75,0,0]),
                                 (57.5,[75,9,26]),(95,[75,18,52]),
                                 (97.5,[75,19,53.5]),(100,[75,20,55])]:
            np.testing.assert_allclose(gripper_targets_deg(percent),expected+[90,90,90])
        for boundary in (20,95):
            np.testing.assert_allclose(gripper_targets_deg(boundary-1e-7),gripper_targets_deg(boundary+1e-7),atol=1e-6)
        for percent in (-1,101,float('nan')):
            with self.assertRaises(ValueError):gripper_targets_deg(percent)

    def test_gripper_switch_is_atomic_and_preserves_return_to_individual(self):
        shared=Shared();shared.initialize([.1]*6)
        shared.set_individual(False)
        np.testing.assert_allclose(shared.targets(),np.deg2rad([0,0,0,90,90,90]))
        for p in (100,97.5,57.5,10,0):
            shared.set_gripper(p)
            np.testing.assert_allclose(shared.targets(),np.deg2rad(gripper_targets_deg(p)))
        shared.set_gripper(97.5);before=shared.targets()
        shared.set_joint(0,5)
        self.assertEqual(before,shared.targets())
        shared.set_individual(True)
        self.assertEqual(before,shared.targets())
        shared.set_joint(0,40)
        self.assertAlmostEqual(shared.targets()[0],np.deg2rad(40))
        np.testing.assert_array_equal(shared.targets()[1:],before[1:])

    def test_feedback_channels_and_current(self):
        value=hand_feedback(dict(online=True,valid_fields=10,joint_q_rad=list(range(11)),
            motor_current_ma=[1,2,3,4,5,6],field_age_us=[0,2000,0,3000,0],mode_raw=3))
        self.assertEqual(value['q_rad'],[0,1,2,4,6,8])
        self.assertEqual(value['current_ma'],[1,2,3,4,5,6])
        self.assertEqual((value['angle_age_ms'],value['current_age_ms']),(2,3))
        self.assertIsNone(hand_feedback({})['current_ma'])

    def test_friction_matches_existing_sixty_percent_formula(self):
        a=arguments('unused');model=Gravity(identified=a.identified);f=Friction(a,model)
        for now in (0.,.02,.04):value=f.torque(now,np.full(6,now))
        expected=[.6*(j['fc_nm']*np.tanh(1/a.friction_smooth)+j['b_nm_s_rad']) for j in f.joints]
        np.testing.assert_allclose(value,expected,atol=1e-12)

    def test_worker_initial_pose_targets_rates_and_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(arm_lab,'Robot',HandRobot):
            a=arguments(str(Path(tmp)/'run'));shared=Shared();stop=threading.Event()
            worker=threading.Thread(target=run_control,args=(a,shared,stop));worker.start()
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                robot=HandRobot.instances[-1] if HandRobot.instances else None
                if robot and robot.hand_commands:break
                time.sleep(.005)
            self.assertIsNotNone(robot);self.assertTrue(robot.hand_commands)
            shared.set_joint(3,30)
            worker.join(4);self.assertFalse(worker.is_alive())
            self.assertIsNone(shared.snapshot()['error'])
            np.testing.assert_allclose(robot.hand_commands[0],robot.initial)
            self.assertTrue(any(abs(row[3]-np.deg2rad(30))<1e-12 for row in robot.hand_commands))
            for row in robot.hand_commands:np.testing.assert_allclose(np.array(row)[[0,1,2,4,5]],np.array(robot.initial)[[0,1,2,4,5]])
            self.assertEqual(robot.mode,3)
            self.assertEqual(robot.config['arm_state_period_us'],2000)
            self.assertEqual(robot.config['hand_state_period_us'],20000)
            self.assertTrue(robot.stream_calls[-1]['hand_angles_only'])
            self.assertTrue(robot.stream_calls[-1]['hand_current_fast'])
            self.assertEqual(robot.stream_calls[-1]['hand_query_hz'],50)
            self.assertTrue(robot.closed);self.assertEqual(robot.lease,0)
            report=read_json(Path(a.output)/'summary.json')
            self.assertGreater(report['hand_commands'],2)
            self.assertTrue((Path(a.output)/'hand.csv').exists())
            for cmd in robot.commands[:-10]:
                np.testing.assert_array_equal(cmd[2],np.zeros(6))
                np.testing.assert_array_equal(cmd[3],np.zeros(6))

    def test_failed_hand_send_and_mode_release_session(self):
        for attribute in ('fail_hand','fail_mode'):
            with tempfile.TemporaryDirectory() as tmp,patch.object(arm_lab,'Robot',HandRobot),patch.object(HandRobot,attribute,True):
                shared=Shared();run_control(arguments(str(Path(tmp)/'run')),shared,threading.Event())
                self.assertIsNotNone(shared.snapshot()['error'])
                robot=HandRobot.instances[-1]
                self.assertTrue(robot.closed);self.assertEqual(robot.lease,0)


if __name__=='__main__':unittest.main(verbosity=2)

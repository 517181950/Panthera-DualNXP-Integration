import sys
from pathlib import Path
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'source/host')]
from haptic_coupling import Coupling,TICK_RAD
from tools.hd1910_haptic import HapticBus

class CouplingTest(unittest.TestCase):
    def create(self,signs=None):
        c=Coupling(signs or [1]*6,[-1]*6,[1]*6,[-2000]*6,[2000]*6)
        c.align([0]*6,[0]*6,1.)
        return c
    def test_no_start_jump(self):
        c=self.create();q,h,_=c.update([0]*6,[0]*6,1.01)
        np.testing.assert_array_equal(q,[0]*6);np.testing.assert_array_equal(h,[0]*6)
    def test_resists_master_motion_for_either_sign(self):
        for sign in [-1,1]:
            c=self.create([sign]*6);q,h,_=c.update([100]*6,[0]*6,1.01)
            self.assertTrue(np.all(h<100));self.assertTrue(np.all(h>0))
            self.assertTrue(np.all(np.sign(q)==sign))
    def test_no_force_when_slave_tracks(self):
        c=self.create();q,h,_=c.update([10]*6,[10*TICK_RAD]*6,1.01)
        np.testing.assert_array_equal(h,[10]*6)
    def test_speed_bound(self):
        c=self.create();q,_,_=c.update([1000]*6,[0]*6,1.01)
        self.assertLessEqual(max(abs(q)),np.deg2rad(5)*.01000001)
    def test_limits_reflect_even_if_target_clipped(self):
        c=self.create();c.align([0]*6,[1]*6,1.)
        q,h,_=c.update([100]*6,[1]*6,1.01)
        np.testing.assert_allclose(q,[1]*6);self.assertTrue(np.all(h<100))
    def test_stale_and_invalid_fail(self):
        for t in [1.,1.1,float('nan')]:
            with self.assertRaises(RuntimeError):self.create().update([0]*6,[0]*6,t)
        with self.assertRaises(ValueError):self.create().update([float('nan')]*6,[0]*6,1.01)
        with self.assertRaises(RuntimeError):self.create().update([3000]*6,[0]*6,1.01)
    def test_zero_boundary_tolerance_no_start_jump(self):
        c=Coupling([1]*6,[0]*6,[1]*6,[-1000]*6,[1000]*6)
        q0=[-.003]*6;c.align([0]*6,q0,1.)
        np.testing.assert_allclose(c.target,q0)
        q,_,_=c.update([0]*6,q0,1.01)
        self.assertTrue(np.all(q>q0));self.assertTrue(np.all(q<0))
    def test_large_outside_start_rejected(self):
        with self.assertRaises(ValueError):
            self.create().align([0]*6,[-1.1]*6,1.)
    def test_unknown_sign_rejected(self):
        with self.assertRaises(ValueError):self.create([None]*6)
    def test_tracking_failure(self):
        with self.assertRaises(RuntimeError):self.create().update([0]*6,[.5]*6,1.01)

class TransportTest(unittest.TestCase):
    def config(self):return SimpleNamespace(CURRENT_LIMIT_MA=100,CURRENT_STOP_MA=200,COMMAND_TIMEOUT_S=.05,SERVO_HZ=50,SERVO_MAX_AGE_S=.06)
    def test_stale_command_disables_without_writes(self):
        b=HapticBus(self.config(),{});b.command=([0]*6,-100)
        b.read_all=lambda:{'ticks':[0]*7};disabled=[];sent=[]
        b._disable_all=lambda:disabled.append(True);b._send=lambda *args:sent.append(args)
        b._worker();self.assertIsNotNone(b.error);self.assertEqual(disabled,[True]);self.assertEqual(sent,[])
    def test_attempt_every_disable_even_on_failure(self):
        b=HapticBus(self.config(),{});b.original={n:{} for n in range(1,7)};called=[]
        class Fake:
            node=0
            def disable(self):
                called.append(self.node)
                if self.node==4:raise OSError('timeout')
        b.b=Fake();b._disable_all();self.assertEqual(set(called),set(range(1,7)));self.assertIn(4,b.cleanup_errors)
    def test_wire_current_limit_and_no_duplicate_goal(self):
        b=HapticBus(self.config(),{});b.shift={1:4096};packets=[]
        b.b=SimpleNamespace(node=0,exchange=lambda *args:(packets.append(args) or (b'',0)))
        b._send(1,-100);b._send(1,-100)
        self.assertEqual(len(packets),1)
        instruction,params,reply=packets[0]
        self.assertEqual(instruction,3);self.assertEqual(params[0],41)
        self.assertEqual(int.from_bytes(params[2:4],'little'),3996)
        self.assertEqual(int.from_bytes(params[4:6],'little'),15)  # 97.5mA
    def test_transport_error_disables(self):
        b=HapticBus(self.config(),{});called=[]
        def fail():raise OSError('serial lost')
        b.read_all=fail;b._disable_all=lambda:called.append(True)
        b._worker();self.assertEqual(called,[True]);self.assertIsNotNone(b.error)
    def test_fast_user_motion_preserves_small_feedback_offset(self):
        b=HapticBus(self.config(),{5:{'min_ticks':4644,'max_ticks':7701}})
        # Last logged master 6353, feedback 6334 (-19); next read has moved.
        goal=b.rebase_goal(5,6334,6353,6400)
        self.assertEqual(goal,6381)
        self.assertEqual(goal-6400,-19)
    def test_rebased_feedback_clips_to_range(self):
        b=HapticBus(self.config(),{1:{'min_ticks':0,'max_ticks':1000}})
        self.assertEqual(b.rebase_goal(1,20,40,5),0)
    def test_invalid_feedback_still_stops(self):
        b=HapticBus(self.config(),{1:{'min_ticks':0,'max_ticks':1000}})
        with self.assertRaises(RuntimeError):b.rebase_goal(1,200,100,150)
        with self.assertRaises(RuntimeError):b.rebase_goal(1,1100,1090,900)
    def test_feedback_stale(self):
        b=HapticBus(self.config(),{});b.latest={'oldest':-100}
        with self.assertRaises(RuntimeError):b.snapshot()
    def test_cleanup_never_replays_motion_or_mode(self):
        b=HapticBus(self.config(),{});b.changed=True;b.original={1:{33:4,42:999,44:500}}
        class Fake:
            node=1
            states={n:1 for n in range(1,8)}
            def disable(self):self.states[self.node]=0
            def integer(self,*args,**kw):return self.states[self.node]
            def write(self,*args,**kw):raise AssertionError('cleanup must not restore registers')
            def close(self):pass
        b.b=Fake();b.close()
        self.assertEqual(b.final_torque,{n:0 for n in range(1,8)})
    def test_cleanup_idempotent(self):
        b=HapticBus(self.config(),{});b.b=SimpleNamespace(close=lambda:None)
        b.close();b.close();self.assertIsNone(b.b)

if __name__=='__main__':unittest.main()

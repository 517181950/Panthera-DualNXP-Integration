import sys
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'source/host')]
from gello_hand_feedback import TriggerMapping,HandProgress
from teleop_tuning import HandTuning
from tools.legacy_haptic_reader import LegacyHapticReader
from tools.hd1910_haptic import HapticBus

class HandTest(unittest.TestCase):
    def test_endpoints_and_clamp(self):
        t=TriggerMapping(291,-292)
        self.assertEqual(t.percent(291),0);self.assertEqual(t.percent(-292),100)
        self.assertEqual(t.percent(400),0);self.assertEqual(t.percent(-400),100)
        self.assertAlmostEqual(t.percent(-.5),50)
        self.assertIsNone(t.percent(None));self.assertIsNone(t.percent(float('nan')))
    def test_hand_progress_roundtrip(self):
        tuning=HandTuning(profile='five_finger');p=HandProgress(tuning)
        for percent in (0,5,20,37,75,95,100):self.assertAlmostEqual(p(tuning.target(percent)),percent,places=4)
    def test_seventh_axis_resists_blocked_grasp(self):
        j={n:{'min_ticks':-1000,'max_ticks':2000} for n in range(1,8)}
        r=LegacyHapticReader(j,include_hand=True);r.active=True
        r.sample={'ticks':[500]*6+[-100]};r.master.publish=Mock()
        r.feedback([0]*6,[0]*6,[1]*6,True,hand_percent=0,trigger=TriggerMapping(291,-292))
        targets=r.master.publish.call_args.args[0]
        self.assertEqual(len(targets),7);self.assertGreater(targets[6],-100)
        self.assertLessEqual(targets[6]+100,35)
    def test_missing_hand_feedback_neutralizes_seventh(self):
        j={n:{'min_ticks':-1000,'max_ticks':2000} for n in range(1,8)}
        r=LegacyHapticReader(j,include_hand=True);r.active=True;r.sample={'ticks':[500]*6+[-100]};r.master.publish=Mock()
        r.feedback(None,[0]*6,[1]*6,False)
        self.assertEqual(r.master.publish.call_args.args[0][6],-100)
    def test_peak_setting_does_not_raise_command_limit(self):
        r=LegacyHapticReader({})
        self.assertEqual(r.config.CURRENT_STOP_MA,1800)
        self.assertEqual(r.config.CURRENT_LIMIT_MA,100)
    def test_short_peak_vs_sustained_current(self):
        c=SimpleNamespace(CURRENT_LIMIT_MA=100,CURRENT_STOP_MA=1800,CURRENT_RMS_MA=900,CURRENT_RMS_SECONDS=.5)
        b=HapticBus(c,{})
        b.check_rms(1,0,0);b.check_rms(1,1700,.02)
        with self.assertRaises(RuntimeError):
            for i in range(2,100):b.check_rms(1,1700,i*.02)

if __name__=='__main__':unittest.main()

import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from local_home import wait_for_local_home
class Monitor:
    def __init__(self,phases):self.phases=iter(phases);self.closed=False;self.calls=[]
    def query(self,node):
        self.calls.append(node);return {'local':next(self.phases)}
    def close(self):self.closed=True
class Tests(unittest.TestCase):
    config={'local':'192.168.10.1','boards':{'left':{'node':11},'right':{'node':12}}}
    def run_wait(self,states,timeout=90):
        m=Monitor(states);self.m=m
        return wait_for_local_home(self.config,timeout,monitor_factory=lambda _:m,clock=lambda:0,sleep=lambda _:None)
    def test_wait_both_independently_then_hold(self):
        self.run_wait([{'home_state':3},{'home_state':5},{'home_state':5},{'home_state':5}]);self.assertEqual(self.m.calls,[11,12,11,12]);self.assertTrue(self.m.closed)
    def test_legacy_firmware_accepted(self):self.run_wait([{},{}]);self.assertTrue(self.m.closed)
    def test_host_stop_is_restartable_without_button(self):
        self.run_wait([{'home_state':15,'home_error':0},{'home_state':15,'home_error':0}])
        self.assertEqual(self.m.calls,[11,12]);self.assertTrue(self.m.closed)
    def test_soft_hold_does_not_require_hard_stop_or_button_to_claim(self):
        self.run_wait([{'home_state':16,'home_error':8},{'home_state':16,'home_error':11}])
        self.assertEqual(self.m.calls,[11,12]);self.assertTrue(self.m.closed)
    def test_tracking_fault_is_not_mistaken_for_normal_stop(self):
        with self.assertRaisesRegex(RuntimeError,'跟踪异常'):
            self.run_wait([{'home_state':6,'home_error':8}])
    def test_fault_aborts_without_retry(self):
        with self.assertRaisesRegex(RuntimeError,'20ms'):self.run_wait([{'home_state':6,'home_error':1}])
        self.assertEqual(self.m.calls,[11]);self.assertTrue(self.m.closed)
    def test_timeout_has_no_control(self):
        with self.assertRaisesRegex(RuntimeError,'超时'):self.run_wait([{'home_state':1},{'home_state':2}],timeout=0)
        self.assertTrue(self.m.closed)
if __name__=='__main__':unittest.main()

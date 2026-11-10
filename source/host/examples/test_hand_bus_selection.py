"""Offline checks: select MCU hand bus before streaming/claiming the arm."""
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

from test_arm_examples import FakeRobot, Gravity, lab


class BusRobot(FakeRobot):
    reject = False

    def operation(self, op, **kwargs):
        result = super().operation(op, **kwargs)
        return {'status': -2} if self.reject and op == 16 else result

    def start_state_stream(self, **kwargs):
        assert [op for op,_ in self.operations[:2]] == [19,16]
        return super().start_state_stream(**kwargs)


class Tests(unittest.TestCase):
    def run_session(self, no_hand, reject=False):
        with tempfile.TemporaryDirectory() as tmp:
            args = lab.parser('impedance').parse_args(['--output', str(Path(tmp)/'run')])
            args.no_hand = no_hand
            BusRobot.reject = reject
            with patch.object(lab, 'Robot', BusRobot):
                if reject:
                    with self.assertRaisesRegex(RuntimeError, 'flash the updated firmware'):
                        with lab.Session(args, Gravity()):
                            self.fail('Unsupported firmware must not start teleoperation')
                else:
                    with lab.Session(args, Gravity()) as session:
                        self.assertEqual(session.summary['hand_can_enabled'], not no_hand)
            robot = BusRobot.instances[-1]
            self.assertEqual(robot.operations[:2], [(19, {}),(16, {'target_mask': int(not no_hand)})])
            self.assertTrue(robot.closed)
            self.assertFalse(robot.commands)
            if reject:
                self.assertNotIn(1, [op for op, _ in robot.operations])

    def test_bare_arm_disables_hand_before_stream(self):
        self.run_session(True)

    def test_hand_run_explicitly_restores_bus(self):
        self.run_session(False)

    def test_old_firmware_does_not_silently_ignore_no_hand(self):
        self.run_session(True, reject=True)

    def test_no_log_creates_no_files_on_success_or_failure(self):
        for reject in (False, True):
            with self.subTest(reject=reject), tempfile.TemporaryDirectory() as tmp:
                args = lab.parser('impedance').parse_args(['--output', tmp])
                args.no_hand = True
                args.no_log = True
                BusRobot.reject = reject
                with patch.object(lab, 'Robot', BusRobot), patch.object(lab, 'write_json') as write:
                    if reject:
                        with self.assertRaisesRegex(RuntimeError, 'flash the updated firmware'):
                            with lab.Session(args, Gravity()):
                                self.fail('Expected firmware rejection')
                    else:
                        with lab.Session(args, Gravity()) as session:
                            session.step('test', 'no_log', lab.ZERO, lab.ZERO, args.kp, args.kd)
                    write.assert_not_called()
                self.assertEqual(list(Path(tmp).iterdir()), [])


if __name__ == '__main__':
    unittest.main()

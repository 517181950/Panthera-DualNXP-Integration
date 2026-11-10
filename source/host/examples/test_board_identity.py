"""Exercise actual SDK packet parsing with fake sockets; no network access."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_sdk import Robot
from robot_protocol import IDS, zero, encode, frame


class InputSocket:
    def __init__(self, packet, peer):
        self.packet, self.peer = packet, peer

    def recvfrom(self, size):
        if self.packet is None:
            raise BlockingIOError
        packet, self.packet = self.packet, None
        return packet, self.peer


class BoardIdentityTests(unittest.TestCase):
    def setUp(self):
        self.sockets = patch.object(Robot, '_ensure_sockets')
        self.sockets.start()
        self.addCleanup(self.sockets.stop)

    def packet(self, node, host=10, name='ArmState'):
        state = zero(name)
        state['header'].update(schema_revision=7, source_node=node, target_node=host,
                               source_boot_id=947, sequence=1)
        return frame(IDS[name], encode(name, state))

    def receive(self, robot, node, host=10, address=None):
        sock = InputSocket(self.packet(node, host), address or (robot.board, robot.peer_ports[0]))
        robot.socks = [sock, None]
        with patch('robot_sdk.select.select', return_value=([sock], [], [])):
            return robot.receive()

    def test_selected_snapshot_is_an_isolated_copy(self):
        robot=Robot()
        robot.last={'ArmState':{'q_rad':[1,2,3,4,5,6]},'HandState':{'mode_raw':3}}
        result=robot.snapshot('ArmState')
        self.assertEqual(set(result),{'ArmState'})
        result['ArmState']['q_rad'][0]=99
        self.assertEqual(robot.last['ArmState']['q_rad'][0],1)
        self.assertEqual(set(robot.snapshot()),{'ArmState','HandState'})
        self.assertEqual(robot.snapshot('Missing'),{})

    def test_arm_receipt_timestamp_does_not_refresh_on_duplicate_or_snapshot(self):
        robot = Robot()
        with patch('robot_sdk.time.perf_counter', return_value=100.):
            self.receive(robot, 11)
        with patch('robot_sdk.time.perf_counter', return_value=105.):
            self.assertEqual(self.receive(robot, 11), [])
            sample = robot.snapshot('ArmState')['ArmState']
        self.assertEqual(sample['host_received_monotonic_s'], 100.)

    def test_hand_receipt_time_is_local_and_duplicate_does_not_refresh_it(self):
        robot=Robot()
        for now in (100.,105.):
            sock=InputSocket(self.packet(11,name='HandState'),(robot.board,robot.peer_ports[0]))
            robot.socks=[sock,None]
            with patch('robot_sdk.select.select',return_value=([sock],[],[])),patch('robot_sdk.time.perf_counter',return_value=now):
                robot.receive()
        self.assertEqual(robot.snapshot('HandState')['HandState']['host_received_monotonic_s'],100.)

    def test_default_and_right_headers_and_ports(self):
        left = Robot()
        right = Robot(board='192.168.10.12', board_node=12, ports=(5015, 5016))
        self.assertEqual(left.header()['target_node'], 11)
        self.assertEqual(right.header()['target_node'], 12)
        self.assertEqual(left.header()['source_node'], 10)
        self.assertTrue(set(left.ports).isdisjoint(right.ports))
        self.assertEqual(right.peer_ports, (5015, 5016))
        self.assertEqual(self.receive(left, 11)[0][0], 'ArmState')
        self.assertEqual(self.receive(right, 12)[0][0], 'ArmState')

    def test_wrong_node_or_host_rejected_before_adopting_boot(self):
        for node, host in [(11, 10), (12, 9)]:
            with self.subTest(node=node, host=host):
                robot = Robot(board='192.168.10.12', board_node=12, ports=(5015, 5016))
                self.assertEqual(self.receive(robot, node, host), [])
                self.assertEqual(robot.board_boot, 0)
                self.assertEqual(robot.stats['invalid_identity'], 1)

    def test_correct_identity_from_wrong_endpoint_rejected(self):
        robot = Robot(board='192.168.10.12', board_node=12, ports=(5015, 5016))
        self.assertEqual(self.receive(robot, 12, address=('192.168.10.11', 5015)), [])
        self.assertEqual(robot.board_boot, 0)
        self.assertEqual(robot.stats['foreign_peer'], 1)

    def test_invalid_identity_rejected_before_socket_creation(self):
        for node in (0, 65536, 10, True, 12.5):
            with self.subTest(node=node), self.assertRaises(ValueError):
                Robot(board_node=node)


if __name__ == '__main__':
    unittest.main()

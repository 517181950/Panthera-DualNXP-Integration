"""Wire interoperability and isolation of the two physical board identities."""
import importlib.util
import json
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch
import zlib

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'source/host'))
from dual_peer import decode,PeerMonitor,SIZE
spec=importlib.util.spec_from_file_location('dual_launcher',ROOT/'run-dual-nxp.py')
launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)


def packet(node=11,target=12,boot=111,peer_boot=222):
    data=bytearray(SIZE)
    struct.pack_into('<4sHHHHQIQ',data,0,b'PHD1',3,SIZE,node,target,boot,5,10000)
    struct.pack_into('<Q',data,304,peer_boot)
    struct.pack_into('<I',data,SIZE-4,zlib.crc32(data[:-4]))
    return bytes(data)


class FakeSocket:
    def __init__(self, mutate=None):self.mutate=mutate
    def sendto(self,data,addr):self.nonce=data[4:];self.addr=addr
    def recvfrom(self,n):
        local=packet();remote=packet(12,11,222,111)
        if self.mutate:local,remote=self.mutate(local,remote)
        return b'PDR1'+self.nonce+local+remote,self.addr


class PeerTests(unittest.TestCase):
    def test_c_packet_with_real_crc_and_all_arrays(self):
        data=(ROOT/'evidence/peer-c-golden.bin').read_bytes();s=decode(data)
        self.assertEqual(s['boot'],0x123456789abcdef0)
        self.assertEqual(s['q'],[-.25,-.125,0,.125,.25,.375])
        self.assertEqual(s['motor_feedback_count'],list(range(1000,1006)))
        self.assertEqual(s['hand_control_ack'],456)
        self.assertEqual(s['home_button_target_boot'],0x12345678abcddcba)
        self.assertEqual(s['home_button_seq'],19)
        self.assertEqual(s['home_button_ref_seq'],999)
        self.assertEqual(s['home_button_ack'],18)

    def test_every_byte_corruption_rejected(self):
        data=packet()
        for i in range(SIZE):
            bad=bytearray(data);bad[i]^=1
            with self.assertRaises(ValueError):decode(bad)

    def test_truncated_and_oversize_rejected(self):
        for data in (packet()[:-1],packet()+b'0'):
            with self.assertRaises(ValueError):decode(data)

    def test_valid_peer_pair(self):
        monitor=PeerMonitor.__new__(PeerMonitor);monitor.socket=FakeSocket()
        s=monitor.query(11)
        self.assertEqual(s['remote']['node'],12)
        self.assertEqual(s['local']['peer_boot'],s['remote']['boot'])

    def test_foreign_node_and_inconsistent_boot_rejected(self):
        for mutate in (lambda a,b:(packet(12,11),b), lambda a,b:(a,packet(11,12,222)),
                       lambda a,b:(a,packet(12,11,333))):
            monitor=PeerMonitor.__new__(PeerMonitor);monitor.socket=FakeSocket(mutate)
            with self.assertRaises(ValueError):monitor.query(11)

    def test_uncalibrated_empty_side_cannot_inherit_other_zero(self):
        config=json.loads((ROOT/'dual-nxp.json').read_text())
        with self.assertRaises(ValueError):launcher.arguments(config,'left',Path('/tmp/test'))

    def test_connected_side_preserves_zero_gain_ports_and_homing(self):
        config=json.loads((ROOT/'dual-nxp.json').read_text())
        args=launcher.arguments(config,'right',Path('/tmp/test'))
        self.assertIn('--home-zero',args);self.assertIn('--require-hand',args)
        self.assertEqual(args[args.index('--board')+1],'192.168.10.12')
        self.assertEqual(args[args.index('--board-node')+1],'12')
        self.assertEqual(args[args.index('--ports')+1:args.index('--ports')+3],['5015','5016'])
        self.assertEqual(args[args.index('--kp')+1:args.index('--kp')+7],list(map(str,config['kp'])))
        profile=json.loads((ROOT/'gello-single-arm.json').read_text())
        self.assertEqual(profile['left']['zero_raw_ticks'],[1937,933,210,1847,1385,2574])
        self.assertTrue(profile['left']['gripper']['inverted'])

if __name__=='__main__':unittest.main()

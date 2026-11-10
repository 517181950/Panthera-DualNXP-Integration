"""Two-group GQ02/GS02 wire identity, per-node freshness and real UDP loopback."""
import socket
import struct
import threading
import time
import unittest
from test_board_gello import reply
from board_gello import BoardGelloReader,DUAL_HEADER,DUAL_SIZE,NODE,decode_dual


def dual_reply(nonce=123,stamp=10000,mask=16383,node=12):
    data=bytearray(DUAL_SIZE)
    DUAL_HEADER.pack_into(data,0,b'GS02',nonce,1,1,stamp,mask,1,2,node,0)
    for i in range(14):NODE.pack_into(data,36+12*i,100+i,-5,4196+i,1000)
    struct.pack_into('<5I',data,204,7,0,0,0,0)
    return data


class Tests(unittest.TestCase):
    def reader(self):
        r=BoardGelloReader(group_mask=3)
        for group in (0,1):r.set_gripper_calibration(group,0,1000)
        return r

    def test_groups_are_disjoint_and_loss_does_not_refresh_other_group(self):
        r=self.reader();self.assertTrue(r._accept(decode_dual(dual_reply(),123),10,10.001))
        self.assertEqual(r.latest(0).servo_ids,tuple(range(1,8)))
        self.assertEqual(r.latest(1).servo_ids,tuple(range(11,18)))
        self.assertEqual(r.latest(0).unwrapped_ticks[0],4196)
        self.assertEqual(r.latest(1).unwrapped_ticks[0],4203)
        r._accept(decode_dual(dual_reply(stamp=20000,mask=127),123),11,11.001)
        self.assertTrue(r.latest(0).data_valid);self.assertFalse(r.latest(1).data_valid)
        self.assertEqual(r.latest(1).servo_captured_at,(None,)*7)
        self.assertFalse(r._accept(decode_dual(dual_reply(stamp=20000),123),12,12.001))

    def test_bad_version_length_identity_mask_reserved_rejected(self):
        for data in (reply(),dual_reply()[:-1],dual_reply()+b'x',dual_reply(node=11)):
            with self.assertRaises(ValueError):decode_dual(data,123)
        for offset,value in [(0,0),(31,1),(34,1),(29,255)]:
            data=dual_reply();data[offset]=value
            with self.assertRaises(ValueError):decode_dual(data,123)
        with self.assertRaises(ValueError):decode_dual(dual_reply(),124)
        data=dual_reply();struct.pack_into('<I',data,36+7*12+8,150000)
        with self.assertRaises(ValueError):decode_dual(data,123)

    def test_group_one_only_can_calibrate_and_read(self):
        r=BoardGelloReader(group_mask=2);r.set_gripper_calibration(1,0,1000)
        r._accept(decode_dual(dual_reply(),123),10,10.001)
        self.assertIsNone(r.latest(0));self.assertTrue(r.latest(1).data_valid)

    def test_actual_udp_single_connection_delivers_both_groups_and_cleans_up(self):
        server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server.bind(('127.0.0.1',0));server.settimeout(.05);stop=threading.Event();clients=set()
        def serve():
            stamp=10000
            while not stop.is_set():
                try:data,addr=server.recvfrom(64)
                except socket.timeout:continue
                clients.add(addr)
                if len(data)==8 and data[:4]==b'GQ02':
                    stamp+=5000;server.sendto(dual_reply(struct.unpack_from('<I',data,4)[0],stamp),addr)
        thread=threading.Thread(target=serve);thread.start()
        r=BoardGelloReader('127.0.0.1','127.0.0.1',port=server.getsockname()[1],group_mask=3)
        for group in (0,1):r.set_gripper_calibration(group,0,1000)
        try:
            r.start();end=time.perf_counter()+1
            while not r.is_connected and time.perf_counter()<end:time.sleep(.005)
            self.assertTrue(r.latest(0).data_valid);self.assertTrue(r.latest(1).data_valid)
            self.assertEqual(len(clients),1)
        finally:r.stop();stop.set();thread.join(1);server.close()
        self.assertIsNone(r.latest(0));self.assertIsNone(r.latest(1))

if __name__=='__main__':unittest.main()

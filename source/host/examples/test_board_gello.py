import importlib.util
import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from board_gello import BoardGelloReader, HEADER, NODE, decode

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('board_gello_teleop',Path(__file__).with_name('18_gello-low-impedance.py'))
teleop=importlib.util.module_from_spec(spec);spec.loader.exec_module(teleop)


def reply(nonce=123,boot=1,stamp=10000,age=1000,mask=127,ticks=None):
    ticks=ticks or [1937,933,210,1847,1385,2574,110]
    data=bytearray(136);HEADER.pack_into(data,0,b'GS01',nonce,boot,1,stamp,mask,1,12)
    for i,t in enumerate(ticks):NODE.pack_into(data,32+12*i,t%4096,-5,t,age)
    struct.pack_into('<5I',data,116,7,0,0,0,0)
    return data


class BoardGelloTests(unittest.TestCase):
    def reader(self):
        r=BoardGelloReader();r.set_gripper_calibration(0,-977,110);return r

    def test_codec_rejects_bad_identity_and_shape(self):
        data=reply();self.assertEqual(decode(data,123)['node'],12)
        for bad in (data[:-1],data+b'x',b'bad!'+data[4:]):
            with self.assertRaises(ValueError):decode(bad,123)
        with self.assertRaises(ValueError):decode(data,124)
        with self.assertRaises(ValueError):decode(data,123,11)
        struct.pack_into('<H',data,32,5000)
        with self.assertRaises(ValueError):decode(data,123)

    def test_saved_zero_same_as_usb(self):
        r=self.reader();r._accept(decode(reply(),123),10,10.001)
        profile=json.loads((ROOT/'gello-single-arm.json').read_text())['left']
        self.assertEqual(teleop.FixedGelloZero(profile).target(r.latest(0).unwrapped_ticks[:6]).tolist(),[0.]*6)
        self.assertEqual(teleop.grasp_percent(r.latest(0),10.001,True),100.)

    def test_trigger_wrap_and_reboot_alignment(self):
        r=self.reader()
        for boot,t,percent in [(1,3119,0),(1,4206,100),(2,110,100),(2,-977,0)]:
            data=reply(boot=boot,stamp=10000+int(t+2000),ticks=[1937,933,210,1847,1385,2574,t])
            # Each independent sample below advances board time.
            sample=decode(data,123);sample['board_us']=r.diagnostics.get('board_us',0)+10000
            self.assertTrue(r._accept(sample,20,20.001))
            self.assertAlmostEqual(teleop.grasp_percent(r.latest(0),20.001,True),percent)

    def test_uart_age_not_refreshed_by_udp(self):
        r=self.reader();r._accept(decode(reply(age=140000),123),10,10.002)
        self.assertAlmostEqual(r.latest(0).servo_captured_at[0],9.86)
        r._accept(decode(reply(stamp=20000,age=149000),123),10.01,10.012)
        self.assertIsNone(r.latest(0).servo_captured_at[0])
        data=reply(mask=126);r2=self.reader();r2._accept(decode(data,123),10,10.001)
        self.assertIsNone(r2.latest(0).unwrapped_ticks[0]);self.assertFalse(r2.latest(0).data_valid)

    def test_board_link_alone_is_not_gello_connection(self):
        r=self.reader();now=time.perf_counter()
        r._accept(decode(reply(mask=0),123),now,now+.001)
        self.assertFalse(r.is_connected)

    def test_delays_duplicates_and_old_board_time(self):
        r=self.reader();sample=decode(reply(),123)
        self.assertFalse(r._accept(sample,10,10.051));self.assertIsNone(r.latest(0))
        self.assertTrue(r._accept(sample,10,10.001))
        self.assertFalse(r._accept(sample,11,11.001))
        self.assertFalse(r._accept(decode(reply(stamp=9999),123),12,12.001))

    def test_real_udp_reader_and_loss(self):
        server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);server.bind(('127.0.0.1',0));server.settimeout(.1)
        stop=threading.Event()
        def serve():
            stamp=0
            while not stop.is_set():
                try:data,addr=server.recvfrom(64)
                except socket.timeout:continue
                if len(data)!=8 or data[:4]!=b'GQ01':continue
                stamp+=5000;server.sendto(reply(nonce=struct.unpack_from('<I',data,4)[0],stamp=stamp),addr)
        worker=threading.Thread(target=serve);worker.start()
        r=BoardGelloReader('127.0.0.1','127.0.0.1',port=server.getsockname()[1]);r.set_gripper_calibration(0,-977,110)
        try:
            r.start();end=time.perf_counter()+1
            while not r.is_connected and time.perf_counter()<end:time.sleep(.01)
            self.assertTrue(r.is_connected);self.assertTrue(r.latest(0).data_valid)
            stop.set();worker.join(1);time.sleep(.18)
            self.assertFalse(r.is_connected)
            self.assertGreater(time.perf_counter()-r.latest(0).servo_captured_at[0],.15)
        finally:stop.set();r.stop();worker.join(1);server.close()
        self.assertIsNone(r.latest(0))


if __name__=='__main__':unittest.main()

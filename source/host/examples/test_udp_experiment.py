"""Real loopback UDP integration test of the SDK and experiment lifecycle.

The board is simulated on 127.0.0.1 with OS-assigned ports. No hardware IP used.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import arm_runtime
import select
import socket
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
import arm_lab
from arm_dynamics import Gravity, read_json
from robot_sdk import Robot
from robot_protocol import zero, frame, unframe, encode, decode, IDS, NAMES


class Board:
    def __init__(self):
        self.socks=[socket.socket(socket.AF_INET,socket.SOCK_DGRAM) for _ in range(2)]
        for sock in self.socks:sock.bind(('127.0.0.1',0));sock.setblocking(False)
        self.ports=tuple(s.getsockname()[1] for s in self.socks)
        self.stop=threading.Event();self.sequence=0;self.commands=[];self.operations=[]
        self.peer_boot=0;self.lease=0;self.q=[0.]*6;self.v=[0.]*6;self.tau=[0.]*6;self.applied=0
        self.client=None;self.thread=None;self.errors=[]
    def factory(self,**kwargs):
        self.client=Robot(local='127.0.0.1',board='127.0.0.1',ports=(0,0),peer_ports=self.ports)
        self.addresses=[s.getsockname() for s in self.client.socks]
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
        return self.client
    def send(self,name,payload,index):
        self.sequence+=1
        payload['header']=dict(schema_revision=7,source_node=11,target_node=10,source_boot_id=947,
            target_boot_id=self.peer_boot,sequence=self.sequence,tx_monotonic_us=int(time.perf_counter()*1e6))
        self.socks[index].sendto(frame(IDS[name],encode(name,payload)),self.addresses[index])
    def run(self):
        try:
            while not self.stop.is_set():
                ready=select.select(self.socks,[],[],.002)[0]
                for sock in ready:
                    while True:
                        try:data,_=sock.recvfrom(2048)
                        except BlockingIOError:break
                        except ConnectionResetError:break  # Windows ICMP port-unreachable during client teardown.
                        kind,mid,session,seq,payload=unframe(data)
                        if kind==2:continue
                        name=NAMES[mid];message=decode(name,payload)
                        if name=='OperationRequest':
                            op=message['operation'];self.operations.append(op)
                            if op==1:self.peer_boot=message['header']['source_boot_id'];self.lease=123
                            if op==2:self.lease=0
                            result=zero('OperationResult');result.update(operation_id=message['operation_id'],status=0,
                                control_lease=self.lease,applied_policy_revision=1)
                            self.send('OperationResult',result,1)
                        elif name=='ArmCommand':
                            self.commands.append(message);v=np.array(message['mit']).reshape(6,5)
                            self.q=v[:,0].tolist();self.v=v[:,1].tolist();self.tau=v[:,4].tolist()
                            self.applied=message['header']['sequence']
                        elif name=='HandCommand':raise AssertionError('Experiment must not send hand targets')
                state=zero('ArmState');state.update(policy_revision=1,online_mask=63,valid_q_mask=63,valid_dq_mask=63,
                    valid_tau_mask=63,q_rad=self.q,dq_rad_s=self.v,tau_measured_nm=self.tau,
                    last_applied_command_sequence=self.applied)
                self.send('ArmState',state,0)
        except BaseException as exc:self.errors.append(exc)
    def close(self):
        self.stop.set()
        if self.thread:self.thread.join(2)
        for sock in self.socks:sock.close()


class UDPTests(unittest.TestCase):
    def test_impedance_over_real_udp(self):
        board=Board()
        try:
            with tempfile.TemporaryDirectory() as d:
                a=arm_lab.parser('impedance').parse_args([]);a.output=str(Path(d)/'udp')
                a.seconds=1.5;a.hz=100;a.transition=.1;a.target_deg=[1,2,3,4,5,6]
                with patch.object(arm_lab,'Robot',board.factory):arm_lab.gravity_run(a,Gravity())
                self.assertFalse(board.errors)
                self.assertGreater(len(board.commands),20)
                np.testing.assert_allclose(np.array(board.commands[-1]['mit']).reshape(6,5)[:,2],a.kp,atol=1e-6)
                np.testing.assert_allclose(np.array(board.commands[-1]['mit']).reshape(6,5)[:,0],np.deg2rad(a.target_deg),atol=1e-6)
                self.assertEqual(board.operations[-3:],[4,2,11])
                report=read_json(Path(a.output)/'summary.json')
                self.assertEqual(report['status'],'complete');self.assertGreater(report['unique_state_records'],10)
        finally:board.close()


if __name__=='__main__':unittest.main(verbosity=2)

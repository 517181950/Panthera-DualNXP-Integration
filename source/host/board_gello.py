"""Read-only GELLO input from NXP UART5; never sends actuator commands.

GQ01/GS01 UDP5021, little endian. Per-node ages include the entire host
roundtrip, so delayed replies cannot make old UART data look fresh.
"""
import secrets
import socket
import struct
import threading
import time

from examples.gello_compare_driver.reader import GroupSnapshot, SERVO_SIGNS

HEADER=struct.Struct('<4sIQIQBBH')
NODE=struct.Struct('<HhiI')
SIZE=136
DUAL_HEADER=struct.Struct('<4sIQIQHBBHH')
DUAL_SIZE=224


def decode(data, nonce, expected_node=12):
    if len(data)!=SIZE:raise ValueError('GELLO reply length')
    magic,echo,boot,scan,board_us,mask,available,node=HEADER.unpack_from(data)
    if magic!=b'GS01' or echo!=nonce or node!=expected_node or not boot or mask&~127 or available>1:
        raise ValueError('GELLO reply identity/header')
    records=[NODE.unpack_from(data,32+12*i) for i in range(7)]
    for i,(raw,velocity,ticks,age) in enumerate(records):
        if raw>=4096 or (mask&(1<<i) and (age>=150000 or ticks%4096!=raw)):
            raise ValueError('GELLO reply sample')
    return dict(boot=boot,scan=scan,board_us=board_us,mask=mask,available=bool(available),node=node,
                records=records,counters=dict(zip(('replies','timeouts','bad','jumps','overflow'),struct.unpack_from('<5I',data,116))))


def decode_dual(data,nonce,expected_node=12):
    if len(data)!=DUAL_SIZE:raise ValueError('Dual GELLO reply length')
    magic,echo,boot,scan,board_us,mask,available,count,node,reserved=DUAL_HEADER.unpack_from(data)
    if (magic!=b'GS02' or echo!=nonce or node!=expected_node or not boot
        or mask&~16383 or available>1 or count!=2 or reserved):
        raise ValueError('Dual GELLO reply identity/header')
    groups=[]
    for group in (0,1):
        records=[NODE.unpack_from(data,36+12*(group*7+i)) for i in range(7)]
        valid_mask=(mask>>(group*7))&127
        for i,(raw,velocity,ticks,age) in enumerate(records):
            if raw>=4096 or (valid_mask&(1<<i) and (age>=150000 or ticks%4096!=raw)):
                raise ValueError('Dual GELLO reply sample')
        groups.append(dict(group=group,records=records,mask=valid_mask))
    return dict(boot=boot,scan=scan,board_us=board_us,mask=mask,available=bool(available),node=node,
                groups=groups,counters=dict(zip(('replies','timeouts','bad','jumps','overflow'),struct.unpack_from('<5I',data,204))))


class BoardGelloReader:
    def __init__(self, board='192.168.10.12', local='192.168.10.1', node=12, port=5021, group_mask=1):
        if group_mask not in (1,2,3):raise ValueError('group_mask must be 1, 2, or 3')
        self.group_mask=group_mask
        self.address=(board,port);self.local=local;self.node=node
        self._lock=threading.Lock();self._stop=threading.Event();self._thread=None
        self._latest={};self._at=0.;self._calibration={};self._aligned={};self._boot=None
        self.diagnostics={};self.error=None

    def set_gripper_calibration(self, group, minimum, maximum):
        if group not in (0,1) or not self.group_mask&(1<<group) or not minimum<maximum or maximum-minimum>=4096:
            raise ValueError('Board GELLO requires a selected group and one-turn gripper calibration')
        self._calibration[group]=(minimum,maximum);self._aligned.pop(group,None)

    @property
    def is_connected(self):
        with self._lock:
            now=time.perf_counter()
            return (bool(self._latest) and now-self._at<.15 and
                    any(stamp is not None and now-stamp<.15 for group in self._latest.values() for stamp in group.servo_captured_at))

    def latest(self,group):
        with self._lock:return self._latest.get(group)

    def start(self):
        if self._thread and self._thread.is_alive():return
        if any(self.group_mask&(1<<g) and g not in self._calibration for g in (0,1)):
            raise ValueError('Set saved gripper calibration for every selected group first')
        self._stop.clear();self._thread=threading.Thread(target=self._run,daemon=True);self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:self._thread.join(1)
        with self._lock:self._latest={};self._at=0.

    def _accept(self,sample,sent,received):
        if received-sent>.05:return False
        if self._boot==sample['boot'] and self.diagnostics:
            if sample['board_us']<=self.diagnostics['board_us']:return False
        if self._boot!=sample['boot']:
            self._aligned={};self._boot=sample['boot']
        snapshots={}
        for part in sample.get('groups',[sample]):
            group_id=part.get('group',0)
            if not self.group_mask&(1<<group_id):continue
            raw=[];ticks=[];velocity=[];stamps=[]
            for i,(r,v,t,age) in enumerate(part['records']):
                # Use send time so network delay cannot refresh old samples.
                stamp=sent-age*1e-6
                valid=sample['available'] and bool(part['mask']&(1<<i)) and received-stamp<.15
                raw.append(r if valid else None);ticks.append(t if valid else None)
                velocity.append(v if valid else None);stamps.append(stamp if valid else None)
            fraction=None
            if ticks[6] is not None:
                if group_id not in self._aligned:
                    lo,hi=self._calibration[group_id];shift=4096*round((ticks[6]-(lo+hi)/2)/4096)
                    self._aligned[group_id]=(lo+shift,hi+shift)
                lo,hi=self._aligned[group_id];fraction=min(1.,max(0.,(ticks[6]-lo)/(hi-lo)))
            bounds=self._aligned.get(group_id,self._calibration[group_id])
            snapshots[group_id]=GroupSnapshot(group=group_id,servo_ids=tuple(range(1+10*group_id,8+10*group_id)),
                raw_positions=tuple(raw),unwrapped_ticks=tuple(ticks),arm_radians=(None,)*6,gripper_radians=None,
                gripper_normalized=fraction,gripper_calib_min_raw=bounds[0],gripper_calib_max_raw=bounds[1],
                gripper_calib_active=False,gripper_calib_done=True,zeroed=False,data_valid=all(x is not None for x in ticks),
                captured_at=sent,raw_velocities=tuple(velocity),servo_captured_at=tuple(stamps),
                arm_velocities_rad_s=tuple(None if v is None else v*.732*6*3.141592653589793/180*SERVO_SIGNS[i] for i,v in enumerate(velocity[:6])),
                velocity_valid=all(v is not None for v in velocity))
        with self._lock:
            self._latest=snapshots;self._at=received;self.diagnostics=sample;self.error=None
        return True

    def _run(self):
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
                sock.bind((self.local,0));sock.connect(self.address);sock.settimeout(.025)
                while not self._stop.is_set():
                    start=time.perf_counter();nonce=secrets.randbits(32)
                    try:
                        dual=bool(self.group_mask&2)
                        sock.send(struct.pack('<4sI',b'GQ02' if dual else b'GQ01',nonce))
                        while time.perf_counter()-start<.04:
                            data=sock.recv(DUAL_SIZE+1)
                            try:sample=(decode_dual if dual else decode)(data,nonce,self.node)
                            except ValueError:continue
                            self._accept(sample,start,time.perf_counter());break
                    except OSError as exc:
                        with self._lock:self.error=str(exc)
                    self._stop.wait(max(0.,.005-(time.perf_counter()-start)))
        except OSError as exc:
            with self._lock:self.error=str(exc)

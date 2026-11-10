"""Read-only PHD1 dual-NXP state protocol. No leases or actuator commands."""
import argparse
import json
from pathlib import Path
import secrets
import socket
import struct
import time
import zlib

SIZE = 492
PORT = 5020
FIELDS = [
    ('node','H'),('target','H'),('boot','Q'),('sequence','I'),('time_us','Q'),
    *[(n,'I') for n in ('active','actuators','q_mask','dq_mask','tau_mask','fault_mask')],
    ('arm_age_us','6I'),('q','6f'),('dq','6f'),('tau','6f'),('fault','6I'),('mode','6I'),
    ('hand_fields','I'),('hand_mode','I'),('hand_age_us','5I'),('hand_q','11f'),
    *[(n,'I') for n in ('tx','rx','tx_error','bad','old','gaps','missed','reboots')],
    ('peer_boot','Q'),
    *[(n,'I') for n in ('peer_sequence','peer_age_us','arm_command_sequence','hand_command_sequence')],
    ('motor_control_tx','6I'),('motor_feedback_count','6I'),('hand_field_feedback_count','5I'),
    *[(n,'I') for n in ('hand_control_tx','hand_control_ack','hand_ack_status','can_state','can_tx_error','can_rx_error')],
    *[(n,'I') for n in ('home_state','home_generation','home_error','home_source')],
    ('home_zero','6f'),
    ('home_button_seq','I'),('home_button_target_boot','Q'),
    *[(n,'I') for n in ('home_button_ref_seq','home_button_age_us','home_button_ack','home_button_status')],
]


def decode(data):
    version=struct.unpack_from('<H',data,4)[0] if len(data)>=8 else 0
    size={1:424,2:464,3:SIZE}.get(version,0)
    if not size or len(data) != size or data[:8] != struct.pack('<4sHH', b'PHD1', version, size):
        raise ValueError('Bad peer packet header/length')
    if zlib.crc32(data[:-4]) != struct.unpack_from('<I',data,size-4)[0]:
        raise ValueError('Bad peer packet CRC')
    result, offset = {}, 8
    for name, fmt in ({1:FIELDS[:-11],2:FIELDS[:-6],3:FIELDS}[version]):
        value = struct.unpack_from('<'+fmt,data,offset)
        result[name] = list(value) if fmt[0].isdigit() else value[0]
        offset += struct.calcsize('<'+fmt)
    assert offset == size-4
    if not result['boot']:
        raise ValueError('Zero boot ID')
    return result


class PeerMonitor:
    def __init__(self, local='192.168.10.1'):
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.bind((local,0))
        self.socket.settimeout(.15)

    def close(self):
        self.socket.close()

    def query(self, node):
        board = '192.168.10.'+str(node)
        nonce = secrets.token_bytes(4)
        self.socket.sendto(b'PDQ1'+nonce,(board,PORT))
        deadline=time.monotonic()+.2
        while time.monotonic()<deadline:
            data, addr=self.socket.recvfrom(2048)
            if addr != (board,PORT) or data[:8] != b'PDR1'+nonce:
                continue
            size=(len(data)-8)//2
            if size not in (424,464,SIZE) or len(data)!=8+size*2:
                raise ValueError('Bad peer inspection reply length')
            local=decode(data[8:8+size])
            other=12 if node==11 else 11
            if local['node']!=node or local['target']!=other:
                raise ValueError('Wrong local board identity')
            remote=decode(data[8+size:]) if any(data[8+size:]) else None
            if remote and (remote['node']!=other or remote['target']!=node or remote['boot']!=local['peer_boot']):
                raise ValueError('Wrong remote board identity/boot')
            return {'local':local,'remote':remote}
        raise TimeoutError('No matching peer inspection reply')


def summarize(samples):
    out={}
    for node in (11,12):
        rows=[s['boards'][str(node)]['local'] for s in samples if str(node) in s['boards']]
        if len(rows)<2:
            out[str(node)]={'samples':len(rows)}
            continue
        a,b=rows[0],rows[-1]
        dt=(b['time_us']-a['time_us'])/1e6
        result={'samples':len(rows),'window_s':dt,'boot_unchanged':len({r['boot'] for r in rows})==1,
                'max_peer_age_us':max(r['peer_age_us'] for r in rows),
                'peer_present_all':all(r['peer_boot'] for r in rows),
                'max_arm_age_us':max(max(r['arm_age_us']) for r in rows),
                'max_hand_angle_age_us':max(r['hand_age_us'][1] for r in rows),
                'arm_faults':sorted({v for r in rows for v in r['fault']}),
                'hand_ack_status':sorted({r['hand_ack_status'] for r in rows}),
                'active_seen':sorted({r['active'] for r in rows}),
                'actuators_enabled':bool(b['actuators']),
                'q_mask':b['q_mask'],'hand_fields':b['hand_fields'],
                'can_state_seen':sorted({r['can_state'] for r in rows}),
                'max_can_tx_error':max(r['can_tx_error'] for r in rows),
                'max_can_rx_error':max(r['can_rx_error'] for r in rows)}
        if result['boot_unchanged'] and dt>0:
            for name in ('tx','rx','tx_error','bad','old','gaps','missed','reboots','hand_control_tx','hand_control_ack'):
                delta=(b[name]-a[name])&0xffffffff
                result[name+'_delta']=delta
                result[name+'_hz']=delta/dt
            for name in ('motor_control_tx','motor_feedback_count','hand_field_feedback_count'):
                result[name+'_hz']=[((v-u)&0xffffffff)/dt for u,v in zip(a[name],b[name])]
        out[str(node)]=result
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    monitor=PeerMonitor();samples=[];errors=[];start=time.monotonic();deadline=start
    args.output.parent.mkdir(parents=True,exist_ok=True)
    try:
        with args.output.with_suffix('.jsonl').open('w') as stream:
            while time.monotonic()-start<args.seconds:
                sample={'elapsed':time.monotonic()-start,'boards':{}}
                for node in (11,12):
                    try: sample['boards'][str(node)]=monitor.query(node)
                    except (OSError,ValueError,TimeoutError) as exc: errors.append({'elapsed':sample['elapsed'],'node':node,'error':str(exc)})
                samples.append(sample);stream.write(json.dumps(sample)+'\n')
                deadline+=.02;time.sleep(max(0,deadline-time.monotonic()))
    finally:
        monitor.close()
        report={'seconds':time.monotonic()-start,'errors':errors,'summary':summarize(samples)}
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))

if __name__=='__main__':main()

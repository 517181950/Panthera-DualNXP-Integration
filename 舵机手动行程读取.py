"""USB总线只读行程记录。不会使能、卸力、写零点或启动机械臂。"""
from datetime import datetime
import json
import signal
import time
from tools.servo_travel_calibration import Bus, CalibrationError, ROOT, resolve_port, signed

PORT=None
CANDIDATE_IDS=[1,2,3,4,5,6,7,11,12,13,14,15,16,17]
PRINT_INTERVAL=1.0
RESCAN_INTERVAL=5.0


def run():
    folder=ROOT/'logs'/('manual_servo_range_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True);b=None;active={};segments=[];last_print=last_scan=0;serial_path=None
    def stop(signum,frame):raise KeyboardInterrupt
    previous=signal.signal(signal.SIGTERM,stop)
    def snapshot():
        p=folder/'ranges.tmp'
        p.write_text(json.dumps({'kind':'observed_manual_ranges_not_confirmed_limits',
                                'segments':segments},ensure_ascii=False,indent=2))
        p.replace(folder/'ranges.json')
    try:
        with (folder/'samples.jsonl').open('w',buffering=1) as log:
            print('只读监测已启动。请缓慢手动移动，不要硬顶挡块；Ctrl+C结束记录。',flush=True)
            print('记录目录：'+str(folder),flush=True)
            while True:
                if b is None:
                    try:
                        serial_path=resolve_port(PORT);b=Bus(serial_path,timeout=.025)
                        active={};last_scan=0;print('连接：'+serial_path,flush=True)
                    except Exception as exc:
                        print('等待读取连接：'+str(exc),flush=True);time.sleep(2);continue
                now=time.monotonic()
                try:
                    if now-last_scan>=RESCAN_INTERVAL:
                        for node in CANDIDATE_IDS:
                            if node in active:continue
                            b.node=node
                            try:
                                identity=list(b.read(0,5,allow_fault=True))
                                # Only support little-endian magnetic encoder families here.
                                if identity[0] not in (2,3) or identity[2]!=0:
                                    print(f'ID{node}版本{identity}未适配此只读入口，跳过',flush=True);continue
                                phase=b.integer(18,1,allow_fault=True)
                                state=dict(id=node,identity=identity,phase=phase,port=serial_path,
                                           segment=len(segments),started_at=datetime.now().isoformat(),
                                           count=0,observed_min=None,observed_max=None,previous=None,
                                           continuous=None,online=True)
                                active[node]=state;segments.append(state)
                                print(f'发现ID{node}，固件{identity[0]}.{identity[1]}，开始独立记录。',flush=True)
                            except CalibrationError:pass
                        if not active:print('尚未发现在线ID，继续等待读取。',flush=True)
                        last_scan=time.monotonic()
                    for node,state in list(active.items()):
                        b.node=node
                        try:
                            data,status=b.exchange(2,[56,8],8)
                            raw=signed(int.from_bytes(data[:2],'little'))
                            torque=b.integer(40,1,allow_fault=True)
                        except CalibrationError:
                            state['online']=False;state['ended_at']=datetime.now().isoformat()
                            active.pop(node);print(f'ID{node}反馈中断；重连后另开记录段。',flush=True);continue
                        previous_raw=state['previous']
                        if previous_raw is None:continuous=raw
                        else:
                            delta=raw-previous_raw
                            if not state['phase']&16:delta=(delta+2048)%4096-2048
                            continuous=state['continuous']+delta
                        state.update(previous=raw,continuous=continuous,count=state['count']+1,
                                     torque=torque,voltage=data[6]/10,temperature=data[7],status=status)
                        if state['observed_min'] is None:state['observed_min']=state['observed_max']=continuous
                        state['observed_min']=min(state['observed_min'],continuous)
                        state['observed_max']=max(state['observed_max'],continuous)
                        log.write(json.dumps(dict(at=datetime.now().isoformat(),id=node,segment=state['segment'],
                                                  raw=raw,continuous=continuous,status=status,torque=torque))+'\n')
                    if time.monotonic()-last_print>=PRINT_INTERVAL:
                        for node,state in active.items():
                            if state['count']:
                                span=state['observed_max']-state['observed_min']
                                print(f"ID{node} 位置={state['previous']}  范围=[{state['observed_min']},{state['observed_max']}]  跨度={span} tick  扭矩={state['torque']}  状态=0x{state['status']:02x}",flush=True)
                        snapshot();last_print=time.monotonic()
                    time.sleep(.08)
                except (OSError,ValueError) as exc:
                    print('串口中断：'+str(exc),flush=True)
                    for state in active.values():state['online']=False
                    active={};b.close();b=None
    except KeyboardInterrupt:print('只读记录已结束。',flush=True)
    finally:
        if b:b.close()
        snapshot();signal.signal(signal.SIGTERM,previous)
        print('行程记录：'+str(folder/'ranges.json'),flush=True)

if __name__=='__main__':run()

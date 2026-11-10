"""USB HD1910M bilateral six-axis teleoperation. Import is hardware-free."""
import json
from pathlib import Path
import runpy
import signal
import sys
import time
from copy import deepcopy
from datetime import datetime

ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'source/host'),str(ROOT/'source/host/examples')]
import numpy as np
import GELLO力反馈参数 as C
from 运行参数 import TELEOP_CONFIG
from arm_dynamics import Gravity
from arm_lab import Session, ZERO
from dual_peer import PeerMonitor
from haptic_coupling import Coupling
from tools.hd1910_haptic import HapticBus


def setup():
    config=deepcopy(TELEOP_CONFIG)
    config['boards']={C.SIDE:config['boards'][C.SIDE]}
    b=config['boards'][C.SIDE];b.update(hand=False,kp=C.ARM_KP,kd=C.ARM_KD)
    config['hz']=C.ARM_HZ
    joints={j['id']:j for j in json.loads(C.CALIBRATION.read_text())['joints']}
    return config,joints


def directions():
    d=json.loads(C.DIRECTIONS.read_text()) if C.DIRECTIONS.exists() else {}
    signs=d.get('signs',[None]*6)
    if d.get('side')!=C.SIDE or d.get('ids')!=list(range(1,7)) or d.get('verified')!=[True]*6:
        raise RuntimeError('新主手六轴方向尚未逐轴核对；先运行 GELLO方向示范.py 和 GELLO方向记录.py')
    if len(signs)!=6 or any(s not in (-1,1) for s in signs):raise RuntimeError('方向数据无效')
    return signs


def inspect_arm(config):
    m=PeerMonitor(config['local'])
    try:s=m.query(config['boards'][C.SIDE]['node'])['local']
    finally:m.close()
    if any(s.get(k)!=63 for k in ('q_mask','dq_mask','tau_mask','fault_mask')) or any(s['fault']) or max(s['arm_age_us'])>=20000:
        raise RuntimeError('原臂六轴反馈无效/故障/过期')
    if s.get('home_state') not in (0,5,15,16):raise RuntimeError('原臂尚未完成本地回零/停止')
    return s


def make_session(config,folder):
    legacy=runpy.run_path(str(ROOT/'run-one-gello-two-arms.py'),run_name='haptic_helpers')
    args=legacy['session_args'](config,C.SIDE,folder)
    model=Gravity(args.urdf,None,args.identified,args.payload_mass,args.payload_com,args.gravity)
    return Session(args,model)


def preview():
    config,joints=setup();r={'arm':inspect_arm(config)};master=HapticBus(C,joints)
    try:
        master.open();samples=[]
        for _ in range(25):samples.append(master.read_all());time.sleep(.02)
        r['master']=samples[-1]
        r['master_scan_max_s']=max(s['received']-s['oldest'] for s in samples)
        try:r['signs']=directions();r['directions_ready']=True
        except RuntimeError as e:r['directions_ready']=False;r['pending']=str(e)
    finally:master.close()
    folder=ROOT/'logs'/('gello_haptic_preview_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    (folder/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2))
    print(json.dumps(r,ensure_ascii=False,indent=2));print('仅查询，未使能。记录：'+str(folder/'result.json'))
    return r


def run():
    if not np.isfinite(C.RUN_SECONDS) or C.RUN_SECONDS<0:
        raise ValueError('RUN_SECONDS须为非负数；0表示持续运行')
    signs=directions()  # Fail before any hardware writes.
    config,joints=setup();inspect_arm(config)
    folder=ROOT/'logs'/('gello_haptic_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    bounds=config['boards'][C.SIDE]['soft_limits']
    coupling=Coupling(signs,np.deg2rad(bounds['lower_deg']),np.deg2rad(bounds['upper_deg']),
                      [joints[n]['min_ticks'] for n in range(1,7)],[joints[n]['max_ticks'] for n in range(1,7)],
                      speed_deg_s=C.ARM_SPEED_DEG_S,strength=C.FEEDBACK_STRENGTH,reflection_deg=C.FEEDBACK_OFFSET_DEG)
    master=HapticBus(C,joints);report={'status':'starting','side':C.SIDE,'signs':signs,'feedback':'bilateral_position_coupling',
            'current_limit_ma':C.CURRENT_LIMIT_MA,'current_stop_ma':C.CURRENT_STOP_MA}
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    old=signal.signal(signal.SIGTERM,stop)
    try:
        master.open()
        stable=[];until=time.perf_counter()+8
        while time.perf_counter()<until:
            stable.append(master.read_all()['ticks'][:6]);stable=stable[-10:]
            if len(stable)==10 and np.max(np.ptp(np.asarray(stable),axis=0))<=2:break
            time.sleep(.02)
        else:raise RuntimeError('等待主手静止超时，请扶稳再启动')
        master.prepare()
        with make_session(config,folder) as session:
            session.configure_encoder_turns(config['boards'][C.SIDE]['encoder_modulo_joints'])
            state=session.state()
            if max(abs(np.asarray(state['dq_rad_s'])))>np.deg2rad(2):raise RuntimeError('机械臂未静止')
            # Check initial arm bounds before enabling the master.
            initial_master=master.read_all()  # Refresh after potentially slow arm startup.
            state=session.state()
            coupling.align(initial_master['ticks'][:6],state['q_rad'],time.perf_counter())
            try:
                master.enable();sample=master.snapshot();state=session.state()
                coupling.align(sample['ticks'][:6],state['q_rad'],time.perf_counter())
                report['alignment']={'master':sample['ticks'],'arm':state['q_rad'],'raw_frame_offsets':dict(master.shift)}
                # First arm target equals measured pose; no autonomous homing trajectory.
                session.step('start','haptic_align',state['q_rad'],ZERO,C.ARM_KP,C.ARM_KD,state=state,pace=False)
                master.publish(sample['ticks'][:6]);master.start()
                start=deadline=time.perf_counter();coupling.now=start;cycle=0;last_print=start
                duration='持续运行' if C.RUN_SECONDS==0 else f'{C.RUN_SECONDS:g}秒测试'
                print(f'原臂六轴力反馈遥操已启动：低强度位置耦合，{duration}；Ctrl+C停止。',flush=True)
                with (folder/'coupling.jsonl').open('w',buffering=1) as log:
                    while C.RUN_SECONDS==0 or time.perf_counter()-start<C.RUN_SECONDS:
                        time.sleep(max(0,deadline-time.perf_counter()))
                        now=time.perf_counter();sample=master.snapshot();state=session.state()
                        qref,reflection,requested=coupling.update(sample['ticks'][:6],state['q_rad'],now)
                        session.step(str(cycle),'haptic_follow',qref,ZERO,C.ARM_KP,C.ARM_KD,state=state,pace=False)
                        master.publish(reflection,source_ticks=sample['ticks'][:6])
                        log.write(json.dumps({'t':now-start,'master':sample,'requested':requested.tolist(),'qref':qref.tolist(),
                                             'q':state['q_rad'],'tau_motor_nm':state['tau_measured_nm'],'reflected_ticks':reflection.tolist()})+'\n')
                        cycle+=1;deadline+=1/C.ARM_HZ
                        if deadline<time.perf_counter():deadline=time.perf_counter()+1/C.ARM_HZ
                        if now-last_print>=5:
                            print(f'遥操{cycle/(now-start):.1f}Hz，主手峰值电流{max(abs(v) for v in sample["current_ma"]):g}mA',flush=True);last_print=now
                report['status']='timed_test_complete'
            finally:
                # Stop independent serial worker before leaving the arm session.
                master.close()
                if session.active:session.stop_homing_fault()
    except BaseException as e:
        report.update(status='stopped',error=str(e));print('停止：'+str(e),flush=True)
    finally:
        try:master.close()
        except Exception as e:report['cleanup_error']=str(e);print(str(e),flush=True)
        report['final_master_torque']=master.final_torque
        report['exit_policy']='disable_and_verify_no_restore'
        signal.signal(signal.SIGTERM,old)
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('记录：'+str(folder/'result.json'),flush=True)
    return report

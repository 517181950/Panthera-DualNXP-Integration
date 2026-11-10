"""Restore legacy read-only GELLO following for the newly calibrated USB master.

Reuse the existing reader, mapping, startup alignment, HoldOnLoss, soft-limit
policy and arm Session, with optional low-current master feedback.
"""
import json
import runpy
import signal
import sys
import time
from pathlib import Path
from datetime import datetime
from copy import deepcopy
ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'source/host'),str(ROOT/'source/host/examples')]
import numpy as np
from 运行参数 import TELEOP_CONFIG
import GELLO力反馈参数 as HAPTIC
from teleop_tuning import HandTuning,HandTargetFilter
from gello_hand_feedback import TriggerMapping,HandProgress
from arm_dynamics import Gravity
from arm_lab import Session,ZERO
from soft_limits import Limits,SoftLimitGovernor
from tools.servo_travel_calibration import Bus,resolve_port,EXPECTED_IDENTITY

SIDE='right'
SECONDS=0.0
DIRECTIONS=ROOT/'gello-haptic-directions.json'
CALIBRATION=ROOT/'logs/gello_reference_zero_20261109_210927_565834/calibration.json'


def build_mapping(teleop):
    d=json.loads(DIRECTIONS.read_text())
    if d.get('side')!=SIDE or d.get('ids')!=list(range(1,7)) or d.get('verified')!=[True]*6:
        raise RuntimeError('新主手方向尚未确认')
    signs=np.asarray(d['signs'])
    if signs.shape!=(6,) or not np.all(np.isin(signs,[-1,1])):raise RuntimeError('方向数据无效')
    joints={j['id']:j for j in json.loads(CALIBRATION.read_text())['joints']}
    # Convert only the saved reference into the legacy mapper's modulo format;
    # the reader still decodes signed raw positions and preserves continuity.
    profile={'zero_raw_ticks':[joints[n]['zero_raw_ticks']%4096 for n in range(1,7)],
             'axes':[{'sign':int(signs[i]*teleop.SERVO_SIGNS[i]),'scale':1.,'offset_rad':0.} for i in range(6)]}
    return teleop.FixedGelloZero(profile),profile


def check_master_off(port):
    b=Bus(port)
    try:
        for n in range(1,8):
            b.node=n
            if b.read(0,5)!=EXPECTED_IDENTITY:raise RuntimeError(f'ID{n}型号不符')
            if b.integer(40,1)!=0:raise RuntimeError(f'ID{n}尚未卸力，先结束其控制程序')
    finally:b.close()


def run(force_feedback=False):
    legacy=runpy.run_path(str(ROOT/'run-one-gello-two-arms.py'),run_name='readonly_helpers')
    teleop=legacy['teleop'];mapping,profile=build_mapping(teleop)
    config=deepcopy(TELEOP_CONFIG);config['boards']={SIDE:config['boards'][SIDE]}
    hand_enabled=bool(HAPTIC.HAND_ENABLED)
    config['boards'][SIDE]['hand']=hand_enabled
    config['require_hands']=True
    hand_options=deepcopy(config['boards'][SIDE]['hand_control'])
    hand_options['max_target_speed_deg_s']=(None if HAPTIC.HAND_TARGET_SPEED_DEG_S is None
                                                else [HAPTIC.HAND_TARGET_SPEED_DEG_S]*6)
    hand_tuning=HandTuning(**hand_options);hand_filter=HandTargetFilter(hand_tuning)
    trigger=TriggerMapping(HAPTIC.TRIGGER_OPEN_TICKS,HAPTIC.TRIGGER_CLOSED_TICKS)
    progress=HandProgress(hand_tuning);hand_percent=None;next_hand=0.;hand_targets={};hand_following={}
    governor=SoftLimitGovernor(Limits(**config['boards'][SIDE]['soft_limits']))
    align=teleop.StartAlignment();hold=teleop.HoldOnLoss()
    folder=ROOT/'logs'/(('gello_legacy_feedback_' if force_feedback else 'gello_readonly_follow_')+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report={'status':'starting','master_commands':'low_current_feedback' if force_feedback else 'read_only','profile':profile,'config':config}
    port=resolve_port();check_master_off(port)
    if force_feedback:
        from tools.legacy_haptic_reader import LegacyHapticReader
        joints={j['id']:j for j in json.loads(CALIBRATION.read_text())['joints']}
        reader=LegacyHapticReader(joints,include_hand=hand_enabled)
    else:
        reader=teleop.GelloReader(port,baudrate=1000000,group_mask=1,timeout_s=.03,read_velocity=True)
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    previous=signal.signal(signal.SIGTERM,stop)
    try:
        legacy['wait_for_local_home'](config)
        reader.start();deadline=time.perf_counter()+3
        while True:
            now=time.perf_counter();group=reader.latest(0)
            values=teleop.gello_values(group,now,stale_s=.15)
            if reader.is_connected and np.all(np.isfinite(values['ticks'])):break
            if now>=deadline:raise RuntimeError('GELLO位置尚未就绪')
            time.sleep(.01)
        args=legacy['session_args'](config,SIDE,folder)
        model=Gravity(args.urdf,None,args.identified,args.payload_mass,args.payload_com,args.gravity)
        with Session(args,model) as session:
            session.configure_encoder_turns(config['boards'][SIDE]['encoder_modulo_joints'])
            report['kp']=args.kp;report['kd']=args.kd
            if hand_enabled:
                legacy['prepare_startup_hands']({SIDE:session},config,{SIDE:hand_tuning})
                report['trigger']={'open_ticks':trigger.open,'closed_ticks':trigger.closed}
                print('七轴→灵巧手已接入：松开张手，扣下握拳。',flush=True)
            if force_feedback:reader.activate()
            start=deadline=time.perf_counter();cycle=0;last_follow=None
            print(('原版跟随＋低强度力反馈已启动' if force_feedback else '只读GELLO遥操已启动：主手不使能')+'；原臂高Kp/Kd，持续运行，Ctrl+C停止。',flush=True)
            with (folder/'targets.jsonl').open('w',buffering=1) as log:
                while SECONDS==0 or time.perf_counter()-start<SECONDS:
                    now=time.perf_counter();state=session.state();group=reader.latest(0)
                    values=teleop.gello_values(group,now,stale_s=.15)
                    desired=mapping.target(values['ticks']) if reader.is_connected else None
                    desired=align.target(desired,state['q_rad'])
                    target,following=hold.target(desired,state['q_rad'])
                    q,v,diag=governor.update(target,state,time.perf_counter())
                    session.step(str(cycle),'readonly_follow' if following else 'input_lost_hold',q,v,args.kp,args.kd,state=state,pace=False)
                    if cycle==0:
                        # Activation ACK can arrive before the next 2ms telemetry frame.
                        until=time.perf_counter()+.02
                        while session.robot.snapshot('ArmState').get('ArmState',{}).get('control_owner')==2:
                            if time.perf_counter()>=until:raise RuntimeError('NXP接管确认超时')
                            time.sleep(.001)
                        deadline=time.perf_counter()
                    if hand_enabled and now>=next_hand:
                        h=legacy['checked_hand_feedback'](session,hand_tuning)
                        grip=None
                        if group is not None and following:
                            stamp=group.servo_captured_at[6]
                            if stamp is not None and 0<=now-stamp<=.15:
                                grip=trigger.percent(group.unwrapped_ticks[6])
                        legacy['update_hand_targets']({SIDE:h},{SIDE:grip},{SIDE:hand_tuning},hand_targets,hand_following)
                        legacy['send_hand_pair']({SIDE:session},{SIDE:h},hand_targets,{SIDE:hand_filter},time.perf_counter())
                        hand_percent=progress(h['q_rad']) if grip is not None else None
                        next_hand=now+.02
                        log.write(json.dumps({'kind':'hand','t':now-start,'grip_requested_percent':grip,'actual_progress_percent':hand_percent,
                                             'actual_q':h['q_rad'],'target_q':hand_targets[SIDE]})+'\n')
                    if force_feedback:reader.feedback(desired,state['q_rad'],mapping.factor,following,hand_percent=hand_percent,trigger=trigger)
                    if following!=last_follow:
                        print('FOLLOW GELLO' if following else 'GELLO反馈缺失，机械臂保持当前位置',flush=True);last_follow=following
                    if cycle%5==0:
                        log.write(json.dumps({'t':now-start,'following':following,'ticks':values['ticks'].tolist(),
                                             'qref':q.tolist(),'q':state['q_rad'],'age_ms':values['age_ms'].tolist(),
                                             **({'master_current_ma':reader.sample['current_ma']} if force_feedback and reader.sample else {})})+'\n')
                    cycle+=1;deadline+=1/config['hz'];delay=deadline-time.perf_counter()
                    if delay>0:time.sleep(delay)
                    else:deadline=time.perf_counter()
            report['status']='completed'
    except BaseException as e:
        report.update(status='stopped',error=str(e));print('停止：'+str(e),flush=True)
    finally:
        try:reader.stop()
        except Exception as e:report['cleanup_error']=str(e);print('主手停止异常：'+str(e),flush=True)
        if force_feedback:
            report['final_master_torque']=reader.master.final_torque
            report['master_fault']=reader.master.last_fault_feedback
            report['feedback_settings']={'current_limit_ma':reader.config.CURRENT_LIMIT_MA,'current_stop_ma':reader.config.CURRENT_STOP_MA,
                                         'strength':reader.config.FEEDBACK_STRENGTH,'offset_deg':reader.config.FEEDBACK_OFFSET_DEG}
        signal.signal(signal.SIGTERM,previous)
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('记录：'+str(folder/'result.json'),flush=True)
    return report

if __name__=='__main__':run()

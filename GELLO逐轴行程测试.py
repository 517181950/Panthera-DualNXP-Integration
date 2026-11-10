"""逐轴遍历已人工记录的GELLO行程。仅用于本次未断电的标定坐标。"""
import json
import math
import signal
import time
from datetime import datetime
from tools.servo_travel_calibration import *

CALIBRATION=ROOT/'logs/gello_reference_zero_20261109_210927_565834/calibration.json'
PORT=None
IDS=[1,2,3,4,5,6,7]
MARGIN_DEG=5.0
STEP_TICKS=4
STEP_INTERVAL=.04
STOP_CURRENT_MA=400.0
CLEARANCE_POSE=None
LOCAL_SWEEP_DEG=None
CLEARANCE_TOLERANCE_DEG=5.0
_pose_by_id={}


def plan(j):
    margin=math.ceil(MARGIN_DEG*TICKS_PER_DEG)
    lo,hi=j['min_ticks']+margin,j['max_ticks']-margin
    zero=_pose_by_id.get(j['id'],j['zero_raw_ticks'])
    if LOCAL_SWEEP_DEG is not None:
        if not 0<LOCAL_SWEEP_DEG<=10:raise CalibrationError('局部测试半幅必须在0..10度')
        delta=round(LOCAL_SWEEP_DEG*TICKS_PER_DEG)
        lo=max(lo,zero-delta);hi=min(hi,zero+delta)
    if lo>=hi or not j['min_ticks']<=zero<=j['max_ticks']:
        raise CalibrationError('行程/零位不适用')
    # Visit the nearer end first; return to the user-confirmed zero last.
    ends=[lo,hi] if abs(zero-lo)<abs(zero-hi) else [hi,lo]
    return ends+[zero]


def run():
    c=Settings(max_current_ma=STOP_CURRENT_MA);c.validate()
    if not 1<=STEP_TICKS<=4 or not .04<=STEP_INTERVAL<=.1 or MARGIN_DEG<5:
        raise ValueError('步进或余量不适用')
    global _pose_by_id
    _pose_by_id={}
    if CLEARANCE_POSE is not None:
        pose=json.loads(Path(CLEARANCE_POSE).read_text())
        if pose['status']!='stable':raise CalibrationError('避让姿态不稳定')
        _pose_by_id={p['id']:p['raw_ticks'] for p in pose['pose']}
    calibration=json.loads(CALIBRATION.read_text());joints={j['id']:j for j in calibration['joints']}
    folder=ROOT/'logs'/('gello_sweep_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    r={'status':'starting','axes':[]};b=None;code=0
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    previous_handler=signal.signal(signal.SIGTERM,stop)
    try:
        b=Bus(resolve_port(PORT))
        def check_clearance(active=None):
            selected=b.node
            try:
                for other,reference in _pose_by_id.items():
                    if other==active:continue
                    b.node=other
                    other_feedback=b.feedback()
                    if other_feedback.status or b.integer(40,1)!=0:
                        raise CalibrationError(f'ID{other}非无故障卸力状态')
                    if abs(other_feedback.position-reference)>CLEARANCE_TOLERANCE_DEG*TICKS_PER_DEG:
                        raise CalibrationError(f'ID{other}偏离抬高姿态超过{CLEARANCE_TOLERANCE_DEG}度，停止')
            finally:b.node=selected
        check_clearance()
        for node in IDS:
            j=joints[node];plan(j);b.node=node
            if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(33,1)!=4:
                raise CalibrationError(f'ID{node}型号/模式不符')
            if b.integer(40,1)!=0:raise CalibrationError(f'ID{node}尚未卸力，不开始遍历')
            if b.integer(55,1)!=1 or b.integer(30,1)!=1 or not b.integer(18,1)&16:
                raise CalibrationError(f'ID{node}锁/分辨率/反馈配置不符')
            if (b.integer(9,2),b.integer(11,2)) not in ((0,4095),(0,0)):
                raise CalibrationError(f'ID{node}已有自定义限位，不覆盖')
            f=b.feedback();check_feedback(f,c)
            if not j['min_ticks']<=f.position<=j['max_ticks']:
                raise CalibrationError(f'ID{node}当前位置不在已记录范围内，停止')
        b.writes_allowed=True
        with (folder/'samples.jsonl').open('w',buffering=1) as log:
            for node in IDS:
                j=joints[node];b.node=node;check_clearance();f=b.feedback();check_feedback(f,c)
                if not j['min_ticks']<=f.position<=j['max_ticks']:raise CalibrationError(f'ID{node}当前位置超出记录范围')
                original={a:b.integer(a,WIDTHS[a]) for a in (11,9,41,44,46)}
                axis={'id':node,'status':'running','initial':asdict(f),'original':original};r['axes'].append(axis)
                try:
                    b.disable();b.write(9,0);b.write(11,0)
                    after=establish_reference(b,c,axis)
                    delta=after.position-f.position;shift=round(delta/4096)*4096
                    if abs(delta-shift)>2:raise CalibrationError('切换后坐标无法按整圈平移对应，拒绝使能')
                    waypoints=[v+shift for v in plan(j)]
                    lo,hi=j['min_ticks']+shift,j['max_ticks']+shift
                    if not -32760<=lo<hi<=32760:raise CalibrationError('行程超出协议范围')
                    axis.update(shift=shift,waypoints=waypoints)
                    def command(pos):
                        data=bytes([0])+encode_position(pos).to_bytes(2,'little')+bytes(4)
                        _,status=b.exchange(3,bytes([41])+data,0)
                        if status:raise CalibrationError(f'ID{node}运动应答故障{status}')
                        if b.read(41,7)!=data:raise CalibrationError('运动回读不符')
                    goal=after.position;command(goal);b.write(40,1)
                    print(f'ID{node}开始：{goal} → '+ ' → '.join(map(str,waypoints)),flush=True)
                    for endpoint in waypoints:
                        v=b.feedback();check_feedback(v,c)
                        started=previous=last_send=last_progress=last_print=last_clearance=time.monotonic()
                        last_pos=previous_pos=v.position;stable=0
                        deadline=started+abs(endpoint-v.position)/50+5
                        while time.monotonic()<deadline:
                            time.sleep(.02);v=b.feedback();now=time.monotonic()
                            log.write(json.dumps(dict(id=node,t=now,target=goal,endpoint=endpoint,**asdict(v)))+'\n')
                            check_feedback(v,c)
                            if now-last_clearance>=.2:
                                check_clearance(active=node);last_clearance=time.monotonic()
                            if now-previous>.1 or abs(v.position-previous_pos)>64:raise CalibrationError('采样延迟/坐标跳变')
                            previous=now;previous_pos=v.position
                            if not lo<=v.position<=hi:raise CalibrationError('反馈超出人工记录范围')
                            if abs(v.position-last_pos)>=2:last_pos=v.position;last_progress=now
                            if abs(goal-v.position)>16:raise CalibrationError('实际位置未跟随目标')
                            if now-last_progress>.8 and abs(v.position-endpoint)>2:raise CalibrationError('运动停滞')
                            if now-last_send>=STEP_INTERVAL and goal!=endpoint:
                                goal=min(goal+STEP_TICKS,endpoint) if goal<endpoint else max(goal-STEP_TICKS,endpoint)
                                command(goal);last_send=time.monotonic()
                            stable=stable+1 if goal==endpoint and abs(v.position-endpoint)<=2 else 0
                            if stable>=3:
                                print(f'ID{node}反馈到达 {v.position}',flush=True);break
                            if now-last_print>=5:
                                print(f'ID{node}位置{v.position}，目标端点{endpoint}，电流{v.current_ma:g}mA',flush=True);last_print=now
                        else:raise CalibrationError('单段动作超时')
                    axis['status']='feedback_sweep_complete'
                except BaseException as exc:
                    axis.update(status='failed',error=str(exc));raise
                finally:
                    # On a stop failure, do not restore stronger parameters or advance axes.
                    b.disable();axis['torque_disabled_confirmed']=True
                    axis['before_restore']=asdict(b.feedback())
                    for a,value in original.items():b.write(a,value)
                    axis['final']=asdict(b.feedback())
                    print(f'ID{node}已卸力并恢复原参数。',flush=True)
        r['status']='feedback_sweep_complete'
    except BaseException as exc:
        code=1;r.update(status='failed',error=str(exc));print('停止：'+str(exc),flush=True)
    finally:
        if b:b.close()
        signal.signal(signal.SIGTERM,previous_handler)
        (folder/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2))
        print('记录：'+str(folder/'result.json'),flush=True)
    return code

if __name__=='__main__':raise SystemExit(run())

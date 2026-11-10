"""复现现场FD的模式4控制字段；小步移动，结束卸力，不修改模式/输出上限。"""
import json
import time
from datetime import datetime
from tools.servo_travel_calibration import *

PORT=None
SERVO_ID=1
DIRECTION=1
MOVE_DEG=20.0
STEP_TICKS=2
STEP_INTERVAL=.05
STOP_CURRENT_MA=200.0


def run():
    if DIRECTION not in (-1,1) or not 0<MOVE_DEG<=30 or not 1<=STEP_TICKS<=2 or not .05<=STEP_INTERVAL<=.1:
        raise ValueError('复现入口仅允许最多30度的小步动作')
    c=Settings(max_current_ma=STOP_CURRENT_MA);c.validate()
    r={'status':'starting','samples':[]};b=None;old=None;code=0
    path=ROOT/'logs'/('fd_replay_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    path.parent.mkdir(exist_ok=True)
    def command(pos):
        payload=bytes([0])+encode_position(pos).to_bytes(2,'little')+bytes(4)
        _,status=b.exchange(3,bytes([41])+payload,0)
        if status:raise CalibrationError(f'应答故障{status}')
        if b.read(41,7)!=payload:raise CalibrationError('控制区回读不符')
    try:
        b=Bus(resolve_port(PORT),SERVO_ID)
        if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(33,1)!=4:raise CalibrationError('需已验证固件和模式4')
        f=b.feedback();check_feedback(f,c)
        target=f.position+DIRECTION*round(MOVE_DEG*TICKS_PER_DEG)
        lo,hi=b.integer(9,2),b.integer(11,2)
        if not 0<=lo<=min(f.position,target)<=max(f.position,target)<=hi<=4095 or lo==hi:raise CalibrationError('目标超出当前单圈区间')
        old={a:b.integer(a,WIDTHS[a]) for a in (41,44,46)}
        r.update(initial=asdict(f),target=target,original=old,output_limit=b.integer(48,2))
        b.writes_allowed=True;b.disable();command(f.position);b.write(40,1)
        goal=f.position;started=last_send=last=time.monotonic();stable=0
        print(f'FD参数复现：{goal} → {target}，加速度/电流字段/速度均为0，保持原输出上限。',flush=True)
        while time.monotonic()-started < MOVE_DEG*TICKS_PER_DEG/STEP_TICKS*STEP_INTERVAL + 4:
            time.sleep(.02);v=b.feedback();now=time.monotonic()
            r['samples'].append(dict(t=now-started,target=goal,**asdict(v)));check_feedback(v,c)
            if now-last>.1:raise CalibrationError('采样间隔异常')
            last=now
            travel=DIRECTION*(v.position-f.position)
            if travel < -4 or travel>abs(target-f.position)+8:raise CalibrationError('运动方向/行程异常')
            if DIRECTION*(goal-v.position)>12:raise CalibrationError('实际位置未跟随目标，停止')
            if now-last_send>=STEP_INTERVAL and goal!=target:
                goal=min(goal+STEP_TICKS,target) if DIRECTION>0 else max(goal-STEP_TICKS,target)
                command(goal);last_send=time.monotonic()
            stable=stable+1 if goal==target and abs(v.position-target)<=2 else 0
            if stable>=3:r['status']='feedback_reached';break
        else:raise CalibrationError('动作超时')
    except BaseException as exc:
        code=1;r.update(status='failed',error=str(exc));print(str(exc),flush=True)
    finally:
        if b:
            if old is not None:
                try:
                    b.disable();r['torque_disabled_confirmed']=True
                    for a,value in old.items():b.write(a,value)
                    r['final']=asdict(b.feedback());print('已确认卸力。',flush=True)
                except Exception as exc:
                    code=1;r.update(status='failed',cleanup_error=str(exc));print(str(exc),flush=True)
            b.close()
        path.write_text(json.dumps(r,ensure_ascii=False,indent=2));print(r['status'],path,flush=True)
    return code

if __name__=='__main__':raise SystemExit(run())

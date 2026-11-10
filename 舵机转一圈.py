"""仅用于已拆除机械挡块、线缆允许整圈旋转的HD1910M；完成后卸力。"""
import json
import signal
import time
from datetime import datetime
from tools.servo_travel_calibration import *

PORT=None
SERVO_ID=1
DIRECTION=1
STEP_TICKS=4
STEP_INTERVAL=.04
STOP_CURRENT_MA=400.0
TIMEOUT_S=65.0


def run():
    if DIRECTION not in (-1,1) or not 1<=STEP_TICKS<=4 or not .04<=STEP_INTERVAL<=.1:
        raise ValueError('无效方向或步进设置')
    c=Settings(max_current_ma=STOP_CURRENT_MA);c.validate()
    r={'status':'starting','samples':[]};b=None;original=None;code=0
    path=ROOT/'logs'/('servo_one_turn_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    path.parent.mkdir(exist_ok=True)
    def interrupted(signum,frame):raise KeyboardInterrupt('停止请求')
    previous_handler=signal.signal(signal.SIGTERM,interrupted)
    def command(pos):
        payload=bytes([0])+encode_position(pos).to_bytes(2,'little')+bytes(4)
        _,status=b.exchange(3,bytes([41])+payload,0)
        if status:raise CalibrationError(f'运动应答故障0x{status:02x}')
        if b.read(41,7)!=payload:raise CalibrationError('运动控制区回读不符')
    try:
        b=Bus(resolve_port(PORT),SERVO_ID)
        if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(33,1)!=4:
            raise CalibrationError('需要已验证的固件和模式4')
        if b.integer(55,1)!=1 or b.integer(30,1)!=1 or not b.integer(18,1)&16:
            raise CalibrationError('需要EEPROM锁定、分辨率1和全角度反馈')
        if (b.integer(9,2),b.integer(11,2)) not in ((0,4095),(0,0)):
            raise CalibrationError('存在自定义机械范围，不自动覆盖')
        initial=b.feedback();check_feedback(initial,c)
        original={a:b.integer(a,WIDTHS[a]) for a in (11,9,41,44,46)}
        r.update(initial=asdict(initial),original=original,output_limit=b.integer(48,2))
        b.writes_allowed=True;b.disable();b.write(9,0);b.write(11,0)
        f=establish_reference(b,c,r)
        target=f.position+DIRECTION*4096
        if not -32760<=target<=32760:raise CalibrationError('目标超出多圈协议范围')
        r.update(start_position=f.position,target=target,coordinate_change=f.position-initial.position)
        goal=f.position;command(goal);b.write(40,1)
        start=previous=last_send=last_progress=last_print=time.monotonic()
        last_pos=previous_pos=f.position;stable=0
        print(f'开始一圈：{goal} → {target}（4096 tick），完成后卸力。',flush=True)
        while time.monotonic()-start<TIMEOUT_S:
            time.sleep(.02);v=b.feedback();now=time.monotonic()
            r['samples'].append(dict(t=now-start,target=goal,**asdict(v)))
            check_feedback(v,c)
            if now-previous>.1:raise CalibrationError('采样间隔异常')
            if abs(v.position-previous_pos)>64:raise CalibrationError('反馈坐标跳变，不能确认连续整圈')
            previous=now;previous_pos=v.position
            travel=DIRECTION*(v.position-f.position)
            if not -4<=travel<=4104:raise CalibrationError('运动方向或行程异常')
            if abs(v.position-last_pos)>=2:last_pos=v.position;last_progress=now
            if now-last_progress>.8:raise CalibrationError('运动停滞，停止')
            if DIRECTION*(goal-v.position)>16:raise CalibrationError('位置未跟随目标，停止')
            if now-last_send>=STEP_INTERVAL and goal!=target:
                goal=min(goal+STEP_TICKS,target) if DIRECTION>0 else max(goal-STEP_TICKS,target)
                command(goal);last_send=time.monotonic()
            stable=stable+1 if goal==target and abs(v.position-target)<=2 else 0
            if stable>=3:
                r['status']='feedback_reached';r['travel_deg']=travel/TICKS_PER_DEG;break
            if now-last_print>=5:
                print(f'反馈累计转角 {travel/TICKS_PER_DEG:.1f}°，电流{v.current_ma:g}mA，电压{v.voltage_v}V',flush=True)
                last_print=now
        else:raise CalibrationError('整圈动作超时')
    except BaseException as exc:
        code=1;r.update(status='failed',error=str(exc));print(str(exc),flush=True)
    finally:
        if b:
            if original is not None:
                try:
                    b.disable();r['torque_disabled_confirmed']=True
                    r['before_restore']=asdict(b.feedback())
                    for a,value in original.items():b.write(a,value)
                    r['final']=asdict(b.feedback());print('已确认卸力，原参数已恢复。',flush=True)
                except Exception as exc:
                    code=1;r.update(status='failed',cleanup_error=str(exc));print(str(exc),flush=True)
            b.close()
        signal.signal(signal.SIGTERM,previous_handler)
        path.write_text(json.dumps(r,ensure_ascii=False,indent=2));print(r['status'],path,flush=True)
    return code

if __name__=='__main__':raise SystemExit(run())

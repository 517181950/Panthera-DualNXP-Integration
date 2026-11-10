"""在已实测的区间内持续往返，Ctrl+C或SIGTERM停止并卸力。"""
import json
import signal
import time
from datetime import datetime
from tools.servo_travel_calibration import *

PORT = None
SERVO_ID = 1
LOWER_TICKS = 3541
UPPER_TICKS = 3872
OUTPUT_PERCENT = 7.0


def run():
    c=Settings(torque_percent=OUTPUT_PERCENT)
    c.validate()
    if not 0<OUTPUT_PERCENT<=7 or not 0<=LOWER_TICKS<UPPER_TICKS<=4095:
        raise ValueError('检查往返区间及输出上限')
    folder=ROOT/'logs'/('servo_loop_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True);r={'status':'starting'};b=None;original=None;code=0
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    old=signal.signal(signal.SIGTERM,stop)
    try:
        b=Bus(resolve_port(PORT),SERVO_ID)
        if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(33,1)!=4:
            raise CalibrationError('需要已验证的固件与模式4')
        f=b.feedback();check_feedback(f,c)
        if not LOWER_TICKS-2<=f.position<=UPPER_TICKS+2:
            raise CalibrationError('当前位置不在已验证的往返区间，拒绝自动移入')
        if b.integer(9,2)>LOWER_TICKS or b.integer(11,2)<UPPER_TICKS:
            raise CalibrationError('往返区间超出舵机软件限位')
        original={a:b.integer(a,WIDTHS[a]) for a in (41,44,46,48)}
        r.update(initial=asdict(f),original=original,lower=LOWER_TICKS,upper=UPPER_TICKS)
        b.writes_allowed=True;b.disable();b.write(48,min(original[48],round(OUTPUT_PERCENT*10)))
        f=establish_reference(b,c,r)
        if not LOWER_TICKS-2<=f.position<=UPPER_TICKS+2:raise CalibrationError('起点已变化')
        target=f.position;direction=-1 if f.position>(LOWER_TICKS+UPPER_TICKS)/2 else 1
        b.write_motion(target,c);b.write(40,1)
        start=last=last_command=last_progress=last_print=time.monotonic();last_pos=f.position;turns=0
        print(f'持续往返开始：{LOWER_TICKS} ↔ {UPPER_TICKS}，Ctrl+C停止并卸力。',flush=True)
        with (folder/'samples.jsonl').open('w',buffering=1) as log:
            while True:
                time.sleep(.02);v=b.feedback();now=time.monotonic()
                check_feedback(v,c)
                if now-last>.1:raise CalibrationError('反馈间隔过长')
                last=now
                if not LOWER_TICKS-4<=v.position<=UPPER_TICKS+4:raise CalibrationError('超出往返范围')
                if abs(v.position-last_pos)>=2:last_pos=v.position;last_progress=now
                if now-last_progress>.8:raise CalibrationError('运动停滞，结束演示')
                endpoint=UPPER_TICKS if direction>0 else LOWER_TICKS
                if abs(v.position-endpoint)<=2:
                    direction=-direction;turns+=1
                if abs(v.current_ma)<c.contact_ma and (direction*(target-v.position)<=2 or now-last_command>=.4):
                    proposed=max(LOWER_TICKS,min(UPPER_TICKS,v.position+direction*c.lead_ticks))
                    if proposed!=target:
                        target=proposed;b.write_motion(target,c);last_command=time.monotonic()
                log.write(json.dumps(dict(t=now-start,target=target,**asdict(v)))+'\n')
                if now-last_print>=5:
                    print(f'往返中：位置{v.position}，电流{v.current_ma:g}mA，换向{turns}次',flush=True);last_print=now
    except KeyboardInterrupt:
        r['status']='stopped';print('收到停止请求。',flush=True)
    except BaseException as exc:
        code=1;r.update(status='failed',error=str(exc));print(str(exc),flush=True)
    finally:
        if b:
            if original is not None:
                try:
                    b.disable();r['torque_disabled_confirmed']=True
                    for a,value in original.items():b.write(a,value)
                    r['final']=asdict(b.feedback());print('已确认卸力。',flush=True)
                except Exception as exc:
                    code=1;r.update(status='failed',cleanup_error=str(exc));print(str(exc),flush=True)
            b.close()
        signal.signal(signal.SIGTERM,old)
        (folder/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2))
        print('记录：'+str(folder/'result.json'),flush=True)
    return code

if __name__=='__main__':raise SystemExit(run())

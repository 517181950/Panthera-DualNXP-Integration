"""按FD连续写控制区驱动一个小角度；保持原运行模式，结束卸力。"""
from datetime import datetime
import json
import time
from tools.servo_travel_calibration import (
    Bus, Settings, CalibrationError, EXPECTED_IDENTITY, ROOT, WIDTHS,
    TICKS_PER_DEG, resolve_port, check_feedback, asdict,
)

# 直接修改此处；运行时不询问参数。
PORT = None
SERVO_ID = 1
DIRECTION = 1
MOVE_DEG = 1.0
DURATION_S = 1.0
OUTPUT_PERCENT = 7.0       # PWM输出上限；模式4不应当作闭环电流限制
CURRENT_COMMAND_MA = 150.0
STOP_CURRENT_MA = 200.0


def run():
    if DIRECTION not in (-1, 1) or not 0 < MOVE_DEG <= 5 or not 0 < DURATION_S <= 2:
        raise ValueError('小动作入口限±方向、最多5度、最多2秒')
    if not 0 < OUTPUT_PERCENT <= 10:
        raise ValueError('输出上限必须在0..10%内')
    settings = Settings(current_limit_ma=CURRENT_COMMAND_MA,max_current_ma=STOP_CURRENT_MA)
    settings.validate()
    report={'status':'starting','samples':[]};bus=None;original=None;code=0
    path=ROOT/'logs'/('servo_fd_move_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    path.parent.mkdir(exist_ok=True)
    try:
        bus=Bus(resolve_port(PORT),SERVO_ID)
        if bus.read(0,5)!=EXPECTED_IDENTITY:
            raise CalibrationError('舵机固件型号与已核对版本不符')
        mode=bus.integer(33,1)
        if mode not in (0,4):raise CalibrationError('小动作仅支持现有位置模式0或4')
        f=bus.feedback();check_feedback(f,settings)
        lower,upper=bus.integer(9,2),bus.integer(11,2)
        target=f.position+DIRECTION*round(MOVE_DEG*TICKS_PER_DEG)
        if not 0 <= lower <= f.position <= upper <= 4095 or not lower < target < upper:
            raise CalibrationError('当前位置或目标不在现有单圈范围内，不自动改模式或限位')
        report.update(initial=asdict(f),mode=mode,target=target)
        original={a:bus.integer(a,WIDTHS[a]) for a in (41,44,46,48)}
        report['original']=original;bus.writes_allowed=True
        bus.disable()
        bus.write(48,min(original[48],round(OUTPUT_PERCENT*10)))
        bus.write_motion(f.position,settings)
        bus.write(40,1)
        bus.write_motion(target,settings)
        print(f'模式{mode}：{f.position} → {target}，小角度动作开始。',flush=True)
        start=time.monotonic();previous=start;stable=0
        while time.monotonic()-start < DURATION_S:
            time.sleep(.02)
            now=time.monotonic();v=bus.feedback()
            report['samples'].append(dict(t=now-start,**asdict(v)))
            check_feedback(v,settings)
            if now-previous>.1:raise CalibrationError('反馈采样间隔过长')
            previous=now
            travel=DIRECTION*(v.position-f.position)
            if travel < -4 or travel > abs(target-f.position)+8:
                raise CalibrationError('实际运动方向或行程异常')
            stable=stable+1 if abs(v.position-target)<=2 else 0
            if stable>=3:
                report['status']='reached';break
        else:raise CalibrationError('未在限定时间到达目标，已停止尝试')
    except BaseException as exc:
        code=1;report.update(status='failed',error=str(exc));print(str(exc),flush=True)
    finally:
        if bus:
            if original is not None:
                try:
                    bus.disable()
                    report['torque_disabled_confirmed']=True
                    for a,value in original.items():bus.write(a,value)
                    report['final']=asdict(bus.feedback())
                except Exception as exc:
                    code=1;report['status']='failed';report['cleanup_error']=str(exc)
                    print('卸力/恢复未完全确认：'+str(exc),flush=True)
            bus.close()
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(f"结果：{report['status']}；记录：{path}",flush=True)
    return code


if __name__=='__main__':raise SystemExit(run())

"""按FD方式、保留现有位置模式，低输出搜索单侧挡块；结束卸力。"""
from datetime import datetime
import json
from tools.servo_travel_calibration import (
    Bus, Settings, CalibrationError, EXPECTED_IDENTITY, ROOT, WIDTHS,
    TICKS_PER_DEG, resolve_port, check_feedback, establish_reference, seek, asdict,
)

PORT = None
SERVO_ID = 1
DIRECTION = 1
MAX_TRAVEL_DEG = 30.0
TIMEOUT_S = 45.0
OUTPUT_PERCENT = 7.0
CURRENT_COMMAND_MA = 150.0   # 模式4下不代表闭环电流限幅
CONTACT_MA = 100.0
STOP_CURRENT_MA = 200.0
CONTACT_SECONDS = .12


def run():
    c=Settings(direction=DIRECTION,max_travel_deg=MAX_TRAVEL_DEG,timeout_s=TIMEOUT_S,
               torque_percent=OUTPUT_PERCENT,current_limit_ma=CURRENT_COMMAND_MA,
               contact_ma=CONTACT_MA,max_current_ma=STOP_CURRENT_MA,contact_s=CONTACT_SECONDS)
    c.validate()
    if OUTPUT_PERCENT>10:raise ValueError('此入口输出上限不超过10%')
    folder=ROOT/'logs'/('servo_fd_contact_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True)
    report=dict(status='starting',settings=asdict(c));bus=None;original=None;code=0
    try:
        bus=Bus(resolve_port(PORT),SERVO_ID)
        if bus.read(0,5)!=EXPECTED_IDENTITY:raise CalibrationError('固件型号与已核对版本不符')
        mode=bus.integer(33,1)
        if mode not in (0,4):raise CalibrationError('仅支持原位置模式0或4，不自动切换')
        f=bus.feedback();check_feedback(f,c)
        lo,hi=bus.integer(9,2),bus.integer(11,2)
        if not 0<=lo<=f.position<=hi<=4095 or lo==hi:
            raise CalibrationError('此入口需处于现有单圈范围，不修改坐标和限位')
        original={a:bus.integer(a,WIDTHS[a]) for a in (41,44,46,48)}
        report.update(initial=asdict(f),mode=mode,original=original)
        bus.writes_allowed=True;bus.disable()
        bus.write(48,min(original[48],round(OUTPUT_PERCENT*10)))
        f=establish_reference(bus,c,report)
        print(f'模式{mode}，从{f.position}开始，方向{DIRECTION}，最多{MAX_TRAVEL_DEG}°。Ctrl+C停止。',flush=True)
        with (folder/'samples.jsonl').open('w',buffering=1) as log:
            endpoint=seek(bus,c,DIRECTION,lo,hi,lambda row:log.write(json.dumps(row)+'\n'),expected_start=f.position)
        report.update(status='completed_candidate',zero_raw_ticks=f.position,endpoint=endpoint,
                      travel_deg=DIRECTION*(endpoint['raw_ticks']-f.position)/TICKS_PER_DEG)
        print(f"疑似挡块：{endpoint['raw_ticks']}，行程{report['travel_deg']:.2f}°；已卸力，需现场确认。",flush=True)
    except BaseException as exc:
        code=1;report.update(status='failed',error=str(exc));print(str(exc),flush=True)
    finally:
        if bus:
            if original is not None:
                try:
                    bus.disable();report['torque_disabled_confirmed']=True
                    for a,value in original.items():bus.write(a,value)
                    report['final']=asdict(bus.feedback())
                except Exception as exc:
                    code=1;report.update(status='failed',cleanup_error=str(exc))
                    print('卸力/恢复未完全确认：'+str(exc),flush=True)
            bus.close()
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('记录：'+str(folder/'result.json'),flush=True)
    return code


if __name__=='__main__':raise SystemExit(run())

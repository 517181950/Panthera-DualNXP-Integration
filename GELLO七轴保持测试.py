"""ID7当前位置保持，模式0电流上限约1A；仅临时参数，结束卸力恢复。"""
from tools.servo_travel_calibration import *
from types import SimpleNamespace

NODE = 7
CURRENT_LIMIT_MA = 1000.0
DURATION_SECONDS = 20.0
MAX_DEVIATION_DEG = 10.0


def run():
    folder=ROOT/'logs'/('gello_id7_hold_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report={'status':'starting'};b=None;original={};changed=False;code=0
    c=SimpleNamespace(max_current_ma=CURRENT_LIMIT_MA,max_temperature_c=50.0)
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    previous=signal.signal(signal.SIGTERM,stop)
    try:
        b=Bus(resolve_port(),node=NODE)
        if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(55,1)!=1 or b.integer(40,1)!=0:
            raise CalibrationError('型号、锁定或卸力状态不符')
        deadline=time.monotonic()+10
        while True:
            try:
                before=establish_reference(b,c,report);break
            except CalibrationError as exc:
                if '机构仍在运动' not in str(exc) or time.monotonic()>=deadline:raise
                time.sleep(.05)
        if not 0<=before.position<=4095:raise CalibrationError('此次保持仅支持当前单圈坐标')
        for a in (33,41,42,44,46,48):original[a]=b.integer(a,WIDTHS[a])
        if original[33] not in (0,4):raise CalibrationError('原模式不适用')
        b.writes_allowed=True;changed=True;b.write(33,0)
        after=establish_reference(b,c,report)
        if abs(after.position-before.position)>2:raise CalibrationError('模式切换后坐标改变，拒绝使能')
        target=after.position;limit=math.floor(CURRENT_LIMIT_MA/CURRENT_UNIT_MA)
        data=bytes([1])+encode_position(target).to_bytes(2,'little')+limit.to_bytes(2,'little')+(1).to_bytes(2,'little')
        _,status=b.exchange(3,bytes([41])+data,0)
        if status or b.read(41,7)!=data:raise CalibrationError('目标写入或回读失败')
        report.update(target=target,current_limit_ma=limit*CURRENT_UNIT_MA,original=original)
        b.write(40,1)
        print(f'ID7已使能保持：目标{target}，电流限幅{limit*CURRENT_UNIT_MA:g}mA，{DURATION_SECONDS:g}秒后卸力。',flush=True)
        started=last=last_print=time.monotonic();peak=0
        with (folder/'samples.jsonl').open('w',buffering=1) as log:
            while time.monotonic()-started<DURATION_SECONDS:
                f=b.feedback();now=time.monotonic()
                log.write(json.dumps({'t':now,'target':target,**asdict(f)})+'\n')
                peak=max(peak,abs(f.current_ma));report['peak_current_ma']=peak
                check_feedback(f,c)
                if now-last>.1:raise CalibrationError('采样延迟超过100ms')
                if abs(f.position-target)>MAX_DEVIATION_DEG*TICKS_PER_DEG:raise CalibrationError('偏离保持位置超过10度')
                if b.integer(40,1)!=1:raise CalibrationError('舵机已退出使能')
                last=now
                if now-last_print>=1:
                    print(f'剩余{DURATION_SECONDS-(now-started):.0f}s，位置{f.position}，电流{f.current_ma:g}mA，温度{f.temperature_c}℃',flush=True);last_print=now
                time.sleep(.02)
        report['status']='hold_test_complete'
    except BaseException as e:
        report.update(status='stopped',error=str(e));code=1;print('停止：'+str(e),flush=True)
    finally:
        if b:
            if changed:
                try:
                    b.disable();report['torque_disabled_confirmed']=True
                    for a,v in original.items():b.write(a,signed(v) if a==42 else v)
                    report['final']=asdict(b.feedback())
                    print('ID7已确认卸力，原参数已恢复。',flush=True)
                except Exception as e:
                    code=1;report['cleanup_error']=str(e);print('停止/恢复异常：'+str(e),flush=True)
            b.close()
        signal.signal(signal.SIGTERM,previous)
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('记录：'+str(folder/'result.json'),flush=True)
    return code

if __name__=='__main__':raise SystemExit(run())

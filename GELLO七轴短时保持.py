"""七轴当前位置短时保持；不使用历史零位或历史姿态。"""
from tools.servo_travel_calibration import *
from types import SimpleNamespace
IDS=list(range(1,8))
CURRENT_MA=1800.0
STOP_MA=2000.0
DURATION_SECONDS=None  # 无固定时长，Ctrl+C或异常停止

def run():
    folder=ROOT/'logs'/('gello_seven_short_hold_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report={'status':'starting','output_ma':CURRENT_MA,'stop_ma':STOP_MA,'duration_s':DURATION_SECONDS}
    limits=SimpleNamespace(max_current_ma=STOP_MA,max_temperature_c=50.)
    b=None;changed=False;packet_log=(folder/'packets.jsonl').open('w',buffering=1)
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    previous=signal.signal(signal.SIGTERM,stop)
    try:
        b=Bus(resolve_port());exchange=b.exchange
        def traced(instruction,params,length):
            data,status=exchange(instruction,params,length)
            packet_log.write(json.dumps({'t':time.monotonic(),**b.recent_packets[-1]})+'\n')
            return data,status
        b.exchange=traced
        for n in IDS:
            b.node=n
            if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(40,1)!=0 or b.integer(55,1)!=1:
                raise CalibrationError(f'ID{n}型号/卸力/锁状态不符')
            if b.integer(33,1) not in (0,4) or not b.integer(18,1)&16:
                raise CalibrationError(f'ID{n}模式或角度反馈配置不符')
            check_feedback(b.feedback(),limits)
        b.writes_allowed=True;changed=True;targets={};limit=math.floor(CURRENT_MA/CURRENT_UNIT_MA)
        def command(n,position):
            b.node=n
            data=bytes([0])+encode_position(position).to_bytes(2,'little')+limit.to_bytes(2,'little')+(3).to_bytes(2,'little')
            _,status=b.exchange(3,bytes([41])+data,0)
            if status or b.read(41,7)!=data:raise CalibrationError(f'ID{n}保持目标回读失败')
        for n in IDS:
            b.node=n;before=establish_reference(b,limits,{})
            if b.integer(33,1)!=0:b.write(33,0)
            if b.integer(9,2)!=0:b.write(9,0)
            if b.integer(11,2)!=0:b.write(11,0)
            after=establish_reference(b,limits,{})
            delta=after.position-before.position
            if abs(delta-round(delta/4096)*4096)>2:raise CalibrationError(f'ID{n}模式切换时机构移动')
            targets[n]=after.position;command(n,after.position)
        start=time.monotonic()
        with (folder/'samples.jsonl').open('w',buffering=1) as log:
            def sample():
                for n in IDS:
                    b.node=n;f=b.feedback()
                    log.write(json.dumps({'t':time.monotonic()-start,'id':n,'target':targets[n],**asdict(f)})+'\n')
                    check_feedback(f,limits)
            for n in IDS:
                b.node=n;f=b.feedback();check_feedback(f,limits)
                # Hold the freshest position rather than an earlier setup sample.
                targets[n]=f.position;command(n,f.position);b.write(40,1);sample()
            report['targets']=targets.copy()
            duration_note='持续保持，无定时卸力' if DURATION_SECONDS is None else f'{DURATION_SECONDS:g}秒后卸力'
            print(f'七轴已使能保持当前位置；每轴输出指令{limit*CURRENT_UNIT_MA:g}mA，{STOP_MA:g}mA停止，{duration_note}。',flush=True)
            while DURATION_SECONDS is None or time.monotonic()-start<DURATION_SECONDS:
                began=time.monotonic();sample()
                for n in IDS:
                    b.node=n
                    if b.integer(33,1)!=0 or b.integer(44,2)!=limit or b.integer(40,1)!=1:
                        raise CalibrationError(f'ID{n}模式/电流指令/使能变化')
                if time.monotonic()-began>.2:raise CalibrationError('反馈采集超时')
                time.sleep(.02)
            report['status']='completed'
    except BaseException as e:
        report.update(status='stopped',error=str(e));print('停止：'+str(e),flush=True)
        if b:report['fault_context']={'id':b.node,'recent_packets':list(b.recent_packets)}
    finally:
        if b:
            errors={};final={}
            if changed:
                for n in IDS:
                    b.node=n
                    try:b.disable()
                    except Exception as e:errors[n]=str(e)
            for n in IDS:
                b.node=n
                try:final[n]=b.integer(40,1,allow_fault=True)
                except Exception as e:errors[n]=str(e)
            report.update(final_torque=final,cleanup_errors=errors)
            print('最终七轴使能状态：'+str(final),flush=True);b.close()
        packet_log.close();signal.signal(signal.SIGTERM,previous)
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print('记录：'+str(folder/'result.json'),flush=True)
    return 0 if report['status']=='completed' and all(report.get('final_torque',{}).get(n)==0 for n in IDS) else 1

if __name__=='__main__':raise SystemExit(run())

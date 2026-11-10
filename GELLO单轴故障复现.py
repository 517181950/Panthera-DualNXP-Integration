"""单独连接ID2：保持当前姿态并记录原始应答，不使用旧标定坐标。"""
from tools.servo_travel_calibration import *
from types import SimpleNamespace
NODE=2
CURRENT_MA=1200.0
STOP_MA=1800.0
HOLD_SECONDS=60.0
CONNECTED_IDS=[2]
POLL_OTHER_SERVOS=False
LOG_PREFIX="gello_id2_isolated_hold_"

def run():
    folder=ROOT/'logs'/(LOG_PREFIX+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir();report={'status':'starting','connected_ids':list(CONNECTED_IDS),'enabled_ids':[NODE],'poll_other_servos':POLL_OTHER_SERVOS};b=None;changed=False
    limits=SimpleNamespace(max_current_ma=STOP_MA,max_temperature_c=50.)
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    previous=signal.signal(signal.SIGTERM,stop)
    try:
        b=Bus(resolve_port(),node=NODE)
        for n in CONNECTED_IDS:
            b.node=n
            if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(40,1)!=0:
                raise CalibrationError(f'ID{n}型号不符或未卸力')
        b.node=NODE
        if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(40,1)!=0 or b.integer(55,1)!=1:
            raise CalibrationError('ID2型号/使能/锁状态不符')
        if b.integer(33,1) not in (0,4):raise CalibrationError('模式不适用')
        if not b.integer(18,1)&16:raise CalibrationError('全角度反馈未启用')
        before=establish_reference(b,limits,{})
        b.writes_allowed=True;changed=True
        if b.integer(33,1)!=0:b.write(33,0)
        if b.integer(9,2)!=0:b.write(9,0)
        if b.integer(11,2)!=0:b.write(11,0)
        after=establish_reference(b,limits,{})
        delta=after.position-before.position
        if abs(delta-round(delta/4096)*4096)>2:raise CalibrationError('模式切换时位置移动')
        target=after.position;limit=math.floor(CURRENT_MA/CURRENT_UNIT_MA)
        payload=bytes([0])+encode_position(target).to_bytes(2,'little')+limit.to_bytes(2,'little')+(3).to_bytes(2,'little')
        _,status=b.exchange(3,bytes([41])+payload,0)
        if status or b.read(41,7)!=payload:raise CalibrationError('保持目标回读失败')
        report.update(target=target,before=asdict(before),after=asdict(after),current_limit_ma=limit*CURRENT_UNIT_MA)
        b.write(40,1)
        print(f'ID2已使能，保持当前位置{target}，最多{HOLD_SECONDS:g}秒；异常立即卸力。',flush=True)
        start=last_print=time.monotonic()
        with (folder/'packets.jsonl').open('w',buffering=1) as packets, (folder/'samples.jsonl').open('w',buffering=1) as log:
            while time.monotonic()-start<HOLD_SECONDS:
                began=time.monotonic()
                data,status=b.exchange(2,[56,15],15)
                packets.write(json.dumps({'t':began-start,**b.recent_packets[-1]})+'\n')
                f=Feedback.decode(data,status)
                log.write(json.dumps({'t':began-start,'target':target,**asdict(f)})+'\n')
                check_feedback(f,limits)
                for addr,size,expected in [(33,1,0),(44,2,limit),(40,1,1),(65,1,0)]:
                    data,status=b.exchange(2,[addr,size],size)
                    packets.write(json.dumps({'t':time.monotonic()-start,**b.recent_packets[-1]})+'\n')
                    if status:raise CalibrationError(f'ID2读取{addr}返回状态0x{status:02x}')
                    if int.from_bytes(data,'little')!=expected:raise CalibrationError(f'ID2寄存器{addr}值变化：{data.hex()}')
                if POLL_OTHER_SERVOS:
                    for n in CONNECTED_IDS:
                        if n==NODE:continue
                        b.node=n
                        for addr,size in [(56,15),(33,1),(44,2),(40,1)]:
                            data,status=b.exchange(2,[addr,size],size)
                            packets.write(json.dumps({'t':time.monotonic()-start,**b.recent_packets[-1]})+'\n')
                            if status:raise CalibrationError(f'ID{n}读取{addr}返回状态0x{status:02x}')
                            if addr==56:check_feedback(Feedback.decode(data,status),limits)
                            if addr==40 and data[0]!=0:raise CalibrationError(f'ID{n}意外使能')
                    b.node=NODE
                if time.monotonic()-began>.2:raise CalibrationError('采集超时')
                if began-last_print>=5:
                    print(f'位置{f.position}，电流{f.current_ma:g}mA，电压{f.voltage_v:g}V，温度{f.temperature_c}℃',flush=True);last_print=began
                time.sleep(.02)
        report['status']='completed'
    except BaseException as e:
        report.update(status='stopped',error=str(e));print('停止：'+str(e),flush=True)
        if b:report['fault_context']={'id':b.node,'recent_packets':list(b.recent_packets)}
    finally:
        if b:
            try:
                b.node=NODE
                if changed:b.disable()
                report['final_torque']=b.integer(40,1,allow_fault=True)
                report['final_feedback']=asdict(b.feedback())
                report['all_final_torque']={}
                for n in CONNECTED_IDS:
                    b.node=n
                    report['all_final_torque'][n]=b.integer(40,1,allow_fault=True)
                b.node=NODE
                print('ID2最终扭矩使能读回：'+str(report['final_torque']),flush=True)
            except Exception as e:report['cleanup_error']=str(e);print('卸力未确认：'+str(e),flush=True)
            finally:b.close()
        signal.signal(signal.SIGTERM,previous)
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('记录：'+str(folder/'result.json'),flush=True)
    return 0 if report['status']=='completed' and report.get('final_torque')==0 else 1

if __name__=='__main__':raise SystemExit(run())

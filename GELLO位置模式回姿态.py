"""模式0位置限流，协调接近人工保存的抬高姿态。无旧目标/模式恢复。"""
from tools.servo_travel_calibration import *
from types import SimpleNamespace
from GELLO力反馈参数 import RAW_FRAME_OFFSETS

POSE=ROOT/'logs/gello_clearance_pose_20261109_211430_763507/pose.json'
CALIBRATION=ROOT/'logs/gello_reference_zero_20261109_210927_565834/calibration.json'
CURRENT_LIMIT_MA=1200.0  # 短时抬升试验
CURRENT_STOP_MA=1800.0  # 单轴峰值停止阈值，不是七轴电流总和
STEP_TICKS=2
STEP_PERIOD_S=.05
FOLLOW_TICKS=None  # 不按跟随误差退出，保留无位移卡滞检测
TIMEOUT_S=25.0
HOLD_SECONDS=2.0
HOLD_CURRENT_POSITION=False
TOTAL_ENABLED_SECONDS=None
STALL_SECONDS=.8


def motion_data(position):
    # Host sends a bounded ramp; avoid repeatedly restarting an internal accel ramp.
    return bytes([0])+encode_position(position).to_bytes(2,'little')+math.floor(CURRENT_LIMIT_MA/CURRENT_UNIT_MA).to_bytes(2,'little')+(3).to_bytes(2,'little')


def run():
    folder=ROOT/'logs'/('gello_position_pose_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report={'status':'starting','mode':0,'current_limit_ma':math.floor(CURRENT_LIMIT_MA/6.5)*6.5,
            'current_stop_ma':CURRENT_STOP_MA};b=None;changed=False;log=None;code=0
    limits=SimpleNamespace(max_current_ma=CURRENT_STOP_MA,max_temperature_c=50.)
    joints={j['id']:j for j in json.loads(CALIBRATION.read_text())['joints']}
    pose=json.loads(POSE.read_text())
    if pose['status']!='stable':raise CalibrationError('无稳定姿态记录')
    destination={p['id']:p['raw_ticks'] for p in pose['pose']}
    shifts={n:RAW_FRAME_OFFSETS[n-1] for n in range(1,8)}
    goals={};enabled=set()
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    previous=signal.signal(signal.SIGTERM,stop)
    try:
        b=Bus(resolve_port());initial={}
        for n in range(1,8):
            b.node=n
            if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(40,1)!=0 or b.integer(55,1)!=1:
                raise CalibrationError(f'ID{n}身份/卸力/锁状态不符')
            if b.integer(33,1) not in (0,4) or not b.integer(18,1)&16:raise CalibrationError(f'ID{n}模式/反馈不符')
            if (b.integer(9,2),b.integer(11,2)) not in ((0,0),(0,4095)):raise CalibrationError(f'ID{n}自定义限位，不覆盖')
            f=b.feedback();check_feedback(f,limits);initial[n]=f.position-shifts[n]
            j=joints[n]
            if not j['min_ticks']<=initial[n]<=j['max_ticks'] or not j['min_ticks']<=destination[n]<=j['max_ticks']:
                raise CalibrationError(f'ID{n}当前或目标坐标不在人工行程')
        if HOLD_CURRENT_POSITION:destination=initial.copy()
        report['initial']=initial;report['destination']=destination
        b.writes_allowed=True
        def command(n,logical):
            b.node=n;data=motion_data(int(logical)+shifts[n])
            _,status=b.exchange(3,bytes([41])+data,0)
            if status or b.read(41,7)!=data:raise CalibrationError(f'ID{n}位置/限流回读失败')
        for n in range(1,8):
            b.node=n;before=establish_reference(b,limits,{})
            changed=True
            # Already configured joints do not need another mode/limit transition.
            if b.integer(33,1)!=0:b.write(33,0)
            if b.integer(9,2)!=0:b.write(9,0)
            if b.integer(11,2)!=0:b.write(11,0)
            after=establish_reference(b,limits,{})
            delta=after.position-before.position;turn=round(delta/4096)*4096
            if abs(delta-turn)>2:raise CalibrationError(f'ID{n}切换模式后坐标不稳定')
            shifts[n]+=turn;goals[n]=after.position-shifts[n];command(n,goals[n])
        report['raw_frame_offsets']=shifts.copy()
        log=(folder/'samples.jsonl').open('w',buffering=1)
        last_sample=time.monotonic();enabled_at=None
        progress_pos={};progress_at={}
        def sample():
            nonlocal last_sample
            if enabled_at is not None and TOTAL_ENABLED_SECONDS is not None and time.monotonic()-enabled_at>=TOTAL_ENABLED_SECONDS:
                raise CalibrationError('5秒短时测试结束，卸力')
            readings={}
            for n in range(1,8):
                b.node=n;f=b.feedback();logical=f.position-shifts[n];readings[n]=logical
                log.write(json.dumps({'t':time.monotonic(),'id':n,'logical':logical,'goal':goals[n],**asdict(f)})+'\n')
                try:check_feedback(f,limits)
                except CalibrationError as e:raise CalibrationError(f'ID{n}: {e}')
                if not joints[n]['min_ticks']<=logical<=joints[n]['max_ticks']:raise CalibrationError(f'ID{n}超出人工行程')
                if n in enabled:
                    if b.integer(33,1)!=0 or b.integer(44,2)!=math.floor(CURRENT_LIMIT_MA/6.5) or b.integer(40,1)!=1:
                        raise CalibrationError(f'ID{n}模式/限流/使能变化')
                    if FOLLOW_TICKS is not None and abs(logical-goals[n])>FOLLOW_TICKS:raise CalibrationError(f'ID{n}跟随误差超过约3度，位置{logical}目标{goals[n]}')
                    now=time.monotonic()
                    if n not in progress_pos or abs(logical-progress_pos[n])>=2 or abs(logical-goals[n])<=12:
                        progress_pos[n]=logical;progress_at[n]=now
                    elif not HOLD_CURRENT_POSITION and now-progress_at[n]>STALL_SECONDS:
                        raise CalibrationError(f'ID{n}持续{STALL_SECONDS:g}秒无有效位移，停止推顶')
            now=time.monotonic()
            if now-last_sample>.2:raise CalibrationError('采样间隔超时')
            last_sample=now
            return readings
        enabled_at=time.monotonic()
        for n in range(1,8):
            b.node=n;f=b.feedback();goals[n]=f.position-shifts[n]
            command(n,goals[n]);b.write(40,1);enabled.add(n);sample()
        print('七轴已使能，保持刚读取的当前位置。' if HOLD_CURRENT_POSITION else '模式0位置控制已使能，开始缓慢接近保存的抬高姿态。',flush=True)
        start=goals.copy();steps=max(1,math.ceil(max(abs(destination[n]-start[n]) for n in start)/STEP_TICKS))
        began=last_print=time.monotonic()
        for i in range(1,steps+1):
            tick=time.monotonic();readings=sample()
            if tick-began>=TIMEOUT_S:raise CalibrationError('抬升超时')
            for n in range(1,8):
                goal=round(start[n]+(destination[n]-start[n])*i/steps)
                if goal!=goals[n]:command(n,goal);goals[n]=goal
            if tick-last_print>=2:
                print('当前位置：'+str(readings),flush=True);last_print=tick
            time.sleep(max(0,STEP_PERIOD_S-(time.monotonic()-tick)))
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            readings=sample()
            if all(abs(readings[n]-destination[n])<=6 for n in destination):break
            time.sleep(.02)
        else:raise CalibrationError('未到达保存姿态')
        report['arrived']=readings
        print('反馈到达目标，持续使能保持，Ctrl+C停止。' if HOLD_SECONDS is None else f'反馈到达目标，保持{HOLD_SECONDS:g}秒后卸力，请托住。',flush=True)
        deadline=float("inf") if HOLD_SECONDS is None else time.monotonic()+HOLD_SECONDS
        while time.monotonic()<deadline:sample();time.sleep(.02)
        report['status']='pose_reached'
    except BaseException as e:
        report.update(status='stopped',error=str(e));print('停止：'+str(e),flush=True);code=1
        if b:report['fault_context']={'id':b.node,'recent_packets':list(getattr(b,'recent_packets',[]))}
    finally:
        if b:
            if changed:
                errors={};final={}
                for n in range(1,8):
                    b.node=n
                    try:b.disable()
                    except Exception as e:errors[n]=str(e)
                for n in range(1,8):
                    b.node=n
                    try:
                        final[n]=b.integer(40,1,allow_fault=True)
                        if final[n]!=0:raise CalibrationError('最终使能仍开启')
                    except Exception as e:errors[n]=str(e)
                report['final_torque']=final;report['disable_errors']=errors
                if errors:print('卸力未确认，请切断主手电源：'+str(errors),flush=True);code=1
                else:print('全部7轴最终卸力已读回确认；不恢复旧运动寄存器。',flush=True)
            b.close()
        if log:log.close()
        signal.signal(signal.SIGTERM,previous)
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('记录：'+str(folder/'result.json'),flush=True)
    return code

if __name__=='__main__':raise SystemExit(run())

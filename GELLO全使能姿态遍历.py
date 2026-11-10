"""全部关节保持，缓慢回到已记录抬高姿态，再逐轴小范围遍历。参数在此预设。"""
from tools.servo_travel_calibration import *

IDS = list(range(1, 8))
HALF_RANGE_DEG = 10.0
STEP_TICKS = 2
STEP_INTERVAL = .05
# HD-1910-C001 A/0: 7.4V rated 900mA, stall 2A.
# 8V continuous holding is unspecified; this is a bounded test stop, not a drive current cap.
STOP_CURRENT_MA = 900.0
LIFT_ONLY = False
HOLD_SECONDS = 10.0
RATED_REFERENCE_MA = 900.0
OVER_RATED_BUDGET_SECONDS = .2
FOLLOW_ERROR_TICKS = 16
TEST_DURATION_SECONDS = None
SINGLE_AXIS = None
SINGLE_AXIS_DELTA_DEG = 5.0
POSE = ROOT/'logs/gello_clearance_pose_20261109_211430_763507/pose.json'
CALIBRATION = ROOT/'logs/gello_reference_zero_20261109_210927_565834/calibration.json'


def targets(center, joints):
    result=[]
    margin=math.ceil(5*TICKS_PER_DEG)
    for n in IDS:
        lo=max(joints[n]['min_ticks']+margin,center[n]-round(HALF_RANGE_DEG*TICKS_PER_DEG))
        hi=min(joints[n]['max_ticks']-margin,center[n]+round(HALF_RANGE_DEG*TICKS_PER_DEG))
        if lo>=hi:raise CalibrationError(f'ID{n}没有可用测试区间')
        for value in (hi,lo,center[n]):
            result.append((n,{**center,n:value}))
    return result


def run():
    folder=ROOT/'logs'/('gello_all_hold_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report={'status':'starting','completed':[]};original={};b=None;log=None;code=0
    from types import SimpleNamespace
    if not 0 < STOP_CURRENT_MA <= (1500 if LIFT_ONLY else 900):
        raise ValueError("遍历上限900mA；短时抬升测试上限1500mA")
    if not 1 <= FOLLOW_ERROR_TICKS <= 34:
        raise ValueError('短时测试跟随容差上限34 tick，约3度')
    if TEST_DURATION_SECONDS is not None and not 0 < TEST_DURATION_SECONDS <= 5:
        raise ValueError('短时峰值试验最长5秒')
    if OVER_RATED_BUDGET_SECONDS is None and (not LIFT_ONLY or TEST_DURATION_SECONDS is None):
        raise ValueError('取消900mA时限仅适用于有总时限的短时抬升测试')
    c=SimpleNamespace(max_current_ma=STOP_CURRENT_MA, max_temperature_c=50.0)
    center={p['id']:p['raw_ticks'] for p in json.loads(POSE.read_text())['pose']}
    joints={j['id']:j for j in json.loads(CALIBRATION.read_text())['joints']}
    if SINGLE_AXIS is not None and (not LIFT_ONLY or SINGLE_AXIS not in IDS or not 0 < abs(SINGLE_AXIS_DELTA_DEG) <= 5):
        raise ValueError('单轴试验仅允许抬升模式、最多5度')
    sequence=[] if LIFT_ONLY else targets(center,joints)
    report['settings']={'stop_current_ma':STOP_CURRENT_MA,'lift_only':LIFT_ONLY,'follow_error_ticks':FOLLOW_ERROR_TICKS,
                        'over_rated_budget_s':OVER_RATED_BUDGET_SECONDS,
                        'test_duration_s':TEST_DURATION_SECONDS}
    overload={n:0.0 for n in IDS};previous_current={n:0.0 for n in IDS};current_time={}
    report['over_rated_seconds']=overload
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    old=signal.signal(signal.SIGTERM,stop)
    try:
        b=Bus(resolve_port());initial={}
        for n in IDS:
            b.node=n
            if b.read(0,5)!=EXPECTED_IDENTITY or b.integer(33,1)!=4 or b.integer(55,1)!=1 or not b.integer(18,1)&16:
                raise CalibrationError(f'ID{n}型号/模式/锁/反馈配置不符')
            if b.integer(40,1)!=0:raise CalibrationError(f'ID{n}已经使能')
            f=b.feedback();check_feedback(f,c);initial[n]=f.position
            if not joints[n]['min_ticks']<=f.position<=joints[n]['max_ticks']:raise CalibrationError(f'ID{n}超出人工行程')
        report['initial']=initial
        if SINGLE_AXIS is not None:
            test_target=initial[SINGLE_AXIS]+round(SINGLE_AXIS_DELTA_DEG*TICKS_PER_DEG)
            if not joints[SINGLE_AXIS]['min_ticks']<=test_target<=joints[SINGLE_AXIS]['max_ticks']:
                raise CalibrationError('单轴目标超出人工行程')
        b.writes_allowed=True
        def command(n,pos):
            b.node=n
            data=bytes([0])+encode_position(pos).to_bytes(2,'little')+bytes(4)
            _,status=b.exchange(3,bytes([41])+data,0)
            if status or b.read(41,7)!=data:raise CalibrationError(f'ID{n}目标写入失败')
        goal={};shift={}
        # Stage all current-position targets before enabling any joint.
        for n in IDS:
            b.node=n
            original[n]={a:b.integer(a,WIDTHS[a]) for a in (11,9,41,44,46)}
            before=b.feedback().position
            b.write(9,0);b.write(11,0)
            after=b.feedback().position
            delta=after-before;shift[n]=round(delta/4096)*4096
            if abs(delta-shift[n])>2:raise CalibrationError(f'ID{n}切换坐标发生非整圈变化')
            goal[n]=after;command(n,after)
        log=(folder/'samples.jsonl').open('w',buffering=1)
        enabled=set();last_sample=time.monotonic();test_started=last_sample
        def sample():
            nonlocal last_sample
            if TEST_DURATION_SECONDS is not None and time.monotonic()-test_started>=TEST_DURATION_SECONDS:
                raise CalibrationError('5秒短时抬升测试结束，卸力')
            readings={}
            for n in IDS:
                b.node=n;f=b.feedback();readings[n]=f
                log.write(json.dumps({'t':time.monotonic(),'id':n,'goal':goal[n],**asdict(f)})+'\n')
                now_current=time.monotonic()
                dt=now_current-current_time.get(n,now_current)
                if max(abs(f.current_ma),previous_current[n])>RATED_REFERENCE_MA:
                    overload[n]+=dt
                current_time[n]=now_current;previous_current[n]=abs(f.current_ma)
                if OVER_RATED_BUDGET_SECONDS is not None and overload[n]>=OVER_RATED_BUDGET_SECONDS:
                    raise CalibrationError(f'ID{n}超过900mA累计{overload[n]:.3f}秒，结束短时测试')
                try:check_feedback(f,c)
                except CalibrationError as e:raise CalibrationError(f'ID{n}: {e}')
                if not joints[n]['min_ticks']+shift[n]<=f.position<=joints[n]['max_ticks']+shift[n]:raise CalibrationError(f'ID{n}超出人工行程')
                if n in enabled and (b.integer(40,1)!=1 or abs(f.position-goal[n])>FOLLOW_ERROR_TICKS):raise CalibrationError(f'ID{n}保持/跟随偏差过大，位置{f.position}目标{goal[n]}')
            now=time.monotonic()
            if now-last_sample>.2:raise CalibrationError('全轴采样延迟超过200ms')
            last_sample=now
            return readings
        for n in IDS:
            b.node=n
            # Refresh immediately before enabling to avoid chasing a stale pose.
            goal[n]=b.feedback().position;command(n,goal[n]);b.write(40,1);enabled.add(n);sample()
        if SINGLE_AXIS is not None:
            center={n:goal[n]-shift[n] for n in IDS}
            center[SINGLE_AXIS]+=round(SINGLE_AXIS_DELTA_DEG*TICKS_PER_DEG)
            if not joints[SINGLE_AXIS]['min_ticks']<=center[SINGLE_AXIS]<=joints[SINGLE_AXIS]['max_ticks']:
                raise CalibrationError('单轴目标超出人工行程')
            report['commanded_pose']=center.copy()
            print(f'全部7轴保持；仅ID{SINGLE_AXIS}移动{SINGLE_AXIS_DELTA_DEG:g}度，目标{center[SINGLE_AXIS]}。',flush=True)
        else:
            print('全部7轴已使能，开始小步协调抬高。',flush=True)
        def move(destination,label):
            start=goal.copy();dest={n:destination[n]+shift[n] for n in IDS}
            steps=max(1,math.ceil(max(abs(dest[n]-start[n]) for n in IDS)/STEP_TICKS))
            for i in range(1,steps+1):
                began=time.monotonic();sample()
                for n in IDS:
                    value=round(start[n]+(dest[n]-start[n])*i/steps)
                    if value!=goal[n]:command(n,value);goal[n]=value
                time.sleep(max(0,STEP_INTERVAL-(time.monotonic()-began)))
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                readings=sample()
                if all(abs(readings[n].position-dest[n])<=4 for n in IDS):break
                time.sleep(.02)
            else:raise CalibrationError('到位超时')
            report['completed'].append(label)
            print(label+' 到位；最大电流 '+str(max(abs(f.current_ma) for f in readings.values()))+'mA',flush=True)
        move(center,f'ID{SINGLE_AXIS}单轴抬升' if SINGLE_AXIS is not None else '抬高姿态')
        for n,destination in sequence:move(destination,f'ID{n}目标{destination[n]}')
        print(f'动作完成，保持{HOLD_SECONDS:g}秒后卸力，请扶住机构。',flush=True)
        until=time.monotonic()+HOLD_SECONDS
        while time.monotonic()<until:sample();time.sleep(.03)
        report['status']='feedback_lift_complete' if LIFT_ONLY else 'feedback_sweep_complete'
    except BaseException as e:
        report.update(status='stopped',error=str(e));code=1;print('停止：'+str(e),flush=True)
    finally:
        if b:
            errors={}
            # Attempt every joint even if one stop fails; never leave others enabled.
            for n in reversed(list(original)):
                b.node=n
                try:b.disable()
                except Exception as e:errors[n]=str(e)
            report['disable_errors']=errors
            for n,values in original.items():
                if n in errors:continue
                b.node=n
                try:
                    for a,v in values.items():b.write(a,v)
                except Exception as e:report.setdefault('restore_errors',{})[n]=str(e);code=1
            if errors:print('无法确认全部卸力，请切断电源：'+str(errors),flush=True);code=1
            elif original:print('已确认全部7轴卸力。',flush=True)
            b.close()
        if log:log.close()
        signal.signal(signal.SIGTERM,old)
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('记录：'+str(folder/'result.json'),flush=True)
    return code

if __name__=='__main__':raise SystemExit(run())

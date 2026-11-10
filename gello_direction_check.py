"""Positive slave jog, then explicit user-operated master direction capture."""
from gello_haptic_entry import *
from tools.servo_travel_calibration import Bus,resolve_port,asdict,check_feedback
from types import SimpleNamespace
PENDING=ROOT/'gello-haptic-direction-pending.json'
DEMO_DEG=10.0
DEMO_RAMP_SECONDS=4.0


def document():
    if C.DIRECTIONS.exists():return json.loads(C.DIRECTIONS.read_text())
    return {'side':C.SIDE,'ids':list(range(1,7)),'signs':[None]*6,'verified':[False]*6,'evidence':{}}


def read_stable():
    config,joints=setup();b=HapticBus(C,joints)
    try:
        b.open();rows=[];deadline=time.perf_counter()+5
        while time.perf_counter()<deadline:
            rows.append(b.read_all());rows=rows[-5:];time.sleep(.02)
            if len(rows)==5 and np.max(np.ptp(np.asarray([r['ticks'] for r in rows]),axis=0))<=3:
                return rows[-1]
        raise RuntimeError('请先保持GELLO静止')
    finally:b.close()


def demo():
    d=document()
    if d['side']!=C.SIDE or d['ids']!=list(range(1,7)):raise RuntimeError('方向文件对应关系不符')
    pending=[i for i,ok in enumerate(d['verified']) if not ok]
    if not pending:print('六轴方向均已完成核对');return
    axis=pending[0];config,joints=setup();inspect_arm(config)
    config['boards'][C.SIDE]['kp']=TELEOP_CONFIG['boards'][C.SIDE]['kp']
    config['boards'][C.SIDE]['kd']=TELEOP_CONFIG['boards'][C.SIDE]['kd']
    master=read_stable()
    folder=ROOT/'logs'/('gello_direction_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    record={'axis':axis+1,'status':'starting','master_before':master,'side':C.SIDE,'folder':str(folder)}
    PENDING.write_text(json.dumps(record,ensure_ascii=False,indent=2))
    def stop(signum,frame):raise KeyboardInterrupt('停止请求')
    old=signal.signal(signal.SIGTERM,stop)
    try:
        with make_session(config,folder) as s:
            s.configure_encoder_turns(config['boards'][C.SIDE]['encoder_modulo_joints'])
            state=s.state();q0=np.array(state['q_rad']);target=q0.copy();target[axis]+=np.deg2rad(DEMO_DEG)
            bounds=config['boards'][C.SIDE]['soft_limits']
            lower=np.deg2rad(bounds['lower_deg']);upper=np.deg2rad(bounds['upper_deg']);tolerance=np.deg2rad(.5)
            if np.any(q0<lower-tolerance) or np.any(q0>upper+tolerance) or not lower[axis]<=target[axis]<=upper[axis]:
                raise RuntimeError('示范起点或目标超出软限位')
            print(f'原臂J{axis+1}将正向移动{DEMO_DEG:g}度，再返回；GELLO保持卸力，请观察方向。',flush=True)
            peak=q0[axis]
            for label,begin,end,duration in [('正向',q0,target,DEMO_RAMP_SECONDS),('保持',target,target,3.),('返回',target,q0,DEMO_RAMP_SECONDS)]:
                start=time.perf_counter()
                while True:
                    elapsed=time.perf_counter()-start;fraction=min(1.,elapsed/duration)
                    ref=begin+(end-begin)*(fraction*fraction*(3-2*fraction))
                    state=s.state();q=np.asarray(state['q_rad']);peak=max(peak,q[axis])
                    if max(abs(q-ref))>np.deg2rad(5):raise RuntimeError('示范跟随误差超过5度')
                    s.step('direction',label,ref,ZERO,s.args.kp,s.args.kd,state=state)
                    if fraction>=1:break
            if peak-q0[axis]<np.deg2rad(1):raise RuntimeError('原臂示范未观察到足够正向移动')
            record.update(status='awaiting_manual_move',arm_start=q0.tolist(),arm_target=target.tolist(),actual_positive_deg=float(np.rad2deg(peak-q0[axis])))
            print(f'现在请把GELLO第{axis+1}轴朝刚才机械臂正向示范的对应方向转动约5～10度，保持住，再运行方向记录。',flush=True)
    except BaseException as exc:
        record.update(status='failed',error=str(exc));raise
    finally:
        signal.signal(signal.SIGTERM,old)
        PENDING.write_text(json.dumps(record,ensure_ascii=False,indent=2))
        (folder/'direction.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))


def capture():
    p=json.loads(PENDING.read_text())
    if p.get('status')!='awaiting_manual_move' or p['side']!=C.SIDE:raise RuntimeError('需要先完成本轴正向示范')
    current=read_stable();axis=p['axis']-1
    delta=np.array(current['ticks'])-np.array(p['master_before']['ticks'])
    if not 30<=abs(delta[axis])<=1024:raise RuntimeError(f'本轴变化{delta[axis]}tick；请在示范前位置基础上同向转动约5～10度')
    # Axis identity is explicitly confirmed by the user. Passive changes of
    # other joints do not determine this axis sign; retain all deltas as evidence.
    d=document();d['signs'][axis]=int(np.sign(delta[axis]));d['verified'][axis]=True
    d['evidence'][str(axis+1)]={**p,'master_after':current,'delta_ticks':delta.tolist(),
                               'method':'user moved corresponding master axis in demonstrated slave-positive direction'}
    temp=C.DIRECTIONS.with_suffix('.tmp');temp.write_text(json.dumps(d,ensure_ascii=False,indent=2));temp.replace(C.DIRECTIONS)
    p['status']='captured';PENDING.write_text(json.dumps(p,ensure_ascii=False,indent=2))
    print(f'J{axis+1}方向已记录：{d["signs"][axis]:+d}；全部方向：{d["signs"]}',flush=True)

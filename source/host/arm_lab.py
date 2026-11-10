"""Reusable UDP experiment runner; no position/velocity/torque/slew clamps.

Input validity and acquisition freshness are data requirements, not gain shaping.
Experiments never issue hand targets or change physical hand modes.
"""
import argparse
import csv
from collections import deque
from datetime import datetime
import hashlib
import itertools
import json
from pathlib import Path
import time
import arm_runtime
import numpy as np
from robot_sdk import Robot
from arm_calibration_plan import legacy285
from arm_dynamics import Gravity, DEFAULT_URDF, read_json, write_json, quintic, cruise, fit_gravity, gravity_observability, fit_link6_mass_constrained

ZERO = np.zeros(6)
KP = [4., 10., 10., 2., 2., 1.]
KD = [.5, .8, .8, .2, .2, .1]
CALIBRATION_KP = [24.48,61.2,61.2,12.24,12.24,12.24]
CALIBRATION_KD = [1.2,1.92,1.92,.48,.48,.48]


def checked(result):
    if result.get('status', -1):
        raise RuntimeError(str(result))
    return result


def fresh(state):
    received=state.get('host_received_monotonic_s')
    host_age=0. if received is None else time.perf_counter()-received
    return (all(state.get(k, 0) == 63 for k in ('online_mask', 'valid_q_mask', 'valid_dq_mask', 'valid_tau_mask'))
            and np.isfinite(host_age) and host_age>=0
            and max(state.get('feedback_age_us', [2**32]))+host_age*1e6 < 20000
            and state.get('motor_fault_valid_mask',63)==63
            and not any(state.get('fault_code_raw',[]))
            and all(np.all(np.isfinite(state.get(k, [float('nan')]))) for k in ('q_rad','dq_rad_s','tau_measured_nm')))


class NullLog:
    """Discard output without allocating a buffer or creating any files."""
    def write(self, value): return len(value)
    def flush(self): pass
    def close(self): pass
    def __enter__(self): return self
    def __exit__(self, *args): pass


class Session:
    def __init__(self, args, model):
        self.args, self.model = args, model
        self.robot = None
        self.logging = not getattr(args, 'no_log', False)
        if not self.logging and args.resume:
            raise ValueError('Cannot resume with logging disabled')
        self.directory = None
        self.summary_path = None
        if self.logging:
            self.directory = Path(args.output or Path(__file__).parent/'examples'/'logs'/
                                  (args.feature+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
            metadata=dict(arguments=vars(args), model_identity=model.identity,gravity_theta=model.theta.tolist())
            if args.resume:
                old=read_json(self.directory/'metadata.json')
                ignored={'resume','execute','output','local','board'}
                comparable=lambda a:{k:v for k,v in dict({'plan_preset':'grid243','link6_mass':None,'link6_mass_tolerance':0.},**a).items() if k not in ignored}
                if old['model_identity']!=model.identity or old.get('gravity_theta')!=metadata['gravity_theta'] or comparable(old['arguments'])!=comparable(metadata['arguments']):
                    raise ValueError('Resume requires unchanged experiment parameters and model')
            else:
                self.directory.mkdir(parents=True, exist_ok=False)
                write_json(self.directory/'metadata.json', metadata)
            run_id=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.summary_path=self.directory/('summary_'+run_id+'.json')
            # Every attempt has its own raw file; incomplete attempts remain available.
            raw_path=self.directory/('raw_'+run_id+'.csv' if args.resume else 'raw.csv')
            self.file = raw_path.open('w', newline='', encoding='utf-8')
            self.writer = csv.writer(self.file)
            self.writer.writerow(['t','point','phase','state_sequence','mcu_us','command_sequence','applied_sequence','fresh']+
                                 [f'{name}{i+1}' for name in ('q','dq','tau','qref','dqref','kp','kd','ff','age_us','mode','fault') for i in range(6)])
        else:
            self.file = NullLog()
            self.writer = csv.writer(self.file)
        self.start = self.deadline = time.perf_counter()
        self.sent = self.records = 0
        self.seen = None
        self.last_print = 0
        self.active = False
        self.encoder_zero_rad = np.zeros(6)
        self.encoder_previous_raw = None
        self.last_send_time=None;self.period_sum=0.;self.period_max=0.;self.period_count=0
        self.summary = {'status': 'running'}

    def __enter__(self):
        try:
            self.robot = Robot(local=self.args.local, board=self.args.board,
                               board_node=getattr(self.args, "board_node", 11),
                               ports=tuple(getattr(self.args, "ports", (5005, 5006)))).start()
            if not self.robot.discover(8):
                raise RuntimeError('No board telemetry; check address and UDP port ownership')
            support=self.robot.operation(19)
            if support.get('status')!=0:
                raise RuntimeError('当前NXP固件尚不支持柔顺停止与断联保护；请先更新配套固件，未启动机械臂')
            if hasattr(self.args, 'no_hand'):
                result = self.robot.operation(16, target_mask=int(not self.args.no_hand))
                legacy_hand = not self.args.no_hand and result.get('status') == -2
                if result.get('status', -1) and not legacy_hand:
                    raise RuntimeError('Firmware hand-CAN selection failed; flash the updated firmware first: '+str(result))
                if legacy_hand:
                    print('Legacy firmware: hand CAN uses its original enabled behavior.', flush=True)
                self.summary['hand_can_enabled'] = not self.args.no_hand
            checked(self.robot.start_state_stream(motor_query_hz=500, hand_query_hz=50, hand_angles_only=True))
            deadline = time.perf_counter()+5
            while not fresh(self.robot.snapshot('ArmState').get('ArmState', {})):
                if time.perf_counter() > deadline:
                    snapshot = self.robot.snapshot()
                    diag = snapshot.get('Diagnostics', {})
                    arm = snapshot.get('ArmState', {})
                    detail = {key: diag.get(key) for key in ('can_tx_completed', 'can_tx_failed', 'can_rx_valid', 'can_bus_state', 'can_tx_error_count', 'hand_state_query_mode')}
                    detail['motor_feedback_count'] = diag.get('motor_feedback_count')
                    detail['feedback_age_us'] = arm.get('feedback_age_us')
                    print('ARM FEEDBACK DIAGNOSTICS: '+json.dumps(detail), flush=True)
                    raise RuntimeError('Six-axis position/velocity/torque feedback not available')
                time.sleep(.01)
            checked(self.robot.operation(1))
            checked(self.robot.operation(13, target_mask=0))  # Disable example2 zero-MIT override.
            self.start = self.deadline = time.perf_counter()
            return self
        except BaseException:
            self.__exit__(*__import__('sys').exc_info())
            raise

    def configure_encoder_turns(self, joints):
        """Freeze explicitly selected axes' equivalent encoder zero before motion."""
        if self.active:raise RuntimeError('Encoder turn selection must precede activation')
        raw=self.robot.snapshot('ArmState').get('ArmState', {})
        if not fresh(raw):raise RuntimeError('Fresh six-axis feedback required for encoder turn selection')
        q=np.asarray(raw['q_rad'],dtype=float)
        offset=np.zeros(6)
        for joint in joints:
            if joint not in range(1,7):raise ValueError('Encoder joint index must be 1..6')
            offset[joint-1]=2*np.pi*np.rint(q[joint-1]/(2*np.pi))
        logical=q-offset
        from home_path import start_ok
        if not start_ok(logical):
            raise RuntimeError('Six finite joint positions required for homing')
        self.encoder_zero_rad=offset
        self.encoder_previous_raw=q.copy()
        self.summary['encoder_reference']=dict(joints=joints,raw_start_deg=np.rad2deg(q).tolist(),
            raw_zero_deg=np.rad2deg(offset).tolist(),logical_start_deg=np.rad2deg(logical).tolist())
        return self.summary['encoder_reference']

    def state(self):
        state = self.robot.snapshot('ArmState').get('ArmState', {})
        if self.active and state.get('control_owner') == 2:
            self.active=False;self.robot.lease=0
            raise RuntimeError('NXP已接管（本地回零／柔顺保持）；本次遥操结束，请检查原因后再启动')
        if not fresh(state):
            raise RuntimeError('Cannot calculate from missing/stale/nonfinite joint feedback')
        raw=np.asarray(state['q_rad'],dtype=float)
        if self.encoder_previous_raw is not None:
            if np.any(np.abs(raw-self.encoder_previous_raw)>np.pi):
                raise RuntimeError('Encoder turn jumped during operation; stop and recheck zero reference')
            self.encoder_previous_raw=raw.copy()
        state['raw_q_rad']=raw.tolist()
        state['q_rad']=(raw-self.encoder_zero_rad).tolist()
        return state

    def step(self, point, phase, qref, dqref, kp, kd, extra=None, state=None, *, pace=True):
        state = state or self.state()
        q = np.asarray(state['q_rad'])
        ff = self.args.gravity_scale*self.model(q) + (ZERO if extra is None else extra)
        sent = self.robot.send_arm(np.asarray(qref)+self.encoder_zero_rad, dqref, kp, kd, ff)
        if not sent.ok:
            raise RuntimeError(sent.error)
        self.sent += 1
        self.last_command_sequence = sent.sequence
        if not self.active:
            # Realtime target and management activation use distinct UDP sockets.
            # Local HOME_HOLD may receive activation before the matching target.
            until=time.perf_counter()+.1
            while True:
                result=self.robot.operation(3)
                if result.get('status')!=-16 or state.get('control_owner')!=2 or time.perf_counter()>=until:
                    checked(result);break
                time.sleep(.002)
                retry=self.robot.send_arm(np.asarray(qref)+self.encoder_zero_rad,dqref,kp,kd,ff)
                if not retry.ok:raise RuntimeError(retry.error)
                self.last_command_sequence=retry.sequence
            self.active = True
        now = time.perf_counter()
        if self.last_send_time is not None:
            period=now-self.last_send_time
            self.period_sum+=period;self.period_max=max(self.period_max,period);self.period_count+=1
        self.last_send_time=now
        seq = state['header']['sequence']
        if self.logging and seq != self.seen:
            self.writer.writerow([now-self.start,point,phase,seq,state['header']['tx_monotonic_us'],sent.sequence,
                                  state['last_applied_command_sequence'],int(fresh(state))]+
                                 state['q_rad']+state['dq_rad_s']+state['tau_measured_nm']+
                                 list(qref)+list(dqref)+list(kp)+list(kd)+list(ff)+state['feedback_age_us']+
                                 state.get('motor_mode_raw',[0]*6)+state.get('fault_code_raw',[0]*6))
            self.seen = seq; self.records += 1
        if now-self.last_print > 1:
            self.file.flush()
            print(f'{point} {phase}: {self.sent/max(now-self.start,.001):.1f} Hz sent; q_deg={np.round(np.rad2deg(q),1).tolist()}', flush=True)
            self.last_print = now
        if pace:
            self.deadline += 1/self.args.hz
            delay = self.deadline-time.perf_counter()
            if delay > 0: time.sleep(delay)
            else: self.deadline = time.perf_counter()
        return state

    def stop_homing_fault(self):
        """Request persistent board gravity/damping hold, never an exit trajectory."""
        self.active=False
        try:
            result=self.robot.operation(18)
            self.summary['soft_stop_request']=result
            checked(result)
        except Exception as exc:
            self.summary['soft_stop_error']=repr(exc)
            # An ACK can be lost after the MCU entered soft hold. Do not turn
            # an uncertain reply into drive-off; the board watchdog is active.
            raise RuntimeError('板端柔顺保持未确认，请检查板端状态并支撑机械臂：'+str(exc)) from exc
        finally:
            self.robot.lease=0

    def rebase_home_zero(self, shift):
        """Same physical pose; preserve selected turns for subsequent GELLO targets."""
        shift=np.asarray(shift,dtype=float)
        if shift.shape!=(6,) or not np.all(np.isfinite(shift)) or not np.allclose(shift/(2*np.pi),np.rint(shift/(2*np.pi)),atol=1e-6,rtol=0):
            raise ValueError('Homing reference must contain whole encoder turns')
        self.encoder_zero_rad=self.encoder_zero_rad+shift
        self.summary['homing_raw_zero_deg']=np.rad2deg(self.encoder_zero_rad).tolist()

    def move(self, target, point):
        origin = np.asarray(self.state()['q_rad'])
        start = time.perf_counter()
        while True:
            t = time.perf_counter()-start
            q, v = quintic(t, self.args.transition, origin, target)
            self.step(point, 'move', q, v, self.args.kp, self.args.kd)
            if t >= self.args.transition: break

    def hold(self, target, seconds, point, phase):
        end = time.perf_counter()+seconds
        rows = []; seen = None
        while time.perf_counter() < end:
            state = self.step(point, phase, target, ZERO, self.args.kp, self.args.kd)
            seq = state['header']['sequence']
            if seq != seen:
                rows.append(state); seen = seq
        return rows

    def __exit__(self, typ, exc, tb):
        errors = []
        if self.robot:
            self.summary['final'] = self.robot.snapshot()
            self.summary['transport_stats']=dict(getattr(self.robot,'stats',{}))
            self.summary['stream_gaps']=dict(getattr(self.robot,'gaps',{}))
            # The MCU continues gravity/damping support after this process exits.
            if self.robot.lease:
                if self.active and (self.args.exit_mode == 'hold' or typ is not None):
                    try:
                        self.stop_homing_fault()
                    except Exception as error: errors.append(repr(error))
                for op in ((4, 2) if self.robot.lease else ()):
                    try: checked(self.robot.operation(op))
                    except Exception as error: errors.append(repr(error))
            # Operation 11 sets hand_isolation=true on the deployed firmware,
            # suppressing arm polling. Board-owned soft hold still needs fresh
            # motor feedback after host exit; stopping queries faults that hold.
            # Close host sockets only and leave NXP motor polling running.
            self.summary['motor_feedback_queries_preserved'] = True
            self.robot.close()
        self.file.close()
        elapsed = time.perf_counter()-self.start
        self.summary.update(status='complete' if typ is None else 'interrupted' if typ is KeyboardInterrupt else 'failed',
                            error=None if exc is None else repr(exc), cleanup_errors=errors,
                            seconds=elapsed, commands=self.sent, unique_state_records=self.records,
                            measured_command_hz=self.sent/max(elapsed,.001), exit_mode=self.args.exit_mode)
        self.summary.update(command_period_mean_s=self.period_sum/max(self.period_count,1),command_period_max_s=self.period_max)
        if self.logging:
            write_json(self.directory/'summary.json', self.summary)
            write_json(self.summary_path,self.summary)
            print('Saved:', self.directory, flush=True)


def gravity_run(args, model):
    parameters = read_json(args.friction) if args.friction else None
    transferred=bool(parameters and parameters.get('format')=='panthera-friction-transfer-v1')
    identity_key='target_model_identity' if transferred else 'model_identity'
    theta_key='target_gravity_theta' if transferred else 'gravity_theta'
    if parameters and parameters[identity_key] != model.identity:
        raise ValueError('Friction model identity mismatch')
    if parameters and parameters.get(theta_key) != model.theta.tolist():
        raise ValueError('Friction fit used a different gravity parameter set')
    if args.feature=='gravity_friction' and not parameters:
        raise ValueError('Example 8 requires --friction PATH from example 9')
    if parameters:
        if len(parameters['joints'])!=6:raise ValueError('Friction model needs six joint entries')
        for fit in parameters['joints']:
            if fit is not None and any(not np.isfinite(fit[k]) or fit[k]<0 for k in ('fc_nm','b_nm_s_rad')):
                raise ValueError('Friction coefficients must be finite and nonnegative')
    history = deque()
    with Session(args, model) as s:
        if parameters:
            s.summary['friction_parameters']=parameters
            s.summary['friction_scale']=args.friction_scale
            if transferred:print('FRICTION: historical Follower coefficients transferred explicitly; not reidentified on current assembly.',flush=True)
        qref = np.asarray(s.state()['q_rad'])
        if args.feature == 'impedance' and args.target_deg:
            qref = np.deg2rad(args.target_deg); s.move(qref, 'target')
        begin = time.perf_counter()
        while args.seconds == 0 or time.perf_counter()-begin < args.seconds:
            state = s.state(); now = time.perf_counter()
            history.append((now, np.asarray(state['q_rad'])))
            while len(history)>2 and now-history[0][0] > .2: history.popleft()
            extra = ZERO.copy()
            if parameters and len(history)>2:
                v = (history[-1][1]-history[0][1])/(history[-1][0]-history[0][0])
                for i, fit in enumerate(parameters['joints']):
                    if fit is not None:
                        extra[i] = args.friction_scale*(fit['fc_nm']*np.tanh(v[i]/args.friction_smooth)+fit['b_nm_s_rad']*v[i])
                        if abs(v[i]) < args.friction_deadband: extra[i] = 0.
            impedance = args.feature == 'impedance'
            s.step('run', args.feature, qref if impedance else ZERO, ZERO,
                   args.kp if impedance else ZERO, args.kd if impedance else ZERO, extra, state)


def plan_points(args, current):
    if args.plan:
        doc = read_json(args.plan)
        return np.deg2rad(np.asarray(doc['poses_deg'], float))
    if args.plan_preset=='legacy285':
        return np.deg2rad(np.asarray(legacy285(),float))
    # 3^5 poses centred on current posture; J1 rotation adds no gravity excitation
    # for a vertical base, but user-supplied plans can vary all six coordinates.
    return np.array([current+np.deg2rad([0,*p]) for p in itertools.product((0,args.span_deg,-args.span_deg), repeat=5)])


def gravity_capture(args, model):
    with Session(args, model) as s:
        points = (np.deg2rad(read_json(s.directory/'plan.json')['poses_deg']) if args.resume
                  else plan_points(args, np.asarray(s.state()['q_rad'])))
        if points.ndim != 2 or points.shape[1] != 6 or not np.all(np.isfinite(points)):
            raise ValueError('poses_deg must be an N x 6 finite array')
        observation=gravity_observability(points,model,args.fit_scope)
        write_json(s.directory/'plan.json', {'poses_deg':np.rad2deg(points).tolist(),'observability':observation,
                   'preset':'custom' if args.plan else args.plan_preset})
        print('GRAVITY_FIT',args.fit_scope,'planned rank',observation['rank'],'/',observation['parameters'],
              'condition',observation['normalized_condition'],flush=True)
        completed = read_json(s.directory/'points.json') if args.resume and (s.directory/'points.json').exists() else []
        finished={r['point'] for r in completed}
        for index, target in enumerate(points):
            if index in finished: continue
            s.move(target, index); s.hold(target, args.settle, index, 'settle')
            rows = s.hold(target, args.sample, index, 'sample')
            q = np.array([r['q_rad'] for r in rows]); tau = np.array([r['tau_measured_nm'] for r in rows])
            if len(rows) < 10: raise RuntimeError('Too few distinct feedback packets')
            good = bool(np.max(np.std(q,axis=0)) <= np.deg2rad(args.stability_deg))
            s.hold(target,args.post_hold,index,'post_hold')
            completed.append(dict(point=index, accepted=good, samples=len(rows),q=q.mean(axis=0).tolist(),
                                  tau=tau.mean(axis=0).tolist(),q_std=q.std(axis=0).tolist(),tau_std=tau.std(axis=0).tolist()))
            write_json(s.directory/'points.json', completed)
            print('POINT', index, 'accepted' if good else 'unstable; retained for review', flush=True)
        accepted = [r for r in completed if r['accepted']]
        if len(accepted)>=10:
            qfit=[r['q'] for r in accepted];taufit=[r['tau'] for r in accepted]
            result = (fit_link6_mass_constrained(qfit,taufit,model,args.link6_mass,args.link6_mass_tolerance)
                      if args.link6_mass is not None else fit_gravity(qfit,taufit,model,args.fit_scope))
            write_json(s.directory/'gravity_identified.json', result)
            s.summary['gravity_fit']=result
            print('GRAVITY_RESULT',json.dumps(dict(usable=result['usable'],link6=result['link6'],
                holdout_rmse_nm=result['holdout_rmse_nm'],quality_issues=result['quality_issues'])),flush=True)


def friction_capture(args, model):
    with Session(args, model) as s:
        center = np.asarray(s.state()['q_rad']) if args.center_deg is None else np.deg2rad(args.center_deg)
        plan = []
        for j in args.joints:
            for speed in args.speeds:
                for direction in (1,-1):
                    for repeat in range(args.repeat):
                        start, end = center.copy(), center.copy()
                        start[j-1] -= direction*np.deg2rad(args.span_deg)
                        end[j-1] += direction*np.deg2rad(args.span_deg)
                        plan.append(dict(joint=j, speed_deg_s=speed, direction=direction,repeat=repeat,
                                         start_rad=start.tolist(),end_rad=end.tolist()))
        if args.resume: plan=read_json(s.directory/'plan.json')
        else: write_json(s.directory/'plan.json', plan)
        completed=read_json(s.directory/'completed.json') if args.resume and (s.directory/'completed.json').exists() else []
        for index, point in enumerate(plan):
            if index in completed: continue
            j = point['joint']-1; start = np.array(point['start_rad']); end=np.array(point['end_rad'])
            s.move(start,index); s.hold(start,args.settle,index,'settle'); begin=time.perf_counter()
            while True:
                q,v,phase,total=cruise(time.perf_counter()-begin,start[j],end[j],np.deg2rad(point['speed_deg_s']),args.ramp)
                target=start.copy(); target[j]=q; velocity=ZERO.copy(); velocity[j]=v
                s.step(index,phase,target,velocity,args.kp,args.kd)
                if phase=='done': break
            s.hold(end,.3,index,'post_hold')
            s.file.flush()
            completed.append(index);write_json(s.directory/'completed.json',completed)
        s.summary['passes']=len(plan)


def static_capture(args, model):
    """Ramp extra effort in each direction; remove PD on tested axis only.

    Breakaway is confirmed by sustained displacement, not noisy single velocity.
    A ramp reaching its experimental endpoint is censored, never called friction.
    """
    with Session(args,model) as s:
        center=np.asarray(s.state()['q_rad']) if args.center_deg is None else np.deg2rad(args.center_deg)
        if args.resume:center=np.array(read_json(s.directory/'static_plan.json')['center_rad'])
        else:write_json(s.directory/'static_plan.json',dict(center_rad=center.tolist()))
        results=read_json(s.directory/'static_results.json') if args.resume and (s.directory/'static_results.json').exists() else []
        completed={(r['joint'],r['direction'],r['repeat']) for r in results}
        for j in args.joints:
            for direction in (1,-1):
                for repeat in range(args.repeat):
                    if (j,direction,repeat) in completed:continue
                    point=f'J{j}_{direction}_{repeat}'
                    s.move(center,point); s.hold(center,args.settle,point,'settle')
                    origin=np.asarray(s.state()['q_rad']); kp=np.array(args.kp); kd=np.array(args.kd)
                    kp[j-1]=kd[j-1]=0
                    begin=time.perf_counter(); detected=None; candidate=None
                    while True:
                        t=time.perf_counter()-begin
                        effort=min(t*args.ramp_nm_s,args.max_extra_nm)
                        extra=ZERO.copy(); extra[j-1]=direction*effort
                        state=s.step(point,'breakaway',center,ZERO,kp,kd,extra)
                        displacement=state['q_rad'][j-1]-origin[j-1]
                        if abs(displacement)>=np.deg2rad(args.breakaway_deg):
                            if candidate is None:
                                candidate=dict(effort_nm=direction*effort,displacement_rad=displacement,
                                               tau_nm=state['tau_measured_nm'][j-1],q=state['q_rad'],time=t)
                            if t-candidate['time']>=args.confirm_seconds:
                                detected=candidate; break
                        else: candidate=None
                        if effort>=args.max_extra_nm: break
                    results.append(dict(joint=j,direction=direction,repeat=repeat,breakaway=detected,
                                        status='censored' if detected is None else 'opposite_drift' if direction*detected['displacement_rad']<0 else 'detected'))
                    write_json(s.directory/'static_results.json',results)
                    # Reacquire at actual pose immediately; subsequent move returns to centre.
                    s.hold(np.asarray(s.state()['q_rad']),.3,point,'recapture')
        s.summary['static_interpretation']='Extra torque at onset includes residual gravity error; use paired directions and repeated poses. Static friction is not fed back automatically.'


def parser(feature):
    p=argparse.ArgumentParser(description=f'Panthera UDP: {feature}; without --execute only print offline preview')
    p.set_defaults(feature=feature)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--local',default='192.168.10.1'); p.add_argument('--board',default='192.168.10.11')
    p.add_argument('--board-node',type=int,default=11)
    p.add_argument('--ports',type=int,nargs=2,default=[5005,5006])
    p.add_argument('--urdf',default=str(DEFAULT_URDF)); p.add_argument('--mapping'); p.add_argument('--identified')
    p.add_argument('--payload-mass',type=float,default=0); p.add_argument('--payload-com',type=float,nargs=3,default=[0,0,0])
    p.add_argument('--gravity',type=float,nargs=3,default=[0,0,-9.81]); p.add_argument('--gravity-scale',type=float,default=1)
    p.add_argument('--hz',type=float,default=500)
    p.add_argument('--seconds',type=float,default=0 if feature in ('gravity','impedance') else 15,
                   help='Run duration; 0 runs until Ctrl+C. Gravity and impedance default to 0; other modes default to 15.')
    p.add_argument('--kp',type=float,nargs=6,default=CALIBRATION_KP if feature=='gravity_capture' else KP)
    p.add_argument('--kd',type=float,nargs=6,default=CALIBRATION_KD if feature=='gravity_capture' else KD)
    p.add_argument('--transition',type=float,default=1 if feature=='gravity_capture' else 5); p.add_argument('--output')
    p.add_argument('--resume',action='store_true',help='Continue a capture using --output existing_dataset and identical settings')
    p.add_argument('--exit-mode',choices=['hold','last'],default='hold',help='hold sends current-pose PD before release; last leaves last motor command')
    p.add_argument('--target-deg',type=float,nargs=6)
    p.add_argument('--friction'); p.add_argument('--friction-scale',type=float,default=.5)
    p.add_argument('--friction-smooth',type=float,default=.1); p.add_argument('--friction-deadband',type=float,default=.03)
    p.add_argument('--plan'); p.add_argument('--span-deg',type=float,default=15)
    p.add_argument('--plan-preset',choices=['legacy285','grid243'],
                   default='legacy285' if feature=='gravity_capture' else 'grid243',
                   help='Gravity capture defaults to historical absolute 285 poses; --plan overrides this')
    p.add_argument('--fit-scope',choices=['link6','all'],default='link6',help='Gravity identification: keep links1-5 fixed by default')
    p.add_argument('--link6-mass',type=float,help='Known TOTAL replacement hand mass in kg; fit COM using J2-J6 only')
    p.add_argument('--link6-mass-tolerance',type=float,default=0.,help='Allowed +/- kg around --link6-mass; zero fixes mass exactly')
    p.add_argument('--settle',type=float,default=.5 if feature=='gravity_capture' else 1.5)
    p.add_argument('--sample',type=float,default=.5 if feature=='gravity_capture' else 1)
    p.add_argument('--post-hold',type=float,default=.5,help='Gravity capture: hold after sampling before marking point complete')
    p.add_argument('--stability-deg',type=float,default=.2)
    p.add_argument('--center-deg',type=float,nargs=6); p.add_argument('--joints',type=int,nargs='+',default=[1,2,3,4,5,6])
    p.add_argument('--speeds',type=float,nargs='+',default=[5,10,20,40]); p.add_argument('--repeat',type=int,default=1)
    p.add_argument('--ramp',type=float,default=.2)
    p.add_argument('--ramp-nm-s',type=float,default=.1); p.add_argument('--max-extra-nm',type=float,default=1)
    p.add_argument('--breakaway-deg',type=float,default=.3); p.add_argument('--confirm-seconds',type=float,default=.05)
    return p


def main(feature, defaults=None):
    p=parser(feature)
    if defaults:p.set_defaults(**defaults)
    args=p.parse_args()
    if args.link6_mass is not None:
        if feature!='gravity_capture' or args.fit_scope!='link6':p.error('--link6-mass requires gravity capture with link6 scope')
        if not np.isfinite(args.link6_mass+args.link6_mass_tolerance) or args.link6_mass_tolerance<0 or args.link6_mass<=args.link6_mass_tolerance:p.error('Invalid link6 mass interval')
    elif args.link6_mass_tolerance!=0:p.error('--link6-mass-tolerance requires --link6-mass')
    if args.resume and (not args.output or feature not in ('gravity_capture','friction_capture','static_capture')):
        p.error('--resume needs a capture feature and --output existing_dataset')
    for key in ('hz','transition','settle','sample','post_hold','span_deg','ramp','ramp_nm_s','max_extra_nm','breakaway_deg','confirm_seconds','friction_smooth','stability_deg'):
        if not np.isfinite(getattr(args,key)) or getattr(args,key)<=0: p.error(key+' must be finite and positive')
    if args.seconds<0 or not np.isfinite(args.seconds) or args.repeat<1 or any(j not in range(1,7) for j in args.joints): p.error('Invalid duration/repeat/joints')
    if any(not np.isfinite(v) or v<=0 for v in args.speeds): p.error('speeds must be positive')
    for key in ('kp','kd','gravity','payload_com','target_deg','center_deg'):
        value=getattr(args,key)
        if value is not None and not np.all(np.isfinite(value)): p.error(key+' must be finite')
    if any(v<0 for v in args.kp+args.kd): p.error('PD gains must be nonnegative')
    for key in ('gravity_scale','friction_scale','friction_deadband'):
        if not np.isfinite(getattr(args,key)) or getattr(args,key)<0:p.error(key+' must be finite and nonnegative')
    model=Gravity(args.urdf,read_json(args.mapping) if args.mapping else None,args.identified,
                  args.payload_mass,args.payload_com,args.gravity)
    if feature=='friction_capture' and 2*args.span_deg<max(args.speeds)*args.ramp: p.error('Travel too short for speed/ramp')
    if not args.execute:
        print(json.dumps(dict(feature=feature,model_identity=model.identity,gravity_at_encoder_zero_nm=model(ZERO).tolist(),
                              arguments=vars(args),status='offline preview; add --execute to run'),indent=2))
        return 0
    try:
        {'gravity':gravity_run,'impedance':gravity_run,'gravity_capture':gravity_capture,
         'friction_capture':friction_capture,'static_capture':static_capture,'gravity_friction':gravity_run}[feature](args,model)
    except KeyboardInterrupt: return 130
    return 0

"""Shared or independent GELLO inputs drive two NXP arms on one scheduler."""
import argparse
from copy import deepcopy
from contextlib import ExitStack, contextmanager
import csv
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import traceback

ROOT=Path(__file__).resolve().parent
HOST=ROOT/'source/host';EXAMPLES=HOST/'examples'
sys.path[:0]=[str(EXAMPLES),str(HOST)]
import arm_runtime
import numpy as np
from arm_dynamics import Gravity,read_json
from arm_lab import Session,ZERO,checked,parser as arm_parser
from dual_peer import PeerMonitor
from local_home import wait_for_local_home
from gello_inputs import GelloInputs
from hand_slider_demo import hand_feedback
from soft_limits import Limits, SoftLimitGovernor
from teleop_tuning import HandTuning, HandTargetFilter, resolve_tuning
spec=importlib.util.spec_from_file_location('single_gello_teleop',EXAMPLES/'18_gello-low-impedance.py')
teleop=importlib.util.module_from_spec(spec);spec.loader.exec_module(teleop)


def session_args(config,side,output):
    b=config['boards'][side]
    if not b['identified']:raise ValueError(f'{side}: terminal load/model not confirmed')
    args=arm_parser('impedance').parse_args([])
    args.execute=True;args.local=config['local'];args.board=b['ip'];args.board_node=b['node'];args.ports=b['ports']
    args.kp=b['kp'];args.kd=b['kd'];args.hz=config['hz'];args.identified=str(ROOT/b['identified'])
    args.output=str(output/side);args.no_hand=not b['hand'];args.no_log=False
    return args


def checked_hand_feedback(session,tuning):
    raw=session.robot.snapshot('HandState').get('HandState',{})
    h=hand_feedback(raw)
    received=raw.get('host_received_monotonic_s',float('nan'))
    age=(time.perf_counter()-received)*1000
    if (h['q_rad'] is None or not np.isfinite(age) or age<0
            or h['angle_age_ms']+age>=tuning.feedback_timeout_ms):
        raise RuntimeError(f'Hand feedback missing/stale (board={h["angle_age_ms"]:.1f}ms, host={age:.1f}ms)')
    h['angle_age_ms']+=age
    return h


def prepare_hand(session,tuning=None):
    tuning=HandTuning() if tuning is None else tuning
    r=session.robot
    try:
        # Keep six-axis feedback and MCU HOME_HOLD running during mode readback.
        checked(r.start_state_stream(motor_query_hz=500,hand_query_hz=50,hand_angles_only=True))
        checked(r.prepare_hand_mode(3))
        checked_hand_feedback(session,tuning)
    finally:
        checked(r.start_state_stream(motor_query_hz=500,hand_query_hz=50,hand_angles_only=True))
        deadline=time.perf_counter()+5
        while True:
            try:session.state();break
            except RuntimeError:
                if time.perf_counter()>=deadline:raise
                time.sleep(.005)
        session.deadline=time.perf_counter()
    checked(r.configure(arm_state_period_us=2000,hand_state_period_us=20000,hand_mode=3,
                        hand_soft_current_ma=[tuning.current_ma]*6,hand_control_feedforward_ma=tuning.current_ma))


def prepare_startup_hands(sessions,config,tunings,*,timeout=2):
    """Prepare available hands; optionally omit unavailable hands for this run."""
    require_hands=config.get('require_hands',True)
    def skip_hand(side,reason):
        s=sessions[side]
        # Cancel pending hand requests and disable hand CAN, never send blind
        # finger targets or hot-add a recovered hand partway through a run.
        checked(s.robot.operation(16,target_mask=0))
        s.state()  # A hand fallback must not hide invalid arm feedback.
        config['boards'][side]['hand']=False
        s.args.no_hand=True
        s.summary.update(hand_can_enabled=False,hand_skipped_reason=reason)
        print(f'{side}: 手未就绪，本次跳过手控制，机械臂继续；{reason}',flush=True)
    pending={side for side in sessions if config['boards'][side]['hand']}
    deadline=time.perf_counter()+timeout
    failures={}
    while pending:
        for side in list(pending):
            try:
                checked_hand_feedback(sessions[side],tunings[side])
                pending.remove(side)
            except RuntimeError as exc:
                failures[side]=str(exc)
        if not pending:break
        if time.perf_counter()>=deadline:
            if not require_hands:
                for side in sorted(pending):skip_hand(side,failures[side])
                break
            detail='；'.join(f'{side}: {failures[side]}' for side in sorted(pending))
            raise RuntimeError('灵巧手启动检查失败，尚未接管机械臂：'+detail+
                               '。请检查对应手的供电和CAN通信，恢复反馈后重试')
        time.sleep(.01)
    for side,s in sessions.items():
        if not config['boards'][side]['hand']:continue
        try:prepare_hand(s,tunings[side])
        except RuntimeError as exc:
            if not require_hands:
                skip_hand(side,str(exc))
                continue
            raise RuntimeError(f'{side}: 灵巧手模式3准备失败，尚未接管机械臂：{exc}') from exc
        print(f'{side}: HAND MODE 3 READY',flush=True)


@contextmanager
def stop_pair_on_exit(sessions):
    """Request board-owned soft hold before cleanup, even on Ctrl-C/send error.

    No exit-time trajectory is sent. The second board is still attempted if
    one RPC fails; each MCU also detects a stale host command independently.
    """
    try:
        yield
    except BaseException as exc:
        for s in sessions.values():
            s.summary['failure_before_cleanup']={'error':repr(exc),'snapshot':s.robot.snapshot()}
        raise
    finally:
        # A claimed lease is not an arm takeover. Failed hand preparation must
        # release that lease normally, leaving MCU HOME_HOLD intact.
        started=any(s.active or s.sent for s in sessions.values())
        for side,s in (sessions.items() if started else ()):
            s.active=False
            if not s.robot.lease:
                continue
            try:
                s.stop_homing_fault()
            except Exception as exc:
                s.summary['paired_stop_error']=repr(exc)
                print(f'{side}: 柔顺保持未确认，需现场支撑：{exc}',flush=True)


def home_sessions(sessions,config):
    """Home concurrently; keep commanding each completed arm until both finish."""
    cancel=threading.Event();done={side:threading.Event() for side in sessions};errors={}
    class HomeProxy:
        def __init__(self,session):self.session=session
        def __getattr__(self,name):return getattr(self.session,name)
        def step(self,*args,**kwargs):
            if cancel.is_set():raise RuntimeError('对侧回零中断，停止本侧轨迹')
            return self.session.step(*args,**kwargs)
        def stop_homing_fault(self):
            cancel.set()  # Cancel the peer trajectory before any blocking RPC.
            return self.session.stop_homing_fault()
    def worker(side,s):
        try:
            proxy=HomeProxy(s);s.deadline=time.perf_counter()
            state=s.state()
            if state.get('control_owner')==2:
                kp=np.array(teleop.CALIBRATION_KP);kd=np.array(teleop.CALIBRATION_KD)
                kp[3]*=3;kd[3]*=np.sqrt(3)
                proxy.step('startup','local_home_takeover',state['q_rad'],ZERO,kp,kd,state=state)
            print(f'{side}: HOME ZERO；following Kp={s.args.kp}; Kd={s.args.kd}',flush=True)
            gains=(s.args.kp,s.args.kd) if config['boards'][side]['homing_gains']=='follow' else None
            teleop.home_zero(proxy,gains=gains)
            done[side].set()
            while not all(event.is_set() for event in done.values()):
                state=s.state()
                if not teleop.home_path.Tracking().update(state['q_rad'],ZERO,time.perf_counter()):
                    raise RuntimeError('等待对侧回零时保持误差超过5°，请求柔顺保持')
                proxy.step('home','wait_peer_hold',ZERO,ZERO,s.args.kp,s.args.kd,state=state)
        except BaseException as exc:
            errors[side]=exc;cancel.set();done[side].set()
    threads=[threading.Thread(target=worker,args=(side,s),name='home-'+side) for side,s in sessions.items()]
    try:
        for thread in threads:thread.start()
        while any(thread.is_alive() for thread in threads):
            for thread in threads:thread.join(.01)
    finally:
        cancel.set()
        for thread in threads:
            if thread.ident is not None:thread.join()
    if errors:
        detail='；'.join(f'{side}: {error}' for side,error in errors.items())
        raise RuntimeError('双臂回零中断：'+detail) from next(iter(errors.values()))


def send_pair(sessions,states,targets,cycle,following,*,governors=None,now=None):
    """Filter and validate BOTH arms before the first target is sent."""
    commands={side:(targets[side],ZERO,{}) for side in sessions}
    if governors is not None:
        now=time.perf_counter() if now is None else now
        commands={}
        for side in sessions:
            try:commands[side]=governors[side].update(targets[side],states[side],now)
            except RuntimeError as exc:
                sessions[side].summary['feedback_at_fault']=states[side]
                raise type(exc)(f'{side}: {exc}') from exc
    for side,s in sessions.items():
        q,v,_=commands[side]
        s.step(str(cycle),'gello_follow' if (following[side] if isinstance(following,dict) else following) else 'gello_lost_hold',
               q,v,s.args.kp,s.args.kd,state=states[side],pace=False)
    return commands


def targets_for(desired,states,holds):
    return {side:holds[side].target(desired[side] if isinstance(desired,dict) else desired,state['q_rad'])[0] for side,state in states.items()}


def send_hand_pair(sessions,feedback,targets,filters,now):
    commands={side:filters[side].update(targets[side],h['q_rad'],now) for side,h in feedback.items()}
    for side,target in commands.items():
        sent=sessions[side].robot.send_hand(target,20)
        if not sent.ok:raise RuntimeError(f'{side}: {sent.error}')
    return commands


def update_hand_targets(feedback,grasps,tunings,targets,following):
    """Latch each hand's actual pose on its own trigger loss at the hand tick."""
    for side,h in feedback.items():
        grasp=grasps[side]
        if grasp is not None:targets[side]=tunings[side].target(grasp).tolist()
        elif following.get(side,False) or side not in targets:targets[side]=list(h['q_rad'])
        following[side]=grasp is not None


def run(config,output,seconds):
    # Validate tuning before any socket, serial port or hardware operation.
    soft_config=read_json(ROOT/config['soft_limits_file'])
    soft_by_side,hand_tuning=resolve_tuning(config,soft_config)
    governors={side:SoftLimitGovernor(Limits(**soft_by_side[side])) for side in config['boards']}
    hand_filters={side:HandTargetFilter(hand_tuning[side]) for side in config['boards']}
    for side,governor in governors.items():
        limits=governor.limits
        print(f'{side}: SOFT LIMITS deg lower={limits.lower_deg.tolist()} upper={limits.upper_deg.tolist()} reference_filter={limits.reference_filter}',flush=True)
    inputs=GelloInputs(config,ROOT,teleop)
    wait_for_local_home(config)
    sessions={};stop=threading.Event();observer=None
    output.mkdir(parents=True,exist_ok=True)
    (output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    (output/'soft-limits.json').write_text(json.dumps(soft_config,indent=2)+'\n')
    (output/'effective-tuning.json').write_text(json.dumps({side:dict(soft_limits=soft_by_side[side],
        hand_control=hand_tuning[side].document()) for side in config['boards']},indent=2)+'\n')
    def observe():
        monitor=PeerMonitor(config['local'])
        try:
            with (output/'peer.jsonl').open('w',buffering=1) as log:
                while not stop.is_set():
                    sample={'time':datetime.now().isoformat(),'boards':{}}
                    for b in config['boards'].values():
                        try:sample['boards'][str(b['node'])]=monitor.query(b['node'])
                        except Exception as exc:sample.setdefault('errors',[]).append(f"{b['node']}: {exc}")
                    log.write(json.dumps(sample)+'\n');stop.wait(.2)
        finally:monitor.close()
    try:
        inputs.wait_ready()
        with ExitStack() as stack, stop_pair_on_exit(sessions):
            for side in config['boards']:
                a=session_args(config,side,output)
                model=Gravity(a.urdf,None,a.identified,a.payload_mass,a.payload_com,a.gravity)
                sessions[side]=stack.enter_context(Session(a,model))
                sessions[side].summary.update(shared_gello=inputs.shared,gello_source=inputs.specs['shared' if inputs.shared else side],side=side,kp=a.kp,kd=a.kd)
            references={}
            for side,s in sessions.items():
                references[side]=s.configure_encoder_turns(config['boards'][side]['encoder_modulo_joints'])
                print(f'{side}: ENCODER ZERO '+json.dumps(references[side]),flush=True)
            (output/'encoder_reference.json').write_text(json.dumps(references,indent=2)+'\n')
            prepare_startup_hands(sessions,config,hand_tuning)
            (output/'active-hands.json').write_text(json.dumps({side:config['boards'][side]['hand']
                for side in sessions},indent=2)+'\n')
            observer=threading.Thread(target=observe,daemon=True);observer.start()
            home_sessions(sessions,config)
            holds={side:teleop.HoldOnLoss() for side in sessions};hand_targets={};hand_following={}
            previous_follow={side:None for side in sessions};cycle=0;missed=0;last_print=0
            start=deadline=next_hand=time.perf_counter()
            last_print=start  # Accumulate a real interval before reporting rate.
            for s in sessions.values():s.deadline=start
            print('等待 GELLO 回到各自保存的固定零位；'+('共享输入' if inputs.shared else '左右独立输入')+'，各侧目标独立限位。',flush=True)
            with (output/('shared_targets.csv' if inputs.shared else 'dual_targets.csv')).open('w',buffering=1) as log, (output/'hands.csv').open('w',buffering=1) as handlog, (output/'soft_limits.csv').open('w',buffering=1) as softlog:
                writer=csv.writer(log)
                if inputs.shared:
                    writer.writerow(['cycle','t','following','grasp_percent']+[f'qref{i}' for i in range(1,7)]+[f'{side}_command_sequence' for side in sessions])
                else:
                    writer.writerow(['cycle','t','side','following','grasp_percent']+[f'qref{i}' for i in range(1,7)]+['command_sequence'])
                hw=csv.writer(handlog);hw.writerow(['t','side','mode','age_ms','ack_status']+[f'target{i}' for i in range(6)]+[f'actual{i}' for i in range(6)])
                sw=csv.writer(softlog);sw.writerow(['cycle','t','side','target_clipped','filtered','min_barrier']+
                    [f'{name}{i}' for name in ('requested_q','sent_q','sent_dq') for i in range(1,7)])
                while not seconds or time.perf_counter()-start<seconds:
                    now=time.perf_counter()
                    # Validate BOTH arms before sending a new target to either board.
                    states={side:s.state() for side,s in sessions.items()}
                    due_hand=now>=next_hand;feedback={}
                    if due_hand:
                        for side,s in sessions.items():
                            if config['boards'][side]['hand']:
                                try:feedback[side]=checked_hand_feedback(s,hand_tuning[side])
                                except RuntimeError as exc:raise RuntimeError(f'{side}: {exc}') from exc
                    samples=inputs.poll(now)
                    desired={side:sample[0] for side,sample in samples.items()}
                    grasps={side:sample[1] for side,sample in samples.items()}
                    following={side:target is not None for side,target in desired.items()}
                    targets=targets_for(desired,states,holds)
                    commands=send_pair(sessions,states,targets,cycle,following,governors=governors)
                    for side,(q,v,diag) in commands.items():
                        sw.writerow([cycle,now-start,side,diag['target_clipped'],diag['filtered'],diag['min_barrier']]+
                                    list(targets[side])+q.tolist()+v.tolist())
                    if due_hand:
                        update_hand_targets(feedback,grasps,hand_tuning,hand_targets,hand_following)
                        hand_commands=send_hand_pair(sessions,feedback,hand_targets,hand_filters,now)
                        for side,target in hand_commands.items():
                            h=feedback[side]
                            diag=sessions[side].robot.snapshot('Diagnostics').get('Diagnostics',{})
                            hw.writerow([now-start,side,h['mode'],h['angle_age_ms'],diag.get('hand_control_ack_latest_status')]+target+h['q_rad'])
                        next_hand+=.02
                        if next_hand<=now:next_hand=now+.02
                    if inputs.shared:
                        side=next(iter(sessions))
                        writer.writerow([cycle,now-start,following[side],grasps[side]]+
                            (desired[side].tolist() if following[side] else ['']*6)+[s.last_command_sequence for s in sessions.values()])
                    for side,s in sessions.items():
                        if not inputs.shared:
                            writer.writerow([cycle,now-start,side,following[side],grasps[side]]+
                                (desired[side].tolist() if following[side] else ['']*6)+[s.last_command_sequence])
                        if following[side]!=previous_follow[side]:
                            print(f'{side}: '+('FOLLOW GELLO' if following[side] else 'GELLO未就绪，保持本侧当前位置'),flush=True)
                    previous_follow=following;cycle+=1
                    if now-last_print>=5:
                        print(f'DUAL: {cycle/max(now-start,.001):.1f} Hz; missed={missed}; '+str({side:np.round(np.rad2deg(states[side]["q_rad"]),1).tolist() for side in sessions}),flush=True);last_print=now
                    deadline+=1/config['hz'];delay=deadline-time.perf_counter()
                    if delay>0:time.sleep(delay)
                    else:
                        skip=int(-delay*config['hz'])+1;missed+=skip;deadline+=skip/config['hz']
                for s in sessions.values():s.summary['shared_scheduler']={'cycles':cycle,'missed_periods':missed,'seconds':time.perf_counter()-start}
    finally:
        inputs.stop();stop.set()
        if observer and observer.is_alive():observer.join(2)


class Tee:
    def __init__(self,terminal,log):self.terminal,self.log=terminal,log
    def write(self,s):self.terminal.write(s);self.log.write(s);return len(s)
    def flush(self):self.terminal.flush();self.log.flush()


def main(argv=None, config_override=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');p.add_argument('--dry-run',action='store_true',help='Print configuration without hardware access');p.add_argument('--seconds',type=float,default=0)
    p.add_argument('--gello-source',choices=['usb','nxp-uart','nxp-usb'],default='usb',help='PC USB adapter, NXP J5 UART5, or NXP J11 USB host')
    p.add_argument('--side',choices=['both','left','right'],default='both',help='Control both arms, new arm (left), or original arm (right)')
    a=p.parse_args(argv)
    config=read_json(ROOT/'one-gello-two-arms.json') if config_override is None else deepcopy(config_override)
    if 'gellos' not in config:config['gello']['transport']=a.gello_source
    if a.side!='both':
        config['boards']={a.side:config['boards'][a.side]}
    if a.dry_run or not a.execute:
        soft,hands=resolve_tuning(config,read_json(ROOT/config['soft_limits_file']))
        config['effective_tuning']={side:dict(soft_limits=soft[side],hand_control=hands[side].document()) for side in config['boards']}
        print(json.dumps(config,indent=2));return
    if a.seconds<0 or not np.isfinite(a.seconds):p.error('seconds must be finite and nonnegative')
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f');output=ROOT/'logs'/(('two_gello_two_arms_' if 'gellos' in config else 'one_gello_two_arms_')+stamp)
    console=output.with_suffix('.console.log');record_path=ROOT/'evidence/current-bimanual-session.json'
    output.parent.mkdir(parents=True,exist_ok=True);record_path.parent.mkdir(parents=True,exist_ok=True)
    record=dict(pid=os.getpid(),started=datetime.now().isoformat(),status='process_running',side=a.side,output=str(output),console=str(console))
    record_path.write_text(json.dumps(record,indent=2)+'\n');out,err=sys.stdout,sys.stderr;code=0
    with console.open('w',buffering=1) as log:
        sys.stdout,sys.stderr=Tee(out,log),Tee(err,log)
        try:run(config,output,a.seconds)
        except KeyboardInterrupt:code=130
        except BaseException:code=1;traceback.print_exc()
        finally:
            record.update(status='exited',exit_code=code,ended=datetime.now().isoformat());record_path.write_text(json.dumps(record,indent=2)+'\n')
            sys.stdout.flush();sys.stderr.flush();sys.stdout,sys.stderr=out,err
    raise SystemExit(code)

if __name__=='__main__':main()

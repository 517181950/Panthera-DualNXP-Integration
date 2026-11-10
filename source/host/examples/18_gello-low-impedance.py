"""GELLO IDs1-7 or IDs11-17 following; optional seventh-axis hand grasp.

Uses saved encoder zeros when configured, otherwise the original startup zero.
Joint directions and scales come from the selected original MuJoCo group mapping.
Stale GELLO feedback latches actual arm position, not the old moving target.
"""
from pathlib import Path
import sys,time,csv,json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import arm_runtime
import home_path
import numpy as np
from arm_dynamics import Gravity,read_json,quintic
from arm_lab import Session,NullLog,ZERO,checked,CALIBRATION_KP,CALIBRATION_KD,parser as arm_parser
from hand_slider_demo import gripper_targets_deg,hand_feedback
from gello_arm_compare import Comparison,gello_values
from gello_compare_driver.reader import GelloReader,SERVO_SIGNS,SERVO_GEARS

HERE=Path(__file__).resolve().parent


class HoldOnLoss:
    def __init__(self):self.hold=None;self.following=False
    def target(self, desired, actual):
        valid=desired is not None and np.all(np.isfinite(desired))
        if valid:
            self.following=True;self.hold=None
            return np.array(desired),True
        if self.hold is None:self.hold=np.array(actual).copy()
        self.following=False
        return self.hold.copy(),False


class StartAlignment:
    """Align the first fresh master sample with the measured follower pose."""
    def __init__(self):self.offset=None
    def target(self,desired,actual):
        if desired is None or not np.all(np.isfinite(desired)):return desired
        if self.offset is None:self.offset=np.asarray(actual)-np.asarray(desired)
        return np.asarray(desired)+self.offset


class FixedGelloZero:
    """Map saved encoder zeros independently of the reader's startup pose."""
    def __init__(self,group_config):
        self.zero=np.asarray(group_config['zero_raw_ticks'],dtype=float)
        if self.zero.shape!=(6,) or not np.all(np.isfinite(self.zero)) or np.any((self.zero<0)|(self.zero>=4096)):
            raise ValueError('Saved GELLO zero must contain six encoder ticks in [0,4096)')
        self.factor=np.array(SERVO_SIGNS[:6])*np.array(SERVO_GEARS[:6])*np.array(
            [axis['sign']*axis['scale'] for axis in group_config['axes']])
        self.offset=np.array([axis['offset_rad'] for axis in group_config['axes']])
        self.previous_delta=None
    def target(self,ticks):
        ticks=np.asarray(ticks,dtype=float)
        if ticks.shape!=(6,) or not np.all(np.isfinite(ticks)):return None
        delta=ticks-self.zero
        reference=np.zeros(6) if self.previous_delta is None else self.previous_delta
        # Pick the initial encoder turn nearest zero, then preserve continuity
        # even if the serial reader restarts its unwrapped counter.
        delta+=4096*np.rint((reference-delta)/4096)
        self.previous_delta=delta.copy()
        return delta*(2*np.pi/4096)*self.factor+self.offset


def home_zero(session, gains=None):
    try:
        return _home_zero(session,gains)
    except BaseException:
        try:session.stop_homing_fault()
        except Exception as error:
            session.summary['homing_stop_error']=repr(error)
            print('回零故障停机请求未确认，请使用现场急停：'+str(error),flush=True)
        raise

def _home_zero(session, gains=None):
    """Raise upright, align yaw upright, then lower into encoder zero."""
    origin=np.asarray(session.state()['q_rad']).copy()
    if not home_path.start_ok(origin):
        session.summary['homing']=dict(status='rejected',reason='invalid_position_feedback')
        raise RuntimeError('回零需要六轴有效位置反馈')
    if gains is None:
        kp,kd=np.array(CALIBRATION_KP),np.array(CALIBRATION_KD)
        # Mounted-hand trials required this J4 homing gain on the original arm.
        kp[3]*=3.;kd[3]*=np.sqrt(3.)
    else:
        kp,kd=(np.asarray(v,dtype=float).copy() for v in gains)
    follow_kp,follow_kd=np.asarray(session.args.kp),np.asarray(session.args.kd)
    stages=home_path.stages(origin)
    durations=[home_path.timing(begin,end)[3] for _,begin,end in stages]
    final_zero=stages[-1][2]
    session.summary['homing']=dict(origin_deg=np.rad2deg(origin).tolist(),target_deg=np.rad2deg(final_zero).tolist(),
        transition_s=sum(durations),stage_durations_s=durations,order=['raise_upright','align_yaw_upright','lower_to_zero'],
        kp=kp.tolist(),kd=kd.tolist(),tolerance_deg=2.,status='running')
    print(f'HOME ZERO: 先协调抬到竖直，竖直对正J1，再降回零位；各段轨迹 {durations} 秒。',flush=True)
    tracking=home_path.Tracking()
    def checked_step(phase,target,velocity,kp,kd):
        state=session.state()
        if not tracking.update(state['q_rad'],target,time.perf_counter()):
            error_deg=np.rad2deg(np.asarray(state['q_rad'])-target)
            session.summary['homing'].update(status='failed',reason='tracking_error',actual_deg=np.rad2deg(state['q_rad']).tolist(),
                target_deg=np.rad2deg(target).tolist(),error_deg=error_deg.tolist(),feedback_at_fault=state)
            joint=int(np.argmax(np.abs(error_deg)))+1
            raise RuntimeError(f'回零未到位（{phase}，J{joint}误差{error_deg[joint-1]:.2f}°），停止轨迹并请求柔顺保持')
        return session.step('home',phase,target,velocity,kp,kd,state=state)
    start=time.perf_counter()
    while True:
        t=time.perf_counter()-start;blend=min(t,1.)
        checked_step('home_gain_ramp',origin,ZERO,
            follow_kp+(kp-follow_kp)*blend,follow_kd+(kd-follow_kd)*blend)
        if t>=1.:break
    for (stage,begin,target),duration in zip(stages,durations):
        session.summary['homing']['stage']=stage
        start=time.perf_counter()
        while True:
            t=time.perf_counter()-start;q,v=home_path.trajectory(t,begin,target)
            checked_step('home_'+stage,q,v,kp,kd)
            if t>=duration:break
        start=time.perf_counter();still=home_path.Still()
        while True:
            state=checked_step('home_'+stage+'_settle',target,ZERO,kp,kd)
            now=time.perf_counter()
            position_ok=np.max(np.abs(np.asarray(state['q_rad'])-target))<=np.deg2rad(2.)
            if still.update(state['q_rad'],state['dq_rad_s'],now,position_ok):break
            if now-start>5.:
                session.summary['homing'].update(status='failed',actual_deg=np.rad2deg(state['q_rad']).tolist())
                raise RuntimeError(f'回零未到位（{stage}，要求误差 <=2° 且稳定），未进入下一阶段或GELLO跟随')
    session.rebase_home_zero(final_zero)
    state=session.state()
    session.summary['homing'].update(status='complete',actual_deg=np.rad2deg(state['q_rad']).tolist())
    print('HOME ZERO COMPLETE: q_deg='+str(np.round(np.rad2deg(state['q_rad']),2).tolist()),flush=True)
    # Return to the requested follow gains while continuing to command zero.
    start=time.perf_counter()
    while True:
        t=time.perf_counter()-start;blend=min(t,1.)
        checked_step('home_follow_gains',ZERO,ZERO,
            kp+(follow_kp-kp)*blend,kd+(follow_kd-kd)*blend)
        if t>=1.:break


def grasp_percent(group,now,inverted=False,timeout=.15):
    if group is None or group.servo_captured_at[6] is None or now-group.servo_captured_at[6]>timeout:return None
    fraction=group.gripper_normalized
    if fraction is None or not np.isfinite(fraction):return None
    opening=1-fraction if inverted else fraction
    return 100*(1-float(np.clip(opening,0,1)))


def main():
    p=arm_parser('impedance')
    p.set_defaults(identified=str(HERE/'profiles/gravity_selected.json'),seconds=0,
        kp=[4,10,10,2,2,1],kd=[.5,.8,.8,.2,.2,.1])
    p.add_argument('--group',type=int,choices=[0,1],default=1,help='0: IDs1-7; 1: IDs11-17')
    startup=p.add_mutually_exclusive_group()
    startup.add_argument('--align-current-pose',action='store_true',help='Use current arm pose as the starting reference for GELLO motion')
    startup.add_argument('--home-zero',action='store_true',help='Return all six joints to encoder zero before starting GELLO following')
    p.add_argument('--port',default='COM5');p.add_argument('--gello-config',default=str(HERE/'profiles/gello_selected.json'))
    p.add_argument('--no-hand',action='store_true',help='Disable ALL hand CAN TX on updated firmware; persists until explicitly re-enabled or board reset');p.add_argument('--loss-ms',type=float,default=150)
    p.add_argument('--require-hand',action='store_true',help='Require confirmed hand mode and fresh angles before following')
    p.add_argument('--hand-current-feedback',action='store_true',help='Also request 50 Hz hand current feedback and firmware auxiliary queries')
    p.add_argument('--no-log',action='store_true',default=True,help='No files or log directory (default)')
    p.add_argument('--log',dest='no_log',action='store_false',help='Enable CSV and summary logging')
    a=p.parse_args()
    if a.require_hand and a.no_hand:p.error('--require-hand cannot be combined with --no-hand')
    if not np.isfinite(a.loss_ms) or a.loss_ms<=0:p.error('Invalid loss timeout')
    if a.hz!=500:p.error('Arm target rate is 500 Hz')
    if a.resume:p.error('Use a new log directory')
    if not np.isfinite(a.seconds) or a.seconds<0 or not np.all(np.isfinite(a.kp+a.kd)) or min(a.kp+a.kd)<0:p.error('Invalid gains or duration')
    model=Gravity(a.urdf,read_json(a.mapping) if a.mapping else None,a.identified,a.payload_mass,a.payload_com,a.gravity)
    config=read_json(a.gello_config);mapping=Comparison(config);group_index=a.group;group_name=('left','right')[group_index];ids=list(range(1+10*group_index,8+10*group_index));grip=config[group_name]['gripper']
    fixed_zero=FixedGelloZero(config[group_name]) if 'zero_raw_ticks' in config[group_name] else None
    if a.execute and not a.no_hand and config.get('_single_arm_calibration',{}).get('gripper',{}).get('validated') is False:
        p.error('GELLO trigger calibration is being corrected; capture and validate endpoints before hand control')
    if not a.execute:
        print(json.dumps(dict(status='offline preview; --execute starts following',port=a.port,ids=ids,
            kp=a.kp,kd=a.kd,gravity_model=a.identified,hand=not a.no_hand,logging=not a.no_log,
            mapping=config[group_name],align_current_pose=a.align_current_pose,home_zero=a.home_zero,
            saved_zero=fixed_zero is not None,
            hand_current_feedback=a.hand_current_feedback,loss='latch actual arm position after stale GELLO; resume on fresh data'),indent=2));return 0
    reader=GelloReader(a.port,baudrate=config['serial']['baud'],group_mask=1<<group_index,timeout_s=.03,
        read_velocity=True,velocity_rpm_per_unit=config['serial'].get('velocity_rpm_per_unit',.732))
    reader.set_gripper_calibration(group_index,grip['min_raw'],grip['max_raw'])
    hold=HoldOnLoss();alignment=StartAlignment();hand_target=None;hand_following=False;hand_enabled=False
    try:
        with Session(a,model) as s:
            if not a.no_hand:
                try:
                    # Confirm hand mode before any arm motion, with arm CAN polling
                    # paused. The hand mode transaction timed out under six-axis load
                    # in HIL, while a hand-only transaction completed and read back 3.
                    checked(s.robot.start_state_stream(motor_query_hz=0,hand_query_hz=50,hand_angles_only=True,hand_current_fast=a.hand_current_feedback))
                    checked(s.robot.prepare_hand_mode(3))
                    feedback=hand_feedback(s.robot.snapshot().get('HandState',{}))
                    if feedback['q_rad'] is None or feedback['angle_age_ms']>=100:
                        raise RuntimeError('No fresh hand angles after mode confirmation')
                    hand_enabled=True
                    print(f'HAND READY: mode 3, GELLO ID{ids[-1]}, five-finger grasp, 50 Hz commands',flush=True)
                except Exception as exc:
                    s.summary['hand_initialization_error']=str(exc)
                    if a.require_hand:raise
                    print('HAND DISABLED: '+str(exc)+'; continuing ARM following only.',flush=True)
                finally:
                    checked(s.robot.start_state_stream(motor_query_hz=500,hand_query_hz=50,hand_angles_only=True,hand_current_fast=a.hand_current_feedback))
                    # Reacquire six-axis feedback after the quiet mode transaction.
                    deadline=time.perf_counter()+5
                    while True:
                        try:s.state();break
                        except RuntimeError:
                            if time.perf_counter()>=deadline:raise
                            time.sleep(.005)
                    s.deadline=time.perf_counter()
                if hand_enabled:
                    checked(s.robot.configure(arm_state_period_us=2000,hand_state_period_us=20000,hand_mode=3,
                        hand_soft_current_ma=[500]*6,hand_control_feedforward_ma=500))
            if a.home_zero:home_zero(s)
            # Start only after homing: master motion during homing cannot become
            # a queued target, and the arm's previous pose is never a map offset.
            reader.start()
            s.summary['hand_enabled']=hand_enabled
            s.summary['hand_current_feedback']=a.hand_current_feedback
            s.summary.update(gello_config=config,gello_port=a.port,gello_ids=ids,loss_ms=a.loss_ms)
            zero_ready=fixed_zero is None
            if fixed_zero is not None:
                print('GELLO FIXED ZERO: '+str(fixed_zero.zero.tolist())+'; waiting for master within 3 degrees of saved zero',flush=True)
            with (NullLog() if a.no_log else (s.directory/'gello_targets.csv').open('w',newline='',encoding='utf-8')) as f, \
                 (NullLog() if a.no_log else (s.directory/'hand.csv').open('w',newline='',encoding='utf-8')) as hf:
                writer=csv.writer(f);writer.writerow(['t','following','grasp_percent',f'raw_id{ids[-1]}',f'unwrapped_id{ids[-1]}','gripper_normalized']+[f'qref{i}' for i in range(1,7)])
                hand_writer=csv.writer(hf)
                hand_writer.writerow(['t','command_sequence','mode','angle_age_ms','current_age_ms','can_tx','can_ack','ack_status']+
                    [f'{kind}{i}' for kind in ('target_rad','actual_rad','current_ma') for i in range(1,7)])
                start=time.perf_counter();next_hand=start;last_status=start;last_hand_print=start;last_follow=None
                while a.seconds==0 or time.perf_counter()-start<a.seconds:
                    now=time.perf_counter();state=s.state();group=reader.latest(group_index)
                    vals=gello_values(group,now,stale_s=a.loss_ms/1000)
                    desired=fixed_zero.target(vals['ticks']) if fixed_zero is not None else np.deg2rad(mapping.calculate(group_index,vals,None)['g_mujoco'])
                    if not reader.is_connected:desired=None
                    if not zero_ready:
                        if desired is not None and np.all(np.isfinite(desired)) and np.max(np.abs(desired))<=np.deg2rad(3):
                            zero_ready=True;s.summary['master_zero_start_error_deg']=np.rad2deg(desired).tolist()
                            print('GELLO SAVED ZERO READY',flush=True)
                        else:desired=None
                    if a.align_current_pose:
                        desired=alignment.target(desired,state['q_rad'])
                        if alignment.offset is not None and 'alignment_offset_rad' not in s.summary:
                            s.summary['alignment_offset_rad']=alignment.offset.tolist()
                    target,following=hold.target(desired,state['q_rad'])
                    s.step('run','gello_follow' if following else 'gello_lost_hold',target,ZERO,a.kp,a.kd,state=state)
                    now=time.perf_counter()
                    percent=grasp_percent(group,now,grip['inverted'],a.loss_ms/1000) if following else None
                    if hand_enabled and now>=next_hand:
                        feedback=hand_feedback(s.robot.snapshot().get('HandState',{}))
                        if a.require_hand and (feedback['q_rad'] is None or feedback['angle_age_ms']>=100):
                            s.summary['hand_feedback_error']=dict(angle_age_ms=feedback['angle_age_ms'],mode=feedback['mode'])
                            raise RuntimeError(f"Hand feedback lost ({feedback['angle_age_ms']:.1f} ms); ending arm/hand following")
                        if percent is not None:
                            hand_target=np.deg2rad(gripper_targets_deg(percent,'five_finger')).tolist();hand_following=True
                        else:
                            feedback=hand_feedback(s.robot.snapshot().get('HandState',{}))
                            if (hand_following or hand_target is None) and feedback['q_rad'] is not None and feedback['angle_age_ms']<100:
                                hand_target=feedback['q_rad']
                            hand_following=False
                        if hand_target is not None:
                            result=s.robot.send_hand(hand_target,20)
                            if not result.ok:
                                hand_enabled=False;s.summary['hand_enabled']=False
                                s.summary['hand_send_error']=result.error
                                if a.require_hand:raise RuntimeError(result.error)
                                print('HAND DISABLED: '+str(result.error)+'; continuing ARM following only.',flush=True)
                            snapshot=s.robot.snapshot();feedback=hand_feedback(snapshot.get('HandState',{}));diag=snapshot.get('Diagnostics',{})
                            hand_writer.writerow([now-start,result.sequence,feedback['mode'],feedback['angle_age_ms'],feedback['current_age_ms'],
                                diag.get('hand_control_tx'),diag.get('hand_control_ack'),diag.get('hand_control_ack_latest_status')]+
                                hand_target+(feedback['q_rad'] or [None]*6)+
                                ((feedback['current_ma'] or [None]*6) if feedback['current_age_ms']<100 else [None]*6))
                        next_hand+=.02
                        if next_hand<=now:next_hand=now+.02
                    if following!=last_follow or now-last_status>=.2:
                        writer.writerow([now-start,following,percent,None if group is None else group.raw_positions[6],
                            None if group is None else group.unwrapped_ticks[6],None if group is None else group.gripper_normalized]+target.tolist())
                        f.flush();hf.flush()
                        last_status=now
                    if hand_enabled and percent is not None and now-last_hand_print>=1.:
                        print(f'HAND grasp={percent:.1f}% raw={group.raw_positions[6]} continuous={group.unwrapped_ticks[6]}',flush=True)
                        last_hand_print=now
                    if following!=last_follow:
                        print('FOLLOW GELLO' if following else 'GELLO not at saved zero: HOLD CURRENT ARM ANGLES' if not zero_ready else 'GELLO unavailable: HOLD CURRENT ARM ANGLES',flush=True)
                        f.flush();last_follow=following
                s.summary['gello_stats']=str(reader.stats)
    except KeyboardInterrupt:return 130
    finally:reader.stop()
    return 0


if __name__=='__main__':raise SystemExit(main())

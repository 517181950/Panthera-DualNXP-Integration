"""Dear PyGui hand teleoperation with continuous arm gravity/friction control.

The GUI owns no sockets. One worker owns commands, telemetry and log files.
Without --execute the GUI is an offline preview and never constructs Robot.
"""
from collections import deque
import csv
import json
from pathlib import Path
import threading
import time
import arm_runtime
import numpy as np
from arm_dynamics import Gravity, read_json
from arm_lab import Session, ZERO, checked, parser as arm_parser
from robot_rates import StateRates

EXAMPLES=Path(__file__).parent/'examples'
HAND_INDICES=(0,1,2,4,6,8)
HAND_NAMES=('Thumb rotation','Thumb bend','Index bend','Middle bend','Ring bend','Little bend')
# Slider travel includes the user's two-finger gripper endpoints.
HAND_MAX_RAD=(np.deg2rad(75),.524,np.pi/2,np.pi/2,np.pi/2,np.pi/2)


def gripper_targets_deg(percent, profile='two_finger'):
    """Continuous piecewise-linear closing path; reverse uses the same path."""
    p=float(percent)
    if not np.isfinite(p) or not 0<=p<=100:
        raise ValueError('Gripper percentage must be between 0 and 100')
    if profile=='five_finger':
        bends=np.array([13.,53.,56.,55.,52.])
        if p<=20:return [100*p/20,0.,0.,0.,0.,0.]
        if p<=95:return [100.,*(bends*((p-20)/75)).tolist()]
        return [100.,*(bends+5*((p-95)/5)).tolist()]
    if profile!='two_finger':raise ValueError('Unknown gripper profile')
    if p<=20:
        rotation=75*p/20;thumb=0.;index=0.
    elif p<=95:
        u=(p-20)/75
        rotation=75.;thumb=18*u;index=52*u
    else:
        u=(p-95)/5
        rotation=75.;thumb=18+2*u;index=52+3*u
    return [rotation,thumb,index,90.,90.,90.]


class Shared:
    def __init__(self, gripper_profile='two_finger'):
        gripper_targets_deg(0,gripper_profile)
        self.gripper_profile=gripper_profile
        self.hand_max_rad=list(HAND_MAX_RAD)
        if gripper_profile=='five_finger':self.hand_max_rad[0]=np.deg2rad(100)
        self.lock=threading.Lock()
        self.target=None
        self.view=dict(ready=False,status='Connecting...',error=None,individual=True,gripper_percent=0.)

    def initialize(self, q):
        with self.lock:
            self.target=list(q)
            self.view.update(ready=True,initial_deg=np.rad2deg(q).tolist(),status='Running')

    def set_joint(self, index, degrees):
        value=float(np.deg2rad(degrees))
        if not np.isfinite(value) or value<0 or value>65.535:
            raise ValueError('Hand target is outside the unsigned milliradian wire representation')
        with self.lock:
            if self.target is not None and self.view['individual']:self.target[index]=value

    def set_individual(self, enabled):
        with self.lock:
            if self.target is None:return
            self.view['individual']=bool(enabled)
            if not enabled:
                self.target=np.deg2rad(gripper_targets_deg(self.view['gripper_percent'],self.gripper_profile)).tolist()

    def set_gripper(self, percent):
        target=np.deg2rad(gripper_targets_deg(percent,self.gripper_profile)).tolist()
        with self.lock:
            self.view['gripper_percent']=float(percent)
            if self.target is not None and not self.view['individual']:self.target=target

    def targets(self):
        with self.lock:return None if self.target is None else self.target.copy()

    def update(self, **values):
        with self.lock:self.view.update(values)

    def snapshot(self):
        with self.lock:return dict(self.view)


def hand_feedback(state):
    q=state.get('joint_q_rad',[]);current=state.get('motor_current_ma',[])
    ages=state.get('field_age_us',[2**32]*5);valid=state.get('valid_fields',0)
    qok=bool(state.get('online') and valid&2 and len(q)==11 and np.all(np.isfinite(q)))
    iok=bool(state.get('online') and valid&8 and len(current)==6 and np.all(np.isfinite(current)))
    return dict(q_rad=[q[i] for i in HAND_INDICES] if qok else None,
                current_ma=list(current) if iok else None,
                angle_age_ms=ages[1]/1000,current_age_ms=ages[3]/1000,
                mode=state.get('mode_raw'))


class Friction:
    def __init__(self, args, model):
        self.doc=read_json(args.friction)
        transferred=self.doc.get('format')=='panthera-friction-transfer-v1'
        identity='target_model_identity' if transferred else 'model_identity'
        theta='target_gravity_theta' if transferred else 'gravity_theta'
        if self.doc[identity]!=model.identity or self.doc[theta]!=model.theta.tolist():
            raise ValueError('Friction profile does not match selected gravity model')
        self.joints=self.doc['joints'];self.args=args;self.history=deque()
        if len(self.joints)!=6:raise ValueError('Expected six friction entries')
        for row in self.joints:
            if row is not None and any(not np.isfinite(row[k]) or row[k]<0 for k in ('fc_nm','b_nm_s_rad')):
                raise ValueError('Invalid friction coefficient')

    def torque(self, now, q):
        self.history.append((now,np.array(q)))
        while len(self.history)>2 and now-self.history[0][0]>.2:self.history.popleft()
        extra=np.zeros(6)
        if len(self.history)>2 and now>self.history[0][0]:
            v=(self.history[-1][1]-self.history[0][1])/(now-self.history[0][0])
            for i,row in enumerate(self.joints):
                if row is not None and abs(v[i])>=self.args.friction_deadband:
                    extra[i]=self.args.friction_scale*(row['fc_nm']*np.tanh(v[i]/self.args.friction_smooth)+row['b_nm_s_rad']*v[i])
        return extra


def run_control(args, shared, stop):
    try:
        model=Gravity(args.urdf,read_json(args.mapping) if args.mapping else None,args.identified,
                      args.payload_mass,args.payload_com,args.gravity)
        friction=Friction(args,model)
        with Session(args,model) as session:
            robot=session.robot
            # Angle-fast + current-fast divides the 20 ms period into two
            # requests: angle at 50 Hz and current at 50 Hz, not 25 Hz each.
            checked(robot.start_state_stream(motor_query_hz=500,hand_query_hz=50,
                hand_angles_only=True,hand_current_fast=True))
            checked(robot.prepare_hand_mode(args.hand_mode))
            checked(robot.configure(arm_state_period_us=2000,hand_state_period_us=20000,
                hand_mode=args.hand_mode,hand_soft_current_ma=[args.hand_current_ma]*6,
                hand_control_feedforward_ma=args.hand_current_ma))
            session.summary.update(friction_parameters=friction.doc,friction_scale=args.friction_scale,
                hand_mode=args.hand_mode,requested_rates_hz=dict(arm_tx=500,arm_rx=500,hand_tx=50,hand_rx=50,hand_angles=50,hand_current=50))
            deadline=time.perf_counter()+5
            while not stop.is_set():
                feedback=hand_feedback(robot.snapshot().get('HandState',{}))
                if feedback['q_rad'] is not None and feedback['angle_age_ms']<100:
                    shared.initialize(feedback['q_rad']);break
                if time.perf_counter()>deadline:raise RuntimeError('No fresh hand angles for slider initialization')
                stop.wait(.01)
            if stop.is_set():return
            with (session.directory/'hand.csv').open('w',newline='',encoding='utf-8') as f:
                writer=csv.writer(f)
                writer.writerow(['t','command_sequence','state_sequence','mode','angle_age_ms','current_age_ms']+
                    [f'{kind}{i}' for kind in ('target_rad','actual_rad','current_ma') for i in range(1,7)])
                begin=time.perf_counter();next_hand=begin;next_view=begin;hand_sent=0
                rate_time=begin;previous_sent=0;arm_sent=0;hand_hz=0.;arm_hz=0.;rates=StateRates()
                while not stop.is_set() and (args.seconds==0 or time.perf_counter()-begin<args.seconds):
                    now=time.perf_counter();state=session.state()
                    extra=friction.torque(now,state['q_rad'])
                    session.step('run','hand_gui',ZERO,ZERO,ZERO,ZERO,extra,state)
                    now=time.perf_counter()
                    if now>=next_hand:
                        target=shared.targets()
                        result=robot.send_hand(target,args.hand_duration_ms)
                        if not result.ok:raise RuntimeError(result.error)
                        hand_sent+=1;next_hand+=.02
                        if next_hand<=now:next_hand=now+.02
                        hand=robot.snapshot().get('HandState',{});feedback=hand_feedback(hand)
                        writer.writerow([now-begin,result.sequence,hand.get('header',{}).get('sequence'),
                            feedback['mode'],feedback['angle_age_ms'],feedback['current_age_ms']]+
                            target+(feedback['q_rad'] or ['']*6)+(feedback['current_ma'] or ['']*6))
                    if now>=next_view:
                        snap=robot.snapshot();feedback=hand_feedback(snap.get('HandState',{}))
                        with robot._lock:counts=dict(robot.count)
                        measured=rates.update(now,counts,session.sent,snap)
                        if now-rate_time>=1:
                            hand_hz=(hand_sent-previous_sent)/(now-rate_time)
                            arm_hz=(session.sent-arm_sent)/(now-rate_time)
                            rate_time=now;previous_sent=hand_sent;arm_sent=session.sent;f.flush()
                        shared.update(status='Running',feedback=feedback,arm_q_deg=np.rad2deg(state['q_rad']).tolist(),
                            arm_tx_hz=arm_hz,hand_tx_hz=hand_hz,rates=measured,log=str(session.directory.resolve()))
                        next_view=now+.02
                session.summary.update(hand_commands=hand_sent,hand_command_hz=hand_sent/max(time.perf_counter()-begin,.001))
    except Exception as exc:
        shared.update(error=str(exc),status='Stopped: '+str(exc))
        print('DEMO ERROR:',repr(exc),flush=True)
    finally:
        shared.update(ready=False)
        if not shared.snapshot().get('error'):shared.update(status='Stopped')


def refresh_controls(dpg, shared):
    view=shared.snapshot();individual=view['individual'];target=shared.targets()
    dpg.set_value('individual_control',individual)
    dpg.configure_item('individual_control',enabled=view['ready'])
    dpg.configure_item('gripper_controls',show=not individual)
    dpg.configure_item('gripper_percent',enabled=view['ready'])
    dpg.set_value('gripper_percent',view['gripper_percent'])
    for i in range(6):
        dpg.configure_item(f'slider{i}',show=individual,enabled=view['ready'])
        dpg.configure_item(f'target{i}',show=not individual)
        if target is not None:
            degrees=float(np.rad2deg(target[i]))
            dpg.set_value(f'slider{i}',degrees)
            dpg.set_value(f'target{i}',f'{degrees:.2f} deg')


def build_gui(dpg, args, shared):
    dpg.create_context();dpg.configure_app(manual_callback_management=True)
    whole=shared.gripper_profile=='five_finger'
    dpg.create_viewport(title='Panthera | '+('Five-finger grasp' if whole else 'Two-finger gripper'),width=980,height=720,min_width=850,min_height=650)
    with dpg.window(tag='main',label='Panthera'):
        dpg.add_text('PANTHERA  /  HAND CONTROL',color=(90,205,220))
        def toggle(sender, value):
            shared.set_individual(value);refresh_controls(dpg,shared)
        def grip(sender, value):
            shared.set_gripper(value);refresh_controls(dpg,shared)
        dpg.add_checkbox(label='Individual finger control',tag='individual_control',default_value=True,
                         enabled=False,callback=toggle)
        with dpg.group(tag='gripper_controls',show=False):
            dpg.add_slider_float(label='Five-finger grasp' if whole else 'Two-finger gripper',tag='gripper_percent',min_value=0,max_value=100,
                default_value=0,format='%.1f %%',clamped=True,width=650,callback=grip)
            if whole:
                dpg.add_text('0-20%: thumb rotation 0-100 deg | 20-95%: all five bends | 95-100%: +5 deg each')
                dpg.add_text('At 95%: thumb/index/middle/ring/little = 13/53/56/55/52 deg. All stages linear.')
            else:
                dpg.add_text('0-20%: thumb rotation 0-75 deg | 20-95%: bend | 95-100%: final closure')
                dpg.add_text('Linear in each stage. Middle / ring / little fingers stay at 90 deg.')
        dpg.add_text(f'Arm: selected gravity + {args.friction_scale:.0%} friction   |   Hand: mode {args.hand_mode}')
        dpg.add_text('LIVE' if args.execute else 'OFFLINE PREVIEW - no robot connection',tag='status')
        dpg.add_separator()
        dpg.add_text('Requested: Arm TX/RX 500/500 Hz   |   Hand TX/RX 50/50 Hz')
        dpg.add_text('Actual rates: waiting...',tag='rates')
        dpg.add_text('Hand angle/current feedback rates: waiting...',tag='field_rates')
        dpg.add_text('Feedback age: waiting...',tag='ages')
        dpg.add_spacer(height=10)
        with dpg.table(header_row=True,borders_innerH=True,policy=dpg.mvTable_SizingStretchProp):
            for label in ('Joint','Target angle (degrees)','Actual (deg)','Current (mA)'):
                dpg.add_table_column(label=label,init_width_or_weight=3 if label.startswith('Target') else 1)
            for i,name in enumerate(HAND_NAMES):
                with dpg.table_row():
                    dpg.add_text(name)
                    with dpg.group():
                        dpg.add_slider_float(tag=f'slider{i}',min_value=0,max_value=float(np.rad2deg(shared.hand_max_rad[i])),
                            format='%.2f deg',width=-1,enabled=not args.execute,
                            callback=lambda sender,value,index:shared.set_joint(index,value),user_data=i)
                        dpg.add_text('--',tag=f'target{i}',show=False)
                    dpg.add_text('--',tag=f'q{i}')
                    dpg.add_text('--',tag=f'current{i}')
        dpg.add_spacer(height=12)
        dpg.add_text('Arm angles (deg): --',tag='arm')
        dpg.add_text('Hand mode: --',tag='mode')
        dpg.add_text('Log: --',tag='log',wrap=900)
        dpg.add_spacer(height=10)
        dpg.add_button(label='Stop and close',callback=lambda:dpg.stop_dearpygui(),width=160)
    dpg.setup_dearpygui();dpg.set_primary_window('main',True)


def main(gripper_profile='two_finger'):
    p=arm_parser('gravity_friction')
    p.set_defaults(identified=str(EXAMPLES/'profiles/gravity_selected.json'),
                   friction=str(EXAMPLES/'profiles/friction_selected.json'),friction_scale=.6,seconds=0,hz=500,
                   gripper_profile=gripper_profile)
    p.add_argument('--hand-mode',type=int,choices=(2,3),default=3)
    p.add_argument('--hand-current-ma',type=int,default=500)
    p.add_argument('--hand-duration-ms',type=int,default=20,help='Mode 2 only; mode 3 uses streamed angle targets')
    p.add_argument('--preview-seconds',type=float,default=0,help='Auto-close offline GUI after N seconds')
    args=p.parse_args()
    if args.hz!=500:p.error('This demo uses arm 500 Hz and hand 50 Hz')
    if not 0<=args.hand_current_ma<=65535 or not 1<=args.hand_duration_ms<=65535:p.error('Hand fields must fit uint16')
    if not np.isfinite(args.seconds+args.preview_seconds) or min(args.seconds,args.preview_seconds)<0:p.error('Invalid duration')
    if not np.isfinite(args.friction_scale+args.friction_deadband+args.friction_smooth+args.gravity_scale) or min(args.friction_scale,args.friction_deadband,args.gravity_scale)<0 or args.friction_smooth<=0:p.error('Invalid compensation settings')
    if args.execute and args.preview_seconds:p.error('--preview-seconds is only for offline previews')
    if args.resume:p.error('GUI logs use a new directory for each run')
    import dearpygui.dearpygui as dpg
    shared=Shared(gripper_profile);stop=threading.Event();worker=None;initialized=False
    try:
        build_gui(dpg,args,shared);dpg.show_viewport()
        if args.execute:
            worker=threading.Thread(target=run_control,args=(args,shared,stop),name='robot-control')
            worker.start()
        else:shared.initialize([0.]*6)
        begin=time.perf_counter()
        while dpg.is_dearpygui_running():
            dpg.run_callbacks(dpg.get_callback_queue())
            view=shared.snapshot()
            if view['ready'] and not initialized:
                for i,value in enumerate(view['initial_deg']):
                    dpg.set_value(f'slider{i}',value)
                    dpg.configure_item(f'slider{i}',enabled=True,min_value=min(0.,value),max_value=max(float(np.rad2deg(shared.hand_max_rad[i])),value))
                initialized=True
            refresh_controls(dpg,shared)
            if args.execute:
                dpg.set_value('status',view['status'])
            feedback=view.get('feedback',{})
            for i in range(6):
                angles=feedback.get('q_rad');currents=feedback.get('current_ma')
                dpg.set_value(f'q{i}',f'{np.rad2deg(angles[i]):.2f}' if angles is not None else '--')
                dpg.set_value(f'current{i}',f'{currents[i]:.0f}' if currents is not None else '--')
            rates=view.get('rates',{});fields=rates.get('can_hand_fields_hz',[None]*5)
            fmt=lambda x:'--' if x is None else f'{x:.1f}'
            dpg.set_value('rates',f"Actual: Arm TX/RX {fmt(view.get('arm_tx_hz'))}/{fmt(rates.get('pc_arm_rx_hz'))} Hz   |   Hand TX/RX {fmt(view.get('hand_tx_hz'))}/{fmt(rates.get('pc_hand_rx_hz'))} Hz")
            dpg.set_value('field_rates',f'Hand angles: {fmt(fields[1])} Hz   |   Hand current: {fmt(fields[3])} Hz')
            dpg.set_value('ages',f"Feedback age: angle {fmt(feedback.get('angle_age_ms'))} ms / current {fmt(feedback.get('current_age_ms'))} ms")
            dpg.set_value('mode',f"Hand mode: {feedback.get('mode','--')}")
            dpg.set_value('arm','Arm angles (deg): '+str([round(x,1) for x in view.get('arm_q_deg',[])]))
            dpg.set_value('log','Log: '+view.get('log','--'))
            dpg.render_dearpygui_frame()
            if not args.execute and args.preview_seconds and time.perf_counter()-begin>=args.preview_seconds:break
    except KeyboardInterrupt:pass
    finally:
        stop.set()
        if worker is not None:
            while worker.is_alive():worker.join(.05)
        dpg.destroy_context()
    return 1 if shared.snapshot().get('error') else 0


if __name__=='__main__':raise SystemExit(main())

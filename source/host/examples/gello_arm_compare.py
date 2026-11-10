"""Read COM5 GELLO positions/speeds beside the arm's gravity-compensated state.

GELLO data is display/log input only: it never becomes an arm target.
All new implementation and the read-only driver reside under examples/.
"""
from dataclasses import asdict
from datetime import datetime
import csv
import json
from pathlib import Path
import sys
import threading
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import arm_runtime
import numpy as np
from arm_dynamics import Gravity,read_json,write_json
from arm_lab import Session,ZERO,parser as arm_parser
from robot_rates import StateRates
from gello_compare_driver.reader import GelloReader,SERVO_SIGNS,RADIANS_PER_TICK

HERE=Path(__file__).resolve().parent
GROUP_LABELS=('Group 0: IDs 1-7','Group 1: IDs 11-17')
MUJOCO_CONFIG=HERE/'遥操资料_20261030_完整版(2)/遥操资料_20261030_完整版/MJC-Dishwasher_Gello遥操/gello_config.json'


def gello_values(group, now, rpm_per_unit=.732, stale_s=.25):
    """Individual samples go missing on timeout/staleness, not to zero."""
    raw=np.full(6,np.nan);ticks=raw.copy();velocity=raw.copy();ages=raw.copy();reader_deg=raw.copy()
    if group is not None:
        for i in range(6):
            stamp=group.servo_captured_at[i]
            if stamp is not None:ages[i]=max(0.,now-stamp)*1000
            if stamp is None or now-stamp>stale_s:continue
            if group.raw_positions[i] is not None:raw[i]=group.raw_positions[i]*360/4096
            if group.unwrapped_ticks[i] is not None:ticks[i]=group.unwrapped_ticks[i]
            angle=getattr(group,'arm_radians',(None,)*6)[i]
            if angle is not None:reader_deg[i]=np.rad2deg(angle)
            if group.raw_velocities[i] is not None:
                velocity[i]=group.raw_velocities[i]*rpm_per_unit*6*SERVO_SIGNS[i]
    return dict(raw_deg=raw,ticks=ticks,velocity_deg_s=velocity,age_ms=ages,reader_deg=reader_deg)


class Comparison:
    def __init__(self, config=None):
        self.reference={};self.flips={0:np.ones(6),1:np.ones(6)}
        self.config=config
        self.factors={};self.offsets={}
        for group,name in ((0,'left'),(1,'right')):
            axes=config[name]['axes'] if config else [dict(sign=1,scale=1,offset_rad=0)]*6
            self.factors[group]=np.array([a['sign']*a['scale'] for a in axes])
            self.offsets[group]=np.rad2deg([a['offset_rad'] for a in axes])

    def capture(self, group, values, arm):
        if arm is None or not np.all(np.isfinite(values['ticks'])) or not np.all(np.isfinite(arm['q'])):
            return False
        self.reference[group]=(values['ticks'].copy(),np.array(arm['q']))
        return True

    def calculate(self, group, values, arm):
        q=np.full(6,np.nan) if arm is None else np.rad2deg(arm['q'])
        v=np.full(6,np.nan) if arm is None else np.rad2deg(arm['dq'])
        gd=np.full(6,np.nan);ad=gd.copy()
        if group in self.reference:
            gr,ar=self.reference[group]
            gd=(values['ticks']-gr)*360/4096*np.array(SERVO_SIGNS[:6])*self.factors[group]*self.flips[group]
            ad=q-np.rad2deg(ar)
        mapped=values['reader_deg']*self.factors[group]*self.flips[group]+self.offsets[group]
        return dict(g_raw=values['raw_deg'],g_mujoco=mapped,g_delta=gd,a_q=q,a_delta=ad,error=mapped-q,
                    g_v=values['velocity_deg_s']*self.factors[group]*self.flips[group],a_v=v)


class ArmView:
    def __init__(self):
        self.lock=threading.Lock();self.value=dict(status='Not connected',sample=None,rates={},tx_hz=None)
    def update(self,**kwargs):
        with self.lock:self.value.update(kwargs)
    def snapshot(self):
        with self.lock:return dict(self.value)


def run_arm(args, view, stop):
    """Only selected gravity feedforward; no GELLO object enters this worker."""
    try:
        model=Gravity(args.urdf,read_json(args.mapping) if args.mapping else None,args.identified,
                      args.payload_mass,args.payload_com,args.gravity)
        with Session(args,model) as session:
            view.update(status='Gravity compensation running')
            stats=StateRates();last=0.;rate_at=time.perf_counter();previous=0;tx=None
            while not stop.is_set():
                state=session.state()
                session.step('run','gello_compare',ZERO,ZERO,ZERO,ZERO,state=state)
                now=time.perf_counter()
                if now-last>=.01:
                    with session.robot._lock:counts=dict(session.robot.count)
                    measured=stats.update(now,counts,session.sent,session.robot.snapshot())
                    if now-rate_at>=1:
                        tx=(session.sent-previous)/(now-rate_at);previous=session.sent;rate_at=now
                    view.update(sample=dict(q=list(state['q_rad']),dq=list(state['dq_rad_s']),
                        sequence=state['header']['sequence'],at=now,
                        feedback_age_us=list(state['feedback_age_us'])),rates=measured,tx_hz=tx)
                    last=now
    except Exception as exc:
        view.update(status='Arm error: '+str(exc),error=str(exc),sample=None)
        print('ARM ERROR:',repr(exc),flush=True)
    finally:
        if not view.snapshot().get('error'):view.update(status='Stopped',sample=None)


def display_number(value):
    return '--' if value is None or not np.isfinite(value) else f'{value:.2f}'


def build_gui(dpg,args,comparison,context):
    dpg.create_context();dpg.configure_app(manual_callback_management=True)
    dpg.create_viewport(title='GELLO COM5 | Panthera angle + velocity comparison',width=1320,height=720,min_width=1150,min_height=650)
    with dpg.window(tag='main'):
        dpg.add_text('GELLO / PANTHERA  -  ANGLE & DIRECTION COMPARISON',color=(90,205,220))
        dpg.add_text('GELLO is read-only. Arm uses the selected gravity model; no motion-following targets.')
        with dpg.group(horizontal=True):
            def select_group(sender,value):
                context['group']=GROUP_LABELS.index(value)
                for i in range(6):dpg.set_value(f'flip{i}',bool(comparison.flips[context['group']][i]<0))
                context['g_values']=None
                if context.get('reader'):context['reader'].group_mask=1<<context['group']
                dpg.set_value('reference_status','Reference captured for this group' if context['group'] in comparison.reference else 'Reference: waiting for valid data')
            dpg.add_combo(GROUP_LABELS,tag='group_select',default_value=GROUP_LABELS[args.group],width=230,callback=select_group)
            def reference():
                ok=comparison.capture(context['group'],context['g_values'],context['arm']) if context.get('g_values') is not None else False
                dpg.set_value('reference_status','Reference captured at current poses' if ok else 'Need six fresh GELLO + arm angles')
            dpg.add_button(label='Capture comparison reference',callback=reference)
            def zero():
                if context.get('reader'):
                    context['reader'].capture_zero(context['group'])
                    comparison.reference.pop(context['group'],None)
                    dpg.set_value('reference_status','GELLO software zero requested at current pose')
            dpg.add_button(label='Set GELLO zero here',callback=zero)
            dpg.add_text('Reference: waiting for valid data',tag='reference_status')
        dpg.add_text('G MuJoCo = startup-zeroed angle x reader sign x config sign/scale + config offset. Neutral = 0.')
        dpg.add_text('Group 0 uses left config; group 1 uses right config. swap_arms only swaps simulated destination.')
        dpg.add_text('Start with GELLO at physical zero, or click Set GELLO zero here. This does not change the robot zero.')
        dpg.add_text(f'Serial {args.port} / {args.baud} baud / speed unit {args.velocity_rpm_per_unit} RPM per unit')
        dpg.add_text('GELLO: waiting...',tag='serial_status')
        dpg.add_text('Arm: waiting...',tag='arm_status')
        dpg.add_text('Rates: waiting...',tag='rates')
        dpg.add_separator()
        with dpg.table(header_row=True,borders_innerH=True,borders_innerV=True,policy=dpg.mvTable_SizingStretchProp):
            for label in ('Joint / ID','Flip','G MuJoCo deg','G delta deg','Arm deg','Arm delta deg','Angle error','G deg/s','Arm deg/s','G age ms'):
                dpg.add_table_column(label=label,init_width_or_weight=.5 if label=='Flip' else 1)
            for i in range(6):
                with dpg.table_row():
                    dpg.add_text(f'J{i+1}',tag=f'joint{i}')
                    def flip(sender,value,index):comparison.flips[context['group']][index]=-1 if value else 1
                    dpg.add_checkbox(tag=f'flip{i}',callback=flip,user_data=i)
                    for field in ('g_mujoco','g_delta','a_q','a_delta','error','g_v','a_v','age_ms'):
                        dpg.add_text('--',tag=f'{field}{i}')
        dpg.add_spacer(height=10)
        dpg.add_text('Gripper raw values (not normalized; open/closed calibration pending)',color=(90,205,220))
        dpg.add_text('ID7 raw ticks: --',tag='gripper0')
        dpg.add_text('ID17 raw ticks: --',tag='gripper1')
        dpg.add_text('Direction check: move one joint at a time; compare signs of delta and velocity.')
        dpg.add_text('Place both arms in matching poses, then capture a reference to compare motion amplitudes.')
        dpg.add_text('Log: '+context.get('log','offline preview'),wrap=1250)
        dpg.add_button(label='Stop and close',callback=lambda:dpg.stop_dearpygui(),width=160)
    dpg.setup_dearpygui();dpg.set_primary_window('main',True)


def main():
    p=arm_parser('gravity')
    p.set_defaults(identified=str(HERE/'profiles/gravity_selected.json'),seconds=0)
    p.add_argument('--port',default='COM5');p.add_argument('--baud',type=int,default=1000000)
    p.add_argument('--serial-timeout-ms',type=float,default=30)
    p.add_argument('--velocity-rpm-per-unit',type=float,default=.732)
    p.add_argument('--group',type=int,choices=(0,1),default=1,help='Selected GELLO group to read and display')
    p.add_argument('--gello-config',default=str(MUJOCO_CONFIG))
    p.add_argument('--gello-only',action='store_true',help='Read COM5 but do not connect to the arm')
    p.add_argument('--preview-seconds',type=float,default=0)
    args=p.parse_args()
    live=args.execute or args.gello_only
    if args.execute and args.gello_only:p.error('Choose --execute or --gello-only')
    if args.resume:p.error('Use a new output directory for this comparison')
    if args.hz!=500:p.error('Arm comparison uses 500 Hz')
    if not np.isfinite(args.serial_timeout_ms+args.velocity_rpm_per_unit) or min(args.serial_timeout_ms,args.velocity_rpm_per_unit)<=0 or args.baud<=0:p.error('Invalid serial settings')
    if not np.isfinite(args.seconds+args.preview_seconds) or min(args.seconds,args.preview_seconds)<0:p.error('Invalid duration')
    if live and args.preview_seconds:p.error('--preview-seconds requires offline mode')
    if not np.isfinite(args.gravity_scale) or args.gravity_scale<0:p.error('Invalid gravity scale')
    import dearpygui.dearpygui as dpg
    mapping_config=read_json(args.gello_config)
    compare=Comparison(mapping_config);context=dict(group=args.group);arm_view=ArmView();stop=threading.Event()
    reader=None;worker=None;file=None;grip_file=None;session_dir=None
    try:
        if live:
            session_dir=Path(args.output or HERE/'logs'/('gello_compare_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
            session_dir.mkdir(parents=True,exist_ok=False);context['log']=str(session_dir.resolve())
            write_json(session_dir/'comparison_metadata.json',dict(arguments=vars(args),reader_signs=SERVO_SIGNS[:6],
                position_ticks_per_revolution=4096,velocity_source='servo address 58 signed-magnitude; not finite-difference',
                mujoco_mapping=mapping_config,reference='MuJoCo startup software zero + config signs/scales/offsets; no hardware zero written',csv='GUI snapshots at up to display frame rate; each group sample timestamp is recorded'))
            file=(session_dir/'comparison.csv').open('w',newline='',encoding='utf-8');writer=csv.writer(file)
            writer.writerow(['pc_time','group','joint','servo_id','gello_sequence','arm_sequence','flip',
                'g_mujoco_deg','g_delta_deg','arm_deg','arm_delta_deg','angle_error_deg','g_deg_s','arm_deg_s','g_age_ms','g_raw_deg'])
            grip_file=(session_dir/'gripper_raw.csv').open('w',newline='',encoding='utf-8')
            grip_writer=csv.writer(grip_file)
            grip_writer.writerow(['pc_time','gello_sequence','group','servo_id','raw_ticks','unwrapped_ticks','raw_velocity','age_ms','fresh'])
        build_gui(dpg,args,compare,context);dpg.show_viewport()
        if live:
            reader=GelloReader(args.port,baudrate=args.baud,group_mask=1<<args.group,timeout_s=args.serial_timeout_ms/1000,
                read_velocity=True,velocity_rpm_per_unit=args.velocity_rpm_per_unit)
            context['reader']=reader
            reader.start()
        if args.execute:
            args.output=str(session_dir/'arm')
            worker=threading.Thread(target=run_arm,args=(args,arm_view,stop),name='arm-gravity')
            worker.start()
        begin=time.perf_counter();last_logged=-1;last_flush=begin
        while dpg.is_dearpygui_running():
            dpg.run_callbacks(dpg.get_callback_queue());now=time.perf_counter()
            arm_info=arm_view.snapshot();arm=arm_info['sample']
            if arm is not None and now-arm['at']>.25:arm=None
            snapshot=reader.latest_snapshot() if reader else None
            group=snapshot.group(context['group']) if snapshot else None
            values=gello_values(group,now,args.velocity_rpm_per_unit)
            context.update(g_values=values,arm=arm)
            if context['group'] not in compare.reference and compare.capture(context['group'],values,arm):
                dpg.set_value('reference_status','Initial poses captured; re-capture after physically aligning')
            result=compare.calculate(context['group'],values,arm)
            for i in range(6):
                node=(1 if context['group']==0 else 11)+i
                dpg.set_value(f'joint{i}',f'J{i+1} / {node}')
                for key in ('g_mujoco','g_delta','a_q','a_delta','error','g_v','a_v'):
                    dpg.set_value(f'{key}{i}',display_number(result[key][i]))
                dpg.set_value(f'age_ms{i}',display_number(values['age_ms'][i]))
            if reader:
                status=reader.connection_status;stats=reader.stats
                dpg.set_value('serial_status',f'GELLO: {status.state.value} | timeouts {stats.servo_timeouts} | checksum {stats.checksum_errors} | status errors {stats.status_errors} | '+(status.last_error or ''))
                scan_hz=stats.scan_hz
            else:
                dpg.set_value('serial_status','OFFLINE PREVIEW - COM5 is not opened');scan_hz=None
            dpg.set_value('arm_status','Arm: '+arm_info['status'])
            dpg.set_value('rates',f"GELLO full-bus scans {display_number(scan_hz)} Hz | Arm TX {display_number(arm_info['tx_hz'])} / RX {display_number(arm_info['rates'].get('pc_arm_rx_hz'))} Hz")
            for gi in (0,1):
                gg=snapshot.group(gi) if snapshot else None
                valid=gg is not None and gg.servo_captured_at[6] is not None and now-gg.servo_captured_at[6]<.25
                grip=f"ID{7 if gi==0 else 17} raw ticks: -- (waiting/stale)"
                if valid:
                    grip=f'ID{gg.servo_ids[6]} raw ticks: {gg.raw_positions[6]} | unwrapped: {gg.unwrapped_ticks[6]} | raw speed: {gg.raw_velocities[6]} | age: {(now-gg.servo_captured_at[6])*1000:.1f} ms'
                dpg.set_value(f'gripper{gi}',grip)
            if file and snapshot and snapshot.sequence!=last_logged:
                for g in snapshot.groups:
                    val=gello_values(g,now,args.velocity_rpm_per_unit);data=compare.calculate(g.group,val,arm)
                    stamp=g.servo_captured_at[6]
                    grip_writer.writerow([now,snapshot.sequence,g.group,g.servo_ids[6],g.raw_positions[6],g.unwrapped_ticks[6],g.raw_velocities[6],
                        None if stamp is None else (now-stamp)*1000,bool(stamp is not None and now-stamp<.25)])
                    for i in range(6):writer.writerow([now,g.group,i+1,g.servo_ids[i],snapshot.sequence,None if arm is None else arm['sequence'],compare.flips[g.group][i]]+
                        [None if not np.isfinite(data[k][i]) else data[k][i] for k in ('g_mujoco','g_delta','a_q','a_delta','error','g_v','a_v')]+[val['age_ms'][i],val['raw_deg'][i]])
                last_logged=snapshot.sequence
            if file and now-last_flush>=1:file.flush();grip_file.flush();last_flush=now
            dpg.render_dearpygui_frame()
            if args.seconds and now-begin>=args.seconds:break
            if not live and args.preview_seconds and now-begin>=args.preview_seconds:break
    except KeyboardInterrupt:pass
    finally:
        stop.set()
        if reader:reader.stop()
        if worker:
            while worker.is_alive():worker.join(.05)
        if file:file.close()
        if grip_file:grip_file.close()
        if session_dir:
            write_json(session_dir/'comparison_summary.json',dict(arm=arm_view.snapshot(),
                gello_stats=asdict(reader.stats) if reader else None,
                gello_error=str(reader.fatal_error) if reader and reader.fatal_error else None,
                flips={str(k):v.tolist() for k,v in compare.flips.items()},
                references={str(k):dict(gello_ticks=v[0].tolist(),arm_q_rad=v[1].tolist()) for k,v in compare.reference.items()}))
            print('Saved:',session_dir.resolve())
        dpg.destroy_context()
    return 1 if arm_view.snapshot().get('error') or (reader and reader.fatal_error) else 0


if __name__=='__main__':raise SystemExit(main())

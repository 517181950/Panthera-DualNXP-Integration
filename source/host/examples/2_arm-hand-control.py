"""Direct Run: six arm axes500Hz zero MIT + six hand actuators50Hz together.
Zero MIT is q=dq=kp=kd=tau_ff=0, not a position hold. Fixed compliance500mA.
"""
from pathlib import Path
import sys,time,json,argparse,math,datetime
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_sdk import Robot
from robot_rates import StateRates
OPEN=[0.,0.,0.,0.,0.,0.]
CLOSE=[1.,.524,1.57,1.57,1.57,1.57]
def main():
 p=argparse.ArgumentParser();p.add_argument("--mode-only",action="store_true");p.add_argument("--mode",type=int,choices=(1,2,3),default=2);p.add_argument("--seconds",type=float,default=15);p.add_argument("--hand-trajectory",choices=("sine","remaining","legacy"),default="remaining");p.add_argument("--hand-duration-ms",type=int,default=10);p.add_argument("--hand-control-hz",type=int,choices=(50,100,200,500),default=50);p.add_argument("--hand-hz",type=int,choices=(50,100,200,500),default=50);p.add_argument("--full-state",action="store_true",default=True);p.add_argument("--split-state",dest="full_state",action="store_false");p.add_argument("--angle-feedback-only",action="store_true",default=True);p.add_argument("--fixed-hand-read",action="store_true",default=True);p.add_argument("--check-modes",action="store_true");p.add_argument("--velocity",action="store_true");p.add_argument("--report",default=str(Path(__file__).resolve().parents[1]/"test_demo"/"logs"/("zero-mit-full-state_"+datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))/"result.json"));a=p.parse_args()
 a.full_state=False if a.angle_feedback_only else a.full_state
 r=Robot().start();rows=[];events=[];sent=0;rates=StateRates();status=0;zero_load_started=False
 def operation(label,value):
  events.append(dict(kind=label,result=value));print(label,value,flush=True)
  if value.get("status",-1):raise RuntimeError(str(value))
 try:
  if not r.discover(10):raise RuntimeError("board missing")
  operation("claim",r.operation(1))
  operation("read_profile",r.start_state_stream(motor_query_hz=0 if a.mode_only else 500,hand_query_hz=a.hand_hz,hand_angles_only=True,hand_current_fast=not a.angle_feedback_only,hand_velocity_fast=a.velocity and not a.angle_feedback_only,hand_full_state=a.full_state,legacy_full_checksum=a.full_state,hand_reply_driven=not a.fixed_hand_read))
  def switch_mode(mode):
   operation("hand_mode_confirmed",r.prepare_hand_mode(mode))
   events.append(dict(kind="hand_mode_readback",mode=mode))
  switch_mode(a.mode if a.mode_only else 2)
  if a.mode_only:return 0
  operation("payload",r.configure(arm_state_period_us=2000,hand_state_period_us=round(1000000/a.hand_hz),hand_mode=2,hand_soft_current_ma=[500]*6,hand_control_feedforward_ma=500))
  operation("zero_MIT_all_axes",r.operation(13,target_mask=1));zero_load_started=True;operation("active",r.operation(3))
  if a.check_modes:
   for mode in (1,2):switch_mode(mode)
  baseline=r.snapshot();events.append(dict(kind="load_baseline",snapshot=baseline))
  feedback_rows=[];hand_sent=0;last_hand_sequence=0;next_hand=0
  start=time.perf_counter();deadline=start;slot=-1;last_print=-1
  while time.perf_counter()-start<a.seconds:
   t=time.perf_counter()-start
   r.send_state_probe();z=r.send_arm([0]*6,[0]*6,[0]*6,[0]*6,[0]*6,joint_mask=63);sent+=int(z.ok)
   if a.hand_trajectory=="sine":
    blend=.5*(1-math.cos(math.pi*t));q=[lo+(hi-lo)*blend for lo,hi in zip(OPEN,CLOSE)];duration_ms=a.hand_duration_ms
   else:
    leg=int(t);q=OPEN if leg%2==0 else CLOSE
    duration_ms=max(1,round((1-(t-leg))*1000)) if a.hand_trajectory=="remaining" else 1000
   if t>=next_hand:
    hand_result=r.send_hand(q,duration_ms);hand_sent+=int(hand_result.ok);last_hand_sequence=hand_result.sequence;next_hand+=1/a.hand_control_hz
    h=r.snapshot().get("HandState",{});feedback_rows.append(dict(t=t,target=q,duration_ms=duration_ms,ok=hand_result.ok,online=h.get("online"),q=[h.get("joint_q_rad",[0]*11)[i] for i in (0,1,2,4,6,8)],age_us=h.get("field_age_us",[0]*5)[1]))
    if next_hand<t:next_hand=t+1/a.hand_control_hz
   if t-last_print>=1:
    snap=r.snapshot()
    with r._lock:counts=dict(r.count)
    v=rates.update(time.perf_counter(),counts,sent,snap)
    row=dict(t=t,hand_targets_sent=hand_sent,hand_last_sequence=last_hand_sequence,hand_target=q,hand_q=snap.get("HandState",{}).get("joint_q_rad"),hand_current_ma=snap.get("HandState",{}).get("motor_current_ma"),hand_velocity=snap.get("HandState",{}).get("motor_dq_rad_s"),rates=dict(v),hand_online=snap.get("HandState",{}).get("online"),mode=snap.get("HandState",{}).get("mode_raw"),age=snap.get("HandState",{}).get("field_age_us"),motor_modes=snap.get("ArmState",{}).get("motor_mode_raw"));rows.append(row);print(json.dumps(row),flush=True);last_print=t
   deadline+=.002;delay=deadline-time.perf_counter()
   if delay>0:time.sleep(delay)
   elif delay<-.01:deadline=time.perf_counter()
 except KeyboardInterrupt:pass
 except Exception as exc:status=1;events.append(dict(kind="error",error=repr(exc)));print(repr(exc),flush=True)
 finally:
  final=r.snapshot()
  if r.lease:
   if zero_load_started:r.operation(13,target_mask=0)
   r.operation(4);r.operation(2)
  r.stop_state_stream();r.close()
  if a.report:
   out=Path(a.report);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(dict(parameters=vars(a),feedback_rows=locals().get("feedback_rows",[]),events=events,rows=rows,final=final,status=status,pc_arm_commands=sent,pc_hand_commands=locals().get("hand_sent",0)),indent=2),encoding="utf-8")
 return status
if __name__=="__main__":raise SystemExit(main())

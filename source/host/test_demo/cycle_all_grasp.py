"""J4/J6/whole-hand requested cycle, with independently recorded pushed state.
Tracking error is diagnostic only; fixed hand compliance, explicit operator STOP.
"""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from demo_paths import default_log_directory
import argparse,json,math,time,threading,os
from pathlib import Path
from robot_sdk import Robot
from session_capture import Capture

def main():
 p=argparse.ArgumentParser();p.add_argument("--preview",action="store_true",help="Only print settings; default runs real hardware");p.add_argument("--out",default=default_log_directory(__file__));p.add_argument("--forever",action="store_true");p.add_argument("--seconds",type=float,default=None);args=p.parse_args()
 if args.preview:
  print("Preview only: fixed hand compliance; no automatic mode switching. Running without --preview connects to real hardware.");return
 out=Path(args.out);out.mkdir(parents=True,exist_ok=True);(out/"pid.txt").write_text(str(os.getpid()))
 capture=Capture(out);r=Robot();capture.attach(r);r.start();stop=threading.Event();events=(out/"cycle_events.jsonl").open("w",encoding="utf-8",buffering=1)
 mode_request=[None];status={"state":"starting","pid":os.getpid()};maxerror=[0.,0.];other_delta=[0.]*6;mode_thread=None
 def event(kind,**data):
  events.write(json.dumps(dict(pc_ns=time.perf_counter_ns(),kind=kind,**data),ensure_ascii=False)+"\n")
 def checked(name,result):
  event(name,result=result)
  if result.get("status",-1):raise RuntimeError(name+str(result))
 def wait(t):time.sleep(t)
 try:
  if not r.discover(12):raise RuntimeError("board not discovered")
  if not capture.ready.wait(8):raise RuntimeError("CAN trace recorder not ready")
  wait(.5);baseline=r.snapshot();(out/"baseline.json").write_text(json.dumps(baseline,indent=2))
  a=baseline["ArmState"];h=baseline["HandState"]
  if a["valid_q_mask"]!=63 or max(a["feedback_age_us"])>100000:raise RuntimeError("fresh encoder feedback unavailable")
  if not h["online"] or (h["valid_fields"]&34)!=34 or h["field_age_us"][1]>100000:raise RuntimeError("fresh hand status missing; no mode/arm commands allowed")
  checked("claim",r.operation(1));checked("configuration",r.configure(hand_mode=2,hand_soft_current_ma=[0,500,500,500,500,500],hand_control_feedforward_ma=500,hand_position_current_limit_ma=[1000]*6))
  checked("hand_mode_confirmed",r.prepare_hand_mode(2))
  if r.snapshot()["ArmState"]["motor_mode_raw"][3]!=21 or r.snapshot()["ArmState"]["feedback_age_us"][3]>10000:raise RuntimeError("initial J4 enable not confirmed")
  q0=a["q_rad"][:];q=q0[:];zero=[0.]*6;kp=zero[:];kd=zero[:];kp[3]=kp[5]=10;kd[3]=kd[5]=.7
  hand=[h["joint_q_rad"][i] for i in (0,1,2,4,6,8)]
  r.send_arm(q,zero,kp,kd,zero,joint_mask=40);checked("activate",r.operation(3))
  # User cancelled all periodic physical mode changes.
  start=time.perf_counter();deadline=start;approach=18.;first_cycle=None;last_mode_slot=-1;last_hand=-1;last_status=-1;count=0;qualified=False
  target0=q0[:];target0[3]=math.radians(30);target0[5]=math.radians(-90)
  while not (out/"STOP").exists():
   now=time.perf_counter();elapsed=now-start;dq=zero[:]
   if elapsed<approach:
    u=elapsed/approach;blend=10*u**3-15*u**4+6*u**5;derivative=(30*u*u-60*u**3+30*u**4)/approach
    for j in (3,5):q[j]=q0[j]+(target0[j]-q0[j])*blend;dq[j]=(target0[j]-q0[j])*derivative
    state="approach"
   else:
    t=elapsed-approach
    if first_cycle is None:
     first_cycle=now;snap=r.snapshot();errors=[abs(math.degrees(snap["ArmState"]["q_rad"][j]-target0[j])) for j in (3,5)]
     event("approach_complete",error_deg=errors)
     # Tracking error is reported and never stops the trajectory.
    q[3]=math.radians(45-15*math.cos(math.pi*t));dq[3]=math.radians(15*math.pi*math.sin(math.pi*t))
    q[5]=math.radians(-90*math.cos(math.pi*t));dq[5]=math.radians(90*math.pi*math.sin(math.pi*t))
    hand[1]=.524 if int(t)%2==0 else 0.0
    slot=int(t)
    if slot!=last_mode_slot:
     last_mode_slot=slot;event("fixed_mode",mode=2,t=t)
    state="cycling"
    snap=r.snapshot();feedback=snap["ArmState"]["q_rad"]
    for k,j in enumerate((3,5)):maxerror[k]=max(maxerror[k],abs(math.degrees(feedback[j]-q[j])))
    for j in (0,1,2,4):other_delta[j]=max(other_delta[j],abs(math.degrees(feedback[j]-q0[j])))
    if t>=4 and not qualified:
     event("qualification",max_tracking_error_deg=maxerror,other_axis_delta_deg=other_delta)
     # Diagnostics only; no position-error stop or trajectory clamp.
     qualified=True
    if args.seconds is not None and t>=args.seconds:break
   result=r.send_arm(q,dq,kp,kd,zero,joint_mask=40)
   if not result.ok:event("send_error",error=result.error)
   # One timed thumb segment per second, aligned to the same 2-second cycle.
   thumb_slot=int(elapsed)
   if thumb_slot>=0 and thumb_slot!=last_hand:
    hand[1:]=[.524,1.57,1.57,1.57,1.57] if thumb_slot%2==0 else [0.]*5
    sent_hand=r.send_hand(hand,1000)
    event("whole_hand_segment",target_rad=hand[:],duration_ms=1000,sequence=sent_hand.sequence,ok=sent_hand.ok)
    last_hand=thumb_slot
   if elapsed-last_status>=1:
    snap=r.snapshot();status=dict(state=state,running=True,pid=os.getpid(),kp=10,kd=.7,grasp_endpoints_rad=[.524,1.57,1.57,1.57,1.57],hand_feedforward_ma=500,elapsed=elapsed,sent=count,q_deg=[math.degrees(x) for x in snap["ArmState"]["q_rad"]],mode=snap["ArmState"]["motor_mode_raw"],hand_mode=snap["HandState"]["mode_raw"],grasp_feedback_rad=[snap["HandState"]["joint_q_rad"][i] for i in (1,2,4,6,8)],max_tracking_error_deg=maxerror,other_axis_delta_deg=other_delta,diagnostics=snap.get("Diagnostics",{}))
    (out/"live_status.json").write_text(json.dumps(status,indent=2));print(json.dumps(status),flush=True);last_status=elapsed
   count+=1;deadline+=.002;delay=deadline-time.perf_counter()
   if delay>0:time.sleep(delay)
   elif delay<-.01:deadline=time.perf_counter()
  event("finished",sent=count,qualified=qualified)
 except Exception as exc:
  event("failure",error=repr(exc));status.update(state="failed",error=repr(exc));print("FAILED",repr(exc),flush=True)
 finally:
  stop.set()
  if mode_thread:mode_thread.join(4)
  if r.lease:
   event("leave_active",result=r.operation(4));event("release",result=r.operation(2))
  (out/"final.json").write_text(json.dumps(r.snapshot(),indent=2));status["running"]=False;(out/"live_status.json").write_text(json.dumps(status,indent=2));r.close();capture.close();events.close()
 return 1 if status.get("state")=="failed" else 0
if __name__=="__main__":raise SystemExit(main())

"""Fixed-compliance whole-hand open/close; no arm trajectory or mode switches."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from demo_paths import default_log_directory
import argparse,json,time,math,os
from pathlib import Path
from robot_sdk import Robot
from session_capture import Capture

def main():
 p=argparse.ArgumentParser();p.add_argument("--preview",action="store_true",help="Only print settings; default runs real hardware");p.add_argument("--out",default=default_log_directory(__file__));args=p.parse_args()
 if args.preview:print("Whole-hand target: rotation unchanged; grasp 0..[.524,1.57,1.57,1.57,1.57]rad,1000ms per leg, fixed compliance; existing J4/J6 hold current positions.");return
 out=Path(args.out);out.mkdir(parents=True,exist_ok=True);(out/"pid.txt").write_text(str(os.getpid()))
 c=Capture(out);r=Robot();c.attach(r);r.start();report={};events=(out/"events.jsonl").open("w",encoding="utf-8",buffering=1)
 def event(kind,**data):events.write(json.dumps(dict(pc_ns=time.perf_counter_ns(),kind=kind,**data))+"\n")
 try:
  if not r.discover(10):raise RuntimeError("board unavailable")
  c.ready.wait(5);time.sleep(.2);s=r.snapshot();a=s["ArmState"];h=s["HandState"]
  if not h["online"] or h["field_age_us"][1]>100000:raise RuntimeError("fresh compliance feedback required; no switch performed")
  report["before"]=s;claimed=r.operation(1);event("claim",result=claimed);prepared=r.prepare_hand_mode(2);event("hand_mode_confirmed",result=prepared)
  if prepared.get("status"):raise RuntimeError(str(prepared))
  if claimed["status"]:raise RuntimeError(str(claimed))
  configured=r.configure(hand_mode=2,hand_soft_current_ma=[0,500,500,500,500,500],hand_control_feedforward_ma=500);event("configure",result=configured)
  if configured["status"]:raise RuntimeError(str(configured))
  q=a["q_rad"][:];zero=[0.]*6;kp=zero[:];kd=zero[:];mask=0
  for j in (3,5):
   if a["motor_mode_raw"][j]==21 and a["feedback_age_us"][j]<10000:mask|=1<<j;kp[j]=10;kd[j]=.7
  if mask:r.send_arm(q,zero,kp,kd,zero,joint_mask=mask)
  active=r.operation(3);event("activate",result=active)
  if active["status"]:raise RuntimeError(str(active))
  rotation=h["joint_q_rad"][0];start=time.perf_counter();deadline=start;last_slot=-1;last_status=-1;sent=0
  while not (out/"STOP").exists():
   t=time.perf_counter()-start;slot=int(t)
   if mask:r.send_arm(q,zero,kp,kd,zero,joint_mask=mask)
   if slot!=last_slot:
    close=slot%2==1;target=[rotation,.524 if close else 0]+[1.57 if close else 0]*4
    result=r.send_hand(target,1000);event("hand_target",phase="grasp" if close else "open",target_rad=target,ok=result.ok,sequence=result.sequence);last_slot=slot
   if t-last_status>=.25:
    snap=r.snapshot();hh=snap["HandState"];status=dict(running=True,pid=os.getpid(),elapsed=t,phase="grasp" if slot%2 else "open",hand_mode=hh["mode_raw"],grasp_feedback_rad=[hh["joint_q_rad"][i] for i in (1,2,4,6,8)],arm_q_deg=[math.degrees(x) for x in snap["ArmState"]["q_rad"]],arm_hold_mask=mask,diagnostics=snap.get("Diagnostics",{}));(out/"live_status.json").write_text(json.dumps(status,indent=2));last_status=t
   sent+=1;deadline+=.002;dt=deadline-time.perf_counter()
   if dt>0:time.sleep(dt)
   elif dt<-.01:deadline=time.perf_counter()
 except Exception as exc:report["error"]=repr(exc);print("FAILED",repr(exc),flush=True)
 finally:
  if r.lease:event("leave_active",result=r.operation(4));event("release",result=r.operation(2))
  report["final"]=r.snapshot();r.close();c.close();events.close();(out/"result.json").write_text(json.dumps(report,indent=2));(out/"stopped.txt").write_text("stopped")
if __name__=="__main__":main()

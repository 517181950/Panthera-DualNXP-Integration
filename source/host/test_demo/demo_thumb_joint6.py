"""UDP SDK demonstration: J6 MIT + thumb; defaults to preview, --execute performs one excursion."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from demo_paths import default_log_directory
import argparse,json,math,time
from pathlib import Path
from robot_sdk import Robot
from session_capture import Capture

def run(execute=True,out=None,capture=None):
    plan=dict(joint_mask=32,other_arm_motion_writes=0,joint6_delta_deg=5,joint6_target_speed_deg_s=1,
              joint6_kp_nm_rad=60,joint6_kd_nm_s_rad=1.2,joint6_tau_ff_nm=0,
              thumb_targets_rad=[.45,"initial"],thumb_duration_ms=400,thumb_feedforward_ma=100,
              control_feedforward_ma=100,physical_mode_switch=False,gravity=False)
    if not execute:
        print(json.dumps(plan,indent=2));return
    recorder=Capture(capture) if capture else None
    r=Robot()
    if recorder:recorder.attach(r)
    r.start()
    log=dict(plan=plan,samples=[],operations=[],complete=False)
    def record(op,result):
        log["operations"].append(dict(operation=op,result=result));print(op,result,flush=True)
        if result.get("status",-1):raise RuntimeError(str(result))
    def wait(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:time.sleep(.002)
    try:
        if not r.discover(10):raise RuntimeError("no board telemetry")
        wait(1)
        snap=r.snapshot();a=snap["ArmState"];h=snap["HandState"]
        log["baseline"]=snap
        if a["valid_q_mask"]!=63 or max(a["feedback_age_us"])>100000:raise RuntimeError("fresh six-axis position feedback required")
        if not h["online"] or not h["valid_fields"]&2 or h["field_age_us"][1]>100000:raise RuntimeError("fresh hand angle feedback required")
        if any(a["fault_code_raw"]) or any(h["fault_code_raw"]):raise RuntimeError("reported device fault before demonstration")
        record("claim",r.operation(1))
        prepared=r.prepare_hand_mode(2);record("hand_mode_confirmed",prepared)
        if prepared.get("status"):raise RuntimeError(str(prepared))
        if h["mode_raw"]!=2:
            print("Hand mode readback",h["mode_raw"],"operator preset compliance; no physical mode command",flush=True)
        record("configure",r.configure(hand_mode=2,hand_soft_current_ma=[0,100,0,0,0,0],hand_control_feedforward_ma=100))
        q=a["q_rad"][:];initial=q[5];hand=[h["joint_q_rad"][i] for i in (0,1,2,4,6,8)]
        initial_thumb=hand[1];z=[0.]*6;kp=z[:];kd=z[:];kp[5]=60.;kd[5]=1.2
        sent=r.send_arm(q,z,kp,kd,z,joint_mask=32)
        if not sent.ok:raise RuntimeError(sent.error)
        record("activate",r.operation(3))
        start=time.perf_counter();deadline=start;last_sample=-1;hand_stage=-1;sent_count=0
        while True:
            now=time.perf_counter();t=now-start
            if t>=12:break
            # Finite requested target waveform; not a firmware or SDK limiter.
            offset=t if t<5 else 10-t if t<10 else 0
            q[5]=initial+math.radians(offset)
            dq=z[:];dq[5]=math.radians(1 if t<5 else -1 if t<10 else 0)
            result=r.send_arm(q,dq,kp,kd,z,joint_mask=32)
            if not result.ok:raise RuntimeError(result.error)
            sent_count+=1
            stage=0 if t<2 else 1 if t<7 else 2
            if stage!=hand_stage:
                hand[1]=.45 if stage==1 else initial_thumb
                result=r.send_hand(hand,400)
                if not result.ok:raise RuntimeError(result.error)
                hand_stage=stage;print("thumb target",hand[1],flush=True)
            # Background receiver records every pushed message independently.
            if t-last_sample>=.001:
                s=r.snapshot();aa=s["ArmState"];hh=s["HandState"]
                row=dict(t=t,q_rad=aa["q_rad"],dq_rad_s=aa["dq_rad_s"],mode=aa["motor_mode_raw"],arm_fault=aa["fault_code_raw"],
                         age_us=aa["feedback_age_us"],thumb_rad=hh["joint_q_rad"][1],hand_mode=hh["mode_raw"],hand_fault=hh["fault_code_raw"])
                log["samples"].append(row);last_sample=t
                if len(log["samples"])%500==0:print("t",round(t,1),"J6deg",round(math.degrees(row["q_rad"][5]),3),"thumb",round(row["thumb_rad"],3),flush=True)
            deadline+=.002;remaining=deadline-time.perf_counter()
            if remaining>0:time.sleep(remaining)
            elif remaining<-.01:deadline=time.perf_counter()
        q[5]=initial;r.send_arm(q,z,kp,kd,z,joint_mask=32);wait(1)
        log["final"]=r.snapshot();log["sent_arm_commands"]=sent_count;log["complete"]=True
        values=[x["q_rad"][5] for x in log["samples"]]
        log["observed_joint6_span_deg"]=math.degrees(max(values)-min(values))
        values=[x["thumb_rad"] for x in log["samples"]];log["observed_thumb_span_rad"]=max(values)-min(values)
        log["other_axis_max_delta_deg"]=[max(abs(math.degrees(x["q_rad"][i]-a["q_rad"][i])) for x in log["samples"]) for i in range(5)]
    except Exception as exc:
        log["error"]=repr(exc);print("DEMO",repr(exc),flush=True)
    finally:
        if r.lease:
            record_result=r.operation(4);log["operations"].append(dict(operation="end_demo_leave_active",result=record_result))
            log["operations"].append(dict(operation="release",result=r.operation(2)))
        log["sdk_stats"]=dict(r.stats);r.close()
        if recorder:recorder.close()
        if out:Path(out).write_text(json.dumps(log,ensure_ascii=False,indent=2),encoding="utf-8")
        print(json.dumps({k:v for k,v in log.items() if k not in ("samples","baseline","final","after_guard")},ensure_ascii=False),flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--preview",action="store_true",help="Print settings only; default runs real hardware");p.add_argument("--out");p.add_argument("--capture");args=p.parse_args();logdir=Path(args.capture or default_log_directory(__file__));
    if not args.preview:logdir.mkdir(parents=True,exist_ok=True)
    run(not args.preview,args.out or str(logdir/"result.json"),str(logdir))

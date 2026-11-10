"""Example1: all arm/hand states, read-only;500Hz arm,50Hz each split hand field,50Hz stdout.
No motor/hand goals, enable/disable, mode changes, control lease or activation.
"""
from pathlib import Path
import sys,time,threading,json,argparse,math
import xml.etree.ElementTree as ET
from collections import deque
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_sdk import Robot
from robot_rates import StateRates

UDP_HZ=500
PRINT_HZ=50
ASSETS=Path(__file__).resolve().parents[1]/"Assets"

def vector(values,valid=None,digits=3):
 return "["+", ".join(f"{v:.{digits}f}" if valid is None or valid&(1<<i) else "--" for i,v in enumerate(values))+"]"
def rate(v):return "--" if v is None else f"{v:.1f}"
def render(snapshot,rates,gaps):
 a=snapshot.get("ArmState",{});h=snapshot.get("HandState",{});d=snapshot.get("Diagnostics",{})
 ages=h.get("field_age_us",[0xffffffff]*5)
 def hv(key,field,digits=3):
  fresh=bool(h.get("valid_fields",0)&(1<<field)) and ages[field]<500000
  return vector(h.get(key,[]),None if fresh else 0,digits)
 motors=rates.get("can_motor_feedback_hz",[]);fields=rates.get("can_hand_fields_hz",[])
 query={0:"detecting",1:"full-state",2:"split-fields",3:"full-state-explicit-checksum-compatibility",4:"disabled"}.get(d.get("hand_state_query_mode",0),"unknown")
 return "\n".join([
  f"UDP Hz PCtx={rate(rates.get('pc_probe_tx_hz'))} MCUrx={rate(rates.get('board_probe_rx_hz'))} PCrx arm/hand={rate(rates.get('pc_arm_rx_hz'))}/{rate(rates.get('pc_hand_rx_hz'))} boardPub={rate(rates.get('board_arm_publish_hz'))}/{rate(rates.get('board_hand_publish_hz'))} gaps arm/hand={gaps.get('ArmState',0)}/{gaps.get('HandState',0)}",
  f"CAN actual feedback Hz motors1..6={vector(motors,digits=1)} hand[pos,angles,vel,current,fault]={vector(fields,digits=1)} query={query}",
  f"ARM q(rad)={vector(a.get('q_rad',[]),a.get('valid_q_mask',0))} dq(rad/s)={vector(a.get('dq_rad_s',[]),a.get('valid_dq_mask',0))} tau(Nm)={vector(a.get('tau_measured_nm',[]),a.get('valid_tau_mask',0))}",
  f"ARM mode={a.get('motor_mode_raw',[])} faults={a.get('fault_code_raw',[])} feedback_age_us={a.get('feedback_age_us',[])}",
  f"HAND online={h.get('online',False)} q11(rad)={hv('joint_q_rad',1)} motor_position(%)={hv('motor_position_percent',0,1)}",
  f"HAND dq6(rad/s)={hv('motor_dq_rad_s',2)} current6(mA)={hv('motor_current_ma',3,0)} mode={h.get('mode_raw','--')} faults={h.get('fault_code_raw',[])} fault_valid_mask={h.get('fault_valid_mask',0):02x} field_age_us={h.get('field_age_us',[])}",
 ])

def main():
 p=argparse.ArgumentParser();p.add_argument("--seconds",type=float,default=None);p.add_argument("--report");p.add_argument("--trace-can",action="store_true");p.add_argument("--hand-query-hz",type=int,choices=(50,100,200,500,1000,2000),default=200);p.add_argument("--motor-query-hz",type=int,choices=(0,500,1000),default=500);p.add_argument("--hand-angles-only",action="store_true",default=False);p.add_argument("--full-state",action="store_true",default=False);p.add_argument("--split-state",dest="full_state",action="store_false");p.add_argument("--hand-current-fast",action="store_true");p.add_argument("--hand-velocity-fast",action="store_true");p.add_argument("--all-hand-fields",dest="hand_angles_only",action="store_false")
 p.add_argument("--local",default="192.168.10.1",help="PC IPv4 address on the robot network")
 p.add_argument("--board",default="192.168.10.11",help="Board IPv4 address")
 p.add_argument("--board-node",type=int,default=11)
 p.add_argument("--ports",type=int,nargs=2,default=[5005,5006],metavar=("RT","MGMT"))
 args=p.parse_args()
 urdf=ET.parse(ASSETS/"PantheraHT.urdf")
 names=[j.get("name") for j in urdf.findall("joint") if j.get("type")=="revolute"]
 if len(names)!=6:raise RuntimeError("PantheraHT asset must contain exactly six arm joints")
 print("URDF joints:",names,flush=True)
 print("Hand angles-only profile:",args.hand_angles_only,"(when enabled, requested rate applies to all 11 angles; auxiliary fields rotate at total100Hz, about20Hz per field)",flush=True)
 print(f"Requested: each motor {args.motor_query_hz}Hz; hand angle block {args.hand_query_hz}Hz" if args.hand_angles_only else f"Requested: each motor {args.motor_query_hz}Hz; hand total split queries {args.hand_query_hz}Hz ({args.hand_query_hz/4:g}Hz per field)",flush=True)
 r=Robot(local=args.local,board=args.board,board_node=args.board_node,ports=args.ports).start();stop=threading.Event();sent=[0];send_errors=[0];print_count=0;history=deque(maxlen=3600);rates=StateRates();started=time.perf_counter();configured_boot=0
 def sender():
  deadline=time.perf_counter()
  while not stop.is_set():
   if r.board_boot:
    result=r.send_state_probe()
    if result.ok:sent[0]+=1
    else:send_errors[0]+=1
   deadline+=1/UDP_HZ;delay=deadline-time.perf_counter()
   if delay>0:time.sleep(delay)
   else:
    # Do not burst stale probes; this is transport scheduling, not motion limiting.
    if delay<-.01:deadline=time.perf_counter()
 thread=threading.Thread(target=sender,name="state-probe-500Hz",daemon=True);thread.start()
 print(f"URDF: {ASSETS/'PantheraHT.urdf'}\nEncoder coordinates; URDF zero/direction are not calibrated. Ctrl+C closes this monitor; no motor mode is changed.",flush=True)
 deadline=time.perf_counter()
 try:
  while args.seconds is None or time.perf_counter()-started<args.seconds:
   if r.board_boot and configured_boot!=r.board_boot:
    result=r.start_state_stream(trace_can=args.trace_can,hand_query_hz=args.hand_query_hz,motor_query_hz=args.motor_query_hz,hand_angles_only=args.hand_angles_only,hand_current_fast=args.hand_current_fast,hand_velocity_fast=args.hand_velocity_fast,hand_full_state=args.full_state,legacy_full_checksum=args.full_state,timeout=1)
    if result.get("status")==0:configured_boot=r.board_boot
    else:print("State-stream setup retry:",result,flush=True)
   snapshot=r.snapshot()
   with r._lock:counts=dict(r.count);gaps=dict(r.gaps)
   before=rates.value;value=rates.update(time.perf_counter(),counts,sent[0],snapshot)
   if value is not before:history.append(dict(pc_elapsed_s=time.perf_counter()-started,**value))
   print(render(snapshot,value,gaps),flush=True);print_count+=1
   deadline+=1/PRINT_HZ;delay=deadline-time.perf_counter()
   if delay>0:time.sleep(delay)
   elif delay<-.1:deadline=time.perf_counter()
 except KeyboardInterrupt:pass
 finally:
  stop.set();thread.join(2);elapsed=time.perf_counter()-started
  report=dict(requested_motor_hz=args.motor_query_hz,requested_hand_hz=args.hand_query_hz,hand_angles_only=args.hand_angles_only,hand_current_fast=args.hand_current_fast,hand_velocity_fast=args.hand_velocity_fast,elapsed_s=elapsed,probe_sent=sent[0],send_errors=send_errors[0],print_frames=print_count,print_hz=print_count/elapsed,rates=list(history),counts=dict(r.count),gaps=dict(r.gaps),sdk_stats=dict(r.stats),last=r.snapshot())
  report["stream_close_result"]=r.stop_state_stream() if configured_boot else None
  r.close()
  if args.report:Path(args.report).parent.mkdir(parents=True,exist_ok=True);Path(args.report).write_text(json.dumps(report,indent=2),encoding="utf-8")
  print("State monitor closed. No actuator stop/disable/mode command sent.",flush=True)
if __name__=="__main__":main()

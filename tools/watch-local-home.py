"""Observe local homing; sends only read-only peer inspection packets."""
from pathlib import Path
import argparse,json,math,sys,time
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'source/host'))
from dual_peer import PeerMonitor
from local_home import PHASES,ERRORS
p=argparse.ArgumentParser();p.add_argument('--node',type=int,nargs='+',default=[11,12]);p.add_argument('--seconds',type=float,default=40);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
m=PeerMonitor();end=time.monotonic()+a.seconds;previous={};last_print={};rows=[];errors=[]
a.output.parent.mkdir(parents=True,exist_ok=True)
try:
 with a.output.with_suffix('.jsonl').open('w',buffering=1) as f:
  while time.monotonic()<end:
   for n in a.node:
    try:
     s=m.query(n)['local'];s['observed_at']=datetime.now().isoformat();rows.append(s);f.write(json.dumps(s)+'\n')
     status=(s.get('home_state'),s.get('home_generation'),s.get('home_error'),s.get('home_button_seq'),s.get('home_button_status'),s.get('home_button_ack'))
     if status!=previous.get(n) or time.monotonic()-last_print.get(n,0)>5:
      print(n,PHASES.get(status[0],'旧固件'),'generation',status[1],'error',ERRORS.get(status[2]),'q_deg',[round(v*180/math.pi,2) for v in s['q']],'arm_age_ms',round(max(s['arm_age_us'])/1000,1),'hand_age_ms',round(s['hand_age_us'][1]/1000,1),'button(seq/status/ack)',status[3:],flush=True)
      previous[n]=status;last_print[n]=time.monotonic()
    except Exception as e:errors.append({'node':n,'error':str(e)})
   time.sleep(.1)
finally:m.close()
report={'finished_at':datetime.now().isoformat(),'errors':errors,'boards':{}}
for n in a.node:
 r=[s for s in rows if s['node']==n];good=[s for s in r if 'home_state' in s]
 if not good:report['boards'][n]={'verified':False};continue
 last=good[-1];hold=[s for s in good if s['home_state']==5]
 report['boards'][n]={'final':last,'observed_phases':sorted(set(s['home_state'] for s in good)),
   'hold_samples':len(hold),'hold_max_abs_q_deg':max((max(abs(q-z)*180/math.pi for q,z in zip(s['q'],s['home_zero'])) for s in hold),default=None),
   'fault_seen':any(s['home_state']==6 for s in good),'max_can_tx_error':max(s['can_tx_error'] for s in good)}
a.output.write_text(json.dumps(report,indent=2)+'\n');print('Saved',a.output,flush=True)

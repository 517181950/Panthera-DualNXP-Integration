"""Summarize the live run from files, without competing for hardware sockets."""
from pathlib import Path
import csv
from datetime import datetime
import json
import math
import os
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'source/host'))
from dual_peer import summarize
session=json.loads((ROOT/'evidence/current-dual-nxp-session.json').read_text())
folder=Path(session['output'])
with (folder/'raw.csv').open() as f:
    raw=[r for r in csv.DictReader(f) if r.get('phase')=='gello_follow' and r.get('fault6') is not None]
if not raw:raise RuntimeError('No follow records')
start_us=int(raw[0]['mcu_us'])+2000000
samples=[]
for line in folder.with_suffix('.peer.jsonl').read_text().splitlines():
    try:s=json.loads(line)
    except json.JSONDecodeError:continue
    right=s['boards'].get('12',{}).get('local',{})
    if right.get('active')==1 and right.get('hand_control_tx',0)>0 and right.get('time_us',0)>=start_us:samples.append(s)
with (folder/'hand.csv').open() as f:hand=[r for r in csv.DictReader(f) if r.get('actual_rad6') is not None]
with (folder/'gello_targets.csv').open() as f:targets=[r for r in csv.DictReader(f) if r.get('qref6') is not None]
running=Path(f'/proc/{session["pid"]}').exists() and session['status']=='process_running'
report={'captured_at':datetime.now().isoformat(),'session':session,'running':running,
        'peer_and_CAN':summarize(samples),'observer_errors':[s['errors'] for s in samples if s.get('errors')],
        'follow_window_s':float(raw[-1]['t'])-float(raw[0]['t']),
        'first_follow_target_deg':[math.degrees(float(raw[0][f'qref{i}'])) for i in range(1,7)],
        'arm_faults':sorted({int(float(r[f'fault{i}'])) for r in raw for i in range(1,7)}),
        'hand_mode_seen':sorted({int(float(r['mode'])) for r in hand if r['mode']}),
        'hand_angle_age_max_ms':max(float(r['angle_age_ms']) for r in hand),
        'hand_missing_angles':sum(not r[f'actual_rad{i}'] for r in hand for i in range(1,7)),
        'grasp_minmax':[min(float(r['grasp_percent']) for r in targets if r['grasp_percent']),max(float(r['grasp_percent']) for r in targets if r['grasp_percent'])],
        'hand_actual_travel_deg':[math.degrees(max(float(r[f'actual_rad{i}']) for r in hand if r[f'actual_rad{i}'])-min(float(r[f'actual_rad{i}']) for r in hand if r[f'actual_rad{i}'])) for i in range(1,7)],
        'limits':'Only node12 has real arm/hand. State exchange is not synchronized dual-arm motion. Short run does not resolve the recurrent hand nonresponse.'}
(ROOT/'evidence/dual-live-verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='session'},indent=2))

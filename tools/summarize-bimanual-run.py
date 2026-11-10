"""Verify the two-arm run from its recorded targets, CAN counters and feedback."""
from pathlib import Path
import csv
from datetime import datetime
import json
import math
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'source/host'))
from dual_peer import summarize
s=json.loads((ROOT/'evidence/current-bimanual-session.json').read_text());folder=Path(s['output'])
raw={};per_side={};first_us={}
for side in ['left','right']:
    targets={};first=last=None;faults=set();kps=set();kds=set();home=None;qmin=[math.inf]*6;qmax=[-math.inf]*6
    with (folder/side/'raw.csv').open() as f:
        for row in csv.DictReader(f):
            if not row.get('fault6'):continue
            if row['phase']=='home_settle':home=[math.degrees(float(row[f'q{i}'])) for i in range(1,7)]
            if row['phase']!='gello_follow':continue
            if first is None:first=row;first_us[side]=int(row['mcu_us'])
            last=row;targets[row['point']]=tuple(float(row[f'qref{i}']) for i in range(1,7))
            faults.update(int(float(row[f'fault{i}'])) for i in range(1,7))
            kps.add(tuple(float(row[f'kp{i}']) for i in range(1,7)));kds.add(tuple(float(row[f'kd{i}']) for i in range(1,7)))
            for i in range(6):q=float(row[f'q{i+1}']);qmin[i]=min(qmin[i],q);qmax[i]=max(qmax[i],q)
    if first is None:raise RuntimeError(side+' has not entered following')
    raw[side]=targets
    per_side[side]={'window_s':float(last['t'])-float(first['t']),'faults':sorted(faults),'kp_seen':sorted(kps),'kd_seen':sorted(kds),'home_end_deg':home,'travel_deg':[math.degrees(b-a) for a,b in zip(qmin,qmax)]}
common=raw['left'].keys()&raw['right'].keys()
error=max((max(abs(a-b) for a,b in zip(raw['left'][n],raw['right'][n])) for n in common),default=None)
samples=[]
for line in (folder/'peer.jsonl').read_text().splitlines():
    try:r=json.loads(line)
    except json.JSONDecodeError:continue
    if all(r['boards'].get(str(node),{}).get('local',{}).get('time_us',0)>first_us[side]+2000000 and r['boards'][str(node)]['local']['active']==1 for side,node in [('left',11),('right',12)]):samples.append(r)
count=0;first_t=last_t=None
with (folder/'shared_targets.csv').open() as f:
    for r in csv.DictReader(f):
        if r.get('following')=='True' and r.get('right_command_sequence'):
            t=float(r['t']);count+=1
            if first_t is None:first_t=t
            last_t=t
hands={side:{'rows':0,'max_age_ms':0,'modes':set(),'acks':set(),'missing_angles':0} for side in ['left','right']}
with (folder/'hands.csv').open() as f:
    for r in csv.DictReader(f):
        if r.get('actual5') is None:continue
        h=hands[r['side']];h['rows']+=1;h['max_age_ms']=max(h['max_age_ms'],float(r['age_ms']))
        h['modes'].add(int(float(r['mode'])));h['acks'].add(int(float(r['ack_status'])))
        h['missing_angles']+=sum(not r[f'actual{i}'] for i in range(6))
for h in hands.values():h['modes']=sorted(h['modes']);h['acks']=sorted(h['acks'])
report={'recorded_at':datetime.now().isoformat(),'session':s,'running':s['status']=='process_running' and Path(f'/proc/{s["pid"]}').exists(),
        'follow_window_s':last_t-first_t,'host_shared_target_hz':(count-1)/(last_t-first_t),
        'same_cycle_comparisons':len(common),'max_logical_target_difference_rad':error,
        'arms':per_side,'hands':hands,'peer_and_CAN':summarize(samples),
        'peer_observer_errors':[r['errors'] for r in samples if r.get('errors')],
        'encoder_reference':json.loads((folder/'encoder_reference.json').read_text()),
        'user_visual_confirmation':'方向和动作正常',
        'limits':'Sequential host sends of identical logical targets; no hardware synchronization. Host update rate differs from 500Hz CAN refresh. Historical hand nonresponse/power-loss causes unresolved.'}
(ROOT/'evidence/one-gello-two-arms-live.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ['session']},indent=2))

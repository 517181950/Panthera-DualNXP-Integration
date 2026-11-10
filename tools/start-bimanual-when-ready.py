"""Wait read-only for both hands, then run the authorized shared GELLO program."""
from pathlib import Path
import sys,os,time,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'source/host'))
from dual_peer import PeerMonitor
p=PeerMonitor();stable=None;last_print=0
print('等待两块 NXP 和两只手回复；恢复后自动启动双臂双手回零及 GELLO 跟随。',flush=True)
while True:
    status={};ready=True
    for n in (11,12):
        try:
            s=p.query(n)['local'];age=s['hand_age_us'][1]
            status[n]={'hand_angle_age_ms':round(age/1000,1),'home_state':s.get('home_state'),'boot':s['boot']}
            ready=ready and (s['hand_fields']&2)!=0 and age<100000 and s.get('home_state',0) in (0,5)
        except Exception as e:status[n]={'error':str(e)};ready=False
    now=time.monotonic()
    (ROOT/'evidence/bimanual-start-readiness.json').write_text(json.dumps({'status':status,'ready':ready},indent=2)+'\n')
    if now-last_print>=3:print(status,flush=True);last_print=now
    if ready:
        if stable is None:stable=now
        if now-stable>=.5:break
    else:stable=None
    time.sleep(.2)
p.close();print('两只手均已回复，启动原共享 GELLO 程序。',flush=True)
os.execv(sys.executable,[sys.executable,'-u',str(ROOT/'run-one-gello-two-arms.py'),'--execute'])

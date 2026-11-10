"""Supervised, supported-arm bench check. No homing path or hand targets.

Requires explicit --execute. Leaves the tested arm in powered soft hold.
Abnormal motion aborts with drive-off; physical support is mandatory.
"""
import argparse,json,math,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'source/host'))
from robot_sdk import Robot
from dual_peer import PeerMonitor
from arm_dynamics import Gravity

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--side',choices=('left','right'),required=True)
    p.add_argument('--execute',action='store_true')
    a=p.parse_args()
    if not a.execute:
        print('Preview only: explicit soft hold, same-pose host control, 50ms watchdog, observe after disconnect.');return
    config=json.loads((ROOT/'one-gello-two-arms.json').read_text());b=config['boards'][a.side]
    monitor=PeerMonitor(config['local']);r=None;rows=[];operations=[];anchor=None;activated=False
    report={'side':a.side,'status':'running'}
    def state():
        for attempt in range(3):
            try:return monitor.query(b['node'])['local']
            except TimeoutError:
                if attempt==2:raise
                time.sleep(.05)
    def check(s):
        if s['q_mask']!=63 or s['dq_mask']!=63 or s['fault_mask']!=63 or any(s['fault']):raise RuntimeError('Invalid/faulted joints')
        if max(s['arm_age_us'])>=20000:raise RuntimeError('CAN feedback stale')
        if not all(math.isfinite(v) for v in s['q']+s['dq']):raise RuntimeError('Nonfinite feedback')
        if anchor is not None and max(abs(q-z) for q,z in zip(s['q'],anchor))>math.radians(5):raise RuntimeError('Bench displacement exceeded 5 degrees')
        if max(abs(v) for v in s['dq'])>math.radians(60):raise RuntimeError('Bench speed exceeded 60 deg/s')
    def op(n):
        result=r.operation(n);operations.append({'operation':n,'result':result})
        if result['status']!=0:raise RuntimeError(str(result))
        return result
    def observe(stage,seconds,expected=16):
        end=time.perf_counter()+seconds
        while time.perf_counter()<end:
            s=state();rows.append({'stage':stage,'host_time':time.perf_counter(),'state':s});check(s)
            if s['home_state']!=expected:raise RuntimeError(f'Unexpected phase {s["home_state"]}')
            time.sleep(.05)
    try:
        before=state();check(before);anchor=before['q'].copy();report['before']=before
        if before['home_state']!=15 or before['active']:raise RuntimeError('Must start in drive-off standby')
        if max(abs(v) for v in before['dq'])>math.radians(3):raise RuntimeError('Arm must be stationary and supported')
        r=Robot(local=config['local'],board=b['ip'],board_node=b['node'],ports=b['ports']).start()
        if not r.discover(3):raise RuntimeError('No board telemetry')
        op(19)
        stream=r.start_state_stream(motor_query_hz=500,hand_query_hz=50,hand_angles_only=True)
        if stream.get('status')!=0:raise RuntimeError(str(stream))
        report['stream']=stream
        op(1);activated=True;op(18)
        print(a.side,'explicit soft hold enabled; observing at current pose',flush=True)
        time.sleep(.05);observe('explicit_soft_hold',5)
        s=state();check(s)
        if max(abs(v) for v in s['dq'])>math.radians(3):raise RuntimeError('Not stationary before watchdog test')
        op(1)
        q=s['q'];model=Gravity(identified=str(ROOT/b['identified']))
        ff=model(q);kp=[4,10,10,2,2,1];kd=[1.2,1.92,1.92,.8313843876,.48,.48]
        result=r.send_arm(q,[0]*6,kp,kd,ff)
        if not result.ok:raise RuntimeError(result.error)
        time.sleep(.005);op(3)
        end=time.perf_counter()+.15
        while time.perf_counter()<end:
            result=r.send_arm(q,[0]*6,kp,kd,ff)
            if not result.ok:raise RuntimeError(result.error)
            time.sleep(.002)
        last=time.perf_counter();end=last+1
        while True:
            s=state();check(s);rows.append({'stage':'watchdog_wait','host_time':time.perf_counter(),'state':s})
            if s['home_state']==16:break
            if s['home_state']!=0:raise RuntimeError(f'Watchdog test entered phase {s["home_state"]}, reason {s["home_error"]}')
            if time.perf_counter()>=end:raise RuntimeError('Watchdog did not take over')
            time.sleep(.01)
        if s['home_error']!=11:raise RuntimeError('Wrong watchdog reason')
        report['watchdog_observed_ms']=(time.perf_counter()-last)*1000
        print(a.side,'watchdog took over; disconnecting PC control',flush=True)
        r.close();r=None
        observe('after_pc_disconnect',3)
        report['status']='passed';report['after']=state()
        report['max_displacement_deg']=[max(abs(row['state']['q'][j]-anchor[j])*180/math.pi for row in rows) for j in range(6)]
        print(json.dumps({k:v for k,v in report.items() if k not in ('before','after')},ensure_ascii=False),flush=True)
    except BaseException as exc:
        report['status']='failed';report['error']=repr(exc)
        if activated and r is not None:
            # Explicit supervised bench abort only; never used by teleop cleanup.
            try:report['bench_abort']=r.operation(17)
            except Exception as stop_error:report['bench_abort_error']=repr(stop_error)
        raise
    finally:
        if r is not None:r.close()
        monitor.close();report['operations']=operations;report['samples']=rows
        (ROOT/f'evidence/soft-stop-bench-{a.side}.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()

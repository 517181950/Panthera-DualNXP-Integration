"""Dual NXP inspection, or explicit GELLO control of one calibrated local side."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import runpy
import sys
import threading
import traceback

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'source/host'))
from dual_peer import PeerMonitor


class Tee:
    def __init__(self, terminal, log): self.terminal,self.log=terminal,log
    def write(self,text): self.terminal.write(text);self.log.write(text);return len(text)
    def flush(self): self.terminal.flush();self.log.flush()


def arguments(config, side, output):
    board=config['boards'][side]
    if not board['actuators_connected'] or not board['gello_config'] or board['group'] is None:
        raise ValueError(f'{side} has no connected/calibrated arm + hand + GELLO')
    script=ROOT/'source/host/examples/18_gello-low-impedance.py'
    return [str(script),'--execute','--home-zero','--group',str(board['group']),
            '--local',config['local'],'--board',board['ip'],'--board-node',str(board['node']),
            '--ports',*map(str,board['ports']),'--port',board['gello_port'],
            '--gello-config',str(ROOT/board['gello_config']),
            '--kp',*map(str,config['kp']),'--kd',*map(str,config['kd']),
            '--hz',str(config['arm_hz']),'--require-hand','--log','--seconds','0','--output',str(output)]


def inspect(config):
    monitor=PeerMonitor(config['local'])
    try:
        result={str(b['node']):monitor.query(b['node']) for b in config['boards'].values()}
        for node,pair in result.items():
            state=pair['local']
            if not pair['remote'] or state['peer_age_us']>100000:
                raise RuntimeError(f'Node {node}: peer state is missing/stale')
        return result
    finally:monitor.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true',help='Home and start the selected arm + hand')
    p.add_argument('--side',choices=['left','right'],default='right')
    args=p.parse_args();config=json.loads((ROOT/'dual-nxp.json').read_text())
    status=inspect(config)
    if not args.execute:
        print(json.dumps(status,indent=2),flush=True);return
    print('两块 NXP 已连接，板间状态新鲜。',flush=True)
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output=ROOT/'logs'/('gello_dual_nxp_'+stamp);console=output.with_suffix('.console.log')
    argv=arguments(config,args.side,output)
    # Bind exclusivity is also enforced by the existing SDK sockets.
    record=dict(pid=os.getpid(),started=datetime.now().isoformat(),status='starting',
                side=args.side,output=str(output),console=str(console),argv=argv)
    paths=[ROOT/'evidence/current-dual-nxp-session.json',ROOT/'evidence/current-single-arm-session.json']
    def save():
        for path in paths:path.write_text(json.dumps(record,indent=2)+'\n')
    save();stop=threading.Event();exit_code=0
    stdout,stderr=sys.stdout,sys.stderr
    with console.open('w',buffering=1) as log:
        sys.stdout,sys.stderr=Tee(stdout,log),Tee(stderr,log)
        def watch():
            monitor=PeerMonitor(config['local']);last_print=0
            try:
                with output.with_suffix('.peer.jsonl').open('w',buffering=1) as peerlog:
                    stop.wait(.02)  # Separate startup inspection from the first observer query.
                    while not stop.is_set():
                        sample={'time':datetime.now().isoformat(),'boards':{}}
                        for node in (11,12):
                            try:sample['boards'][str(node)]=monitor.query(node)
                            except Exception as exc:sample.setdefault('errors',[]).append(f'{node}: {exc}')
                        peerlog.write(json.dumps(sample)+'\n')
                        import time
                        now=time.monotonic()
                        if now-last_print>5:
                            ages={n:round(v['local']['peer_age_us']/1000,1) for n,v in sample['boards'].items()}
                            print(f'DUAL NXP peer age(ms)={ages} errors={sample.get("errors",[])}',flush=True)
                            last_print=now
                        stop.wait(.2)
            finally:monitor.close()
        thread=threading.Thread(target=watch,name='dual-peer-observer',daemon=True)
        try:
            print(f'双 NXP 臂手固件；本次运行 {args.side} / node{config["boards"][args.side]["node"]}，各侧使用独立配置。',flush=True)
            print('先回机械臂零位，再检查 GELLO 保存的固定零点；FOLLOW GELLO 后开始跟随。',flush=True)
            print('Ctrl+C 结束并尝试保持当前姿态；不是断电或急停。',flush=True)
            record['status']='process_running';save();thread.start()
            sys.path.insert(0,str(Path(argv[0]).parent));sys.argv=argv
            runpy.run_path(argv[0],run_name='__main__')
        except KeyboardInterrupt:exit_code=130
        except SystemExit as exc:exit_code=exc.code if isinstance(exc.code,int) else 1
        except BaseException:exit_code=1;traceback.print_exc()
        finally:
            stop.set()
            if thread.is_alive():thread.join(2)
            record.update(status='exited',exit_code=exit_code,ended=datetime.now().isoformat());save()
            sys.stdout.flush();sys.stderr.flush();sys.stdout,sys.stderr=stdout,stderr
    raise SystemExit(exit_code)

if __name__=='__main__':main()

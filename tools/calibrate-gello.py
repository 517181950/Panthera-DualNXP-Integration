"""Capture one GELLO's saved zero/trigger endpoints; reads inputs only, never moves arms."""
import argparse
from datetime import datetime
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'source/host'),str(ROOT/'source/host/examples')]
from 运行参数 import DUAL_GELLO_CONFIG
from gello_inputs import make_reader
spec=importlib.util.spec_from_file_location('calibration_teleop',ROOT/'source/host/examples/18_gello-low-impedance.py')
teleop=importlib.util.module_from_spec(spec);spec.loader.exec_module(teleop)


def stable_ticks(samples,span_limit=8):
    values=np.asarray(samples,dtype=float)
    if values.ndim!=2 or values.shape[1]!=7 or len(values)<10 or not np.isfinite(values).all():
        raise ValueError('需要至少10组新鲜的七节点数据')
    # Readers can start with different whole-turn offsets; retain a continuous
    # branch during capture, then save the six joint zeros modulo one turn.
    values=values+4096*np.rint((values[0]-values)/4096)
    if np.any(np.ptp(values,axis=0)>span_limit):raise ValueError('GELLO未保持静止，请固定姿态后重试')
    return np.rint(np.median(values,axis=0)).astype(int).tolist()


def apply_capture(profile,group,stage,ticks,signs=None):
    data=profile.setdefault(group,{})
    if stage=='zero':
        if signs is None or len(signs)!=6 or any(x not in (-1,1) for x in signs):
            raise ValueError('采零位必须明确提供六轴 --signs（标定方向，非根据零位猜测）')
        data['zero_raw_ticks']=[v%4096 for v in ticks[:6]]
        data['axes']=[dict(sign=s,scale=1.0,offset_rad=0.0) for s in signs]
    else:
        capture=data.setdefault('_trigger_capture',{})
        capture[stage]=ticks[6]%4096
        if 'open' in capture and 'closed' in capture:
            opened=capture['open'];delta=(capture['closed']-opened+2048)%4096-2048
            if abs(delta)<20 or abs(delta)==2048:raise ValueError('扳机端点无法区分或行程不明确，请重新采集')
            closed=opened+delta
            data['gripper']=dict(min_raw=min(opened,closed),max_raw=max(opened,closed),
                                 inverted=delta>0,output_min=0.0,output_max=0.04)
    return profile


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--side',choices=['left','right'],required=True)
    p.add_argument('--stage',choices=['zero','open','closed'],required=True)
    p.add_argument('--signs',type=int,nargs=6,choices=[-1,1],help='配置轴方向乘子；需人工确认每轴正向')
    a=p.parse_args()
    if a.stage=='zero' and a.signs is None:p.error('zero阶段必须提供 --signs 六轴方向，不能沿用另一GELLO的标定')
    source=DUAL_GELLO_CONFIG['gellos'][a.side];path=ROOT/source['config']
    profile=json.loads(path.read_text()) if path.exists() else {'serial':{'baud':1000000}}
    group=source.get('profile_group',('left','right')[source['group']])
    reader=make_reader(source,profile,DUAL_GELLO_CONFIG['local'],teleop)
    reader.set_gripper_calibration(source['group'],0,4095)
    print(f'{a.side}: 仅读取GELLO；保持'+{'zero':'主手在六轴参考零位','open':'扳机完全松开','closed':'扳机完全按下'}[a.stage]+'，3秒后采样。',flush=True)
    samples=[];last=None
    try:
        reader.start();time.sleep(3);end=time.perf_counter()+1
        while time.perf_counter()<end:
            snap=reader.latest(source['group']);now=time.perf_counter()
            if (reader.is_connected and snap is not None and snap.data_valid
                and all(t is not None and 0<=now-t<.15 for t in snap.servo_captured_at)):
                stamp=tuple(snap.servo_captured_at)
                if stamp!=last:samples.append(snap.unwrapped_ticks);last=stamp
            time.sleep(.01)
    finally:reader.stop()
    ticks=stable_ticks(samples);apply_capture(profile,group,a.stage,ticks,a.signs)
    profile.setdefault('_captures',{})[a.stage]=dict(at=datetime.now().isoformat(),source=source,ticks=ticks)
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():
        backup=path.with_name(path.name+'.'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.bak')
        backup.write_bytes(path.read_bytes())
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n');temporary.replace(path)
    print(f'已保存 {path} 的 {a.stage}。六轴方向需现场逐轴确认；zero/open/closed三项齐备后才能用于遥操。')

if __name__=='__main__':main()

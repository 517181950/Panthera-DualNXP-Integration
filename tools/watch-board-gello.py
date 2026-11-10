"""Inspect either board GELLO raw inputs only; no arm/hand commands or ownership changes."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'source/host'),str(ROOT/'source/host/examples')]
from board_gello import BoardGelloReader

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=10);p.add_argument('--output',type=Path)
    p.add_argument('--side',choices=['left','right'],default='right')
    p.add_argument('--group-mask',type=int,choices=[1,2,3],default=1,help='1: IDs1-7; 2: IDs11-17; 3: both groups (dual firmware required)')
    a=p.parse_args()
    if not 0<a.seconds<=3600:p.error('seconds must be in (0,3600]')
    reader=BoardGelloReader(board='192.168.10.'+('11' if a.side=='left' else '12'),node=11 if a.side=='left' else 12,group_mask=a.group_mask)
    # Raw diagnostics do not require calibration for the new input.
    for group in (0,1):
        if a.group_mask&(1<<group):reader.set_gripper_calibration(group,0,4095)
    reader.start()
    records=[];start=time.perf_counter()
    try:
        while time.perf_counter()-start<a.seconds:
            now=time.perf_counter()
            row=dict(t=now-start,connected=reader.is_connected,error=reader.error,diagnostics=reader.diagnostics,groups={})
            for group_id in (0,1):
                group=reader.latest(group_id)
                if group:
                    row['groups'][str(group_id)]=dict(ids=group.servo_ids,valid=group.data_valid,
                        raw=group.raw_positions,ticks=group.unwrapped_ticks,
                        age_ms=[None if stamp is None else round((now-stamp)*1000,2) for stamp in group.servo_captured_at])
            records.append(row);print(json.dumps(row,ensure_ascii=False),flush=True);time.sleep(.5)
    finally:
        reader.stop()
        if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(records,indent=2)+'\n')

if __name__=='__main__':main()

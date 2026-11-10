"""Compatibility entry point and inactive communication benchmark."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from demo_paths import default_log_directory
import argparse,json,time
from pathlib import Path
from robot_protocol import *
from robot_sdk import Robot as SDKRobot

class Robot(SDKRobot):
 def benchmark(self,hz,seconds):
  warm=time.perf_counter()+.35
  while time.perf_counter()<warm:self.receive(.01)
  self.receive();start_count=self.count.copy();start_gaps=self.gaps.copy();start_diag=self.last.get('Diagnostics',{}).copy();sent=0;start=time.perf_counter();deadline=start;period=1/hz
  echoes=[]; sent_at={}; confirmed=set(); board_stamps=[]; arrival_stamps=[]
  while time.perf_counter()-start<seconds:
   now=time.perf_counter()
   if now>=deadline:
    packet=self.arm_packet();sent_at[self.sequence]=time.perf_counter();self.socks[0].sendto(packet,(self.board,5005));sent+=1;deadline=start+sent*period
   for name,d in self.receive():
    if name=='ArmState':
     board_stamps.append(d['header']['tx_monotonic_us']);arrival_stamps.append(time.perf_counter())
     ack=d['last_applied_command_sequence']
     if ack in sent_at and ack not in confirmed:echoes.append((time.perf_counter()-sent_at[ack])*1000);confirmed.add(ack)
   remaining=deadline-time.perf_counter()
   if remaining>.0005:time.sleep(.0001)
  elapsed=time.perf_counter()-start; measured_count=self.count.copy()
  end=time.perf_counter()+.2
  while time.perf_counter()<end:self.receive(.005)
  diag=self.last.get('Diagnostics',{});rates={k:round((measured_count[k]-start_count[k])/elapsed,2) for k in ('ArmState','HandState','FaultSnapshot','Diagnostics')}
  result=dict(target_hz=hz,sent=sent,elapsed_s=round(elapsed,4),actual_send_hz=round(sent/elapsed,2),telemetry_hz=rates,arm_state_missing_sequence=self.gaps['ArmState']-start_gaps['ArmState'],board_accepted_delta=diag.get('udp_rx_valid',0)-start_diag.get('udp_rx_valid',0),board_bad=diag.get('udp_rx_bad'),board_old=diag.get('udp_rx_old'),board_tx_drop=diag.get('udp_tx_drop'),control_period_max_us=diag.get('control_period_max_us'),control_jitter_max_us=diag.get('control_jitter_abs_max_us'),arm_online_mask=self.last.get('ArmState',{}).get('online_mask'),hand_online=self.last.get('HandState',{}).get('online'))
  result['board_bad_delta']=diag.get('udp_rx_bad',0)-start_diag.get('udp_rx_bad',0)
  result['board_old_delta']=diag.get('udp_rx_old',0)-start_diag.get('udp_rx_old',0)
  result['board_tx_drop_delta']=diag.get('udp_tx_drop',0)-start_diag.get('udp_tx_drop',0)
  intervals=sorted(b-a for a,b in zip(board_stamps,board_stamps[1:]))
  result['board_state_period_us']={str(q):intervals[min(len(intervals)-1,int((len(intervals)-1)*q/100))] for q in (0,50,95,99,100)} if intervals else {}
  result['host_state_span_hz']=round((len(arrival_stamps)-1)/(arrival_stamps[-1]-arrival_stamps[0]),4) if len(arrival_stamps)>1 else None
  result['board_state_span_hz']=round((len(board_stamps)-1)*1e6/(board_stamps[-1]-board_stamps[0]),4) if len(board_stamps)>1 else None
  ordered=sorted(echoes)
  result['command_to_state_ms']={str(q):round(ordered[min(len(ordered)-1,int((len(ordered)-1)*q/100))],3) for q in (50,95,99,100)} if ordered else {}
  print(json.dumps(result,ensure_ascii=False),flush=True);return result
def main():
 p=argparse.ArgumentParser();p.add_argument('--local',default='192.168.10.1');p.add_argument('--board',default='192.168.10.11');p.add_argument('--seconds',type=float,default=5);p.add_argument('--rates',default='100,500,1000,2000');p.add_argument('--out',default=str(Path(default_log_directory(__file__))/'benchmark.json'));a=p.parse_args()
 r=Robot(a.local,a.board);
 if not r.discover():
  print('Board discovery timed out');r.close();return
 print('BOARD boot',hex(r.board_boot),'CLAIM',r.operation(1),flush=True)
 results=[r.benchmark(float(hz),a.seconds) for hz in a.rates.split(',')]
 r.operation(2);r.close()
 if a.out:Path(a.out).parent.mkdir(parents=True,exist_ok=True);Path(a.out).write_text(json.dumps(dict(results=results,can_enabled=True,physical_device_test=False),indent=2),encoding='utf-8')
if __name__=='__main__':main()


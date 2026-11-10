"""Loss-accounted raw UDP recorder + complete decoded application messages."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from demo_paths import default_log_directory
import json,socket,threading,time,struct
from pathlib import Path
class Capture:
 def __init__(self,path):
  self.path=Path(path);self.path.mkdir(parents=True,exist_ok=True)
  self.lock=threading.Lock();self.raw=(self.path/"udp_packets.bin").open("wb",buffering=1024*1024)
  self.decoded=(self.path/"messages.jsonl").open("w",encoding="utf-8",buffering=1024*1024)
  self.raw.write(b"RUDP1");self.raw_part=0;self.stop=threading.Event();self.trace_count=0;self.trace_callback=None;self.ready=threading.Event()
  self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
  self.sock.bind(("192.168.10.1",5007));self.sock.settimeout(.1)
  self.thread=threading.Thread(target=self.trace,daemon=True);self.thread.start()
 def record(self,direction,port,data):
  with self.lock:
   if self.raw.tell()>128*1024*1024:
    self.raw.close();self.raw_part+=1;self.raw=(self.path/f"udp_packets_{self.raw_part:04d}.bin").open("wb",buffering=1024*1024);self.raw.write(b"RUDP1")
   self.raw.write(struct.pack("<QBHH",time.perf_counter_ns(),direction,port,len(data))+data)
 def trace(self):
  while not self.stop.is_set():
   try:
    b,a=self.sock.recvfrom(2048)
    if a[0]=="192.168.10.11":
     self.record(1,5007,b);self.trace_count+=1;self.ready.set()
     if self.trace_callback:self.trace_callback(b)
   except socket.timeout:pass
 def attach(self,r):
  r.packet_recorder=self.record;original=r.receive
  def receive(timeout=0):
   packets=original(timeout)
   for name,data in packets:self.decoded.write(json.dumps(dict(pc_ns=time.perf_counter_ns(),name=name,value=data),separators=(",",":"))+"\n")
   return packets
  r.receive=receive
 def close(self):
  self.stop.set();self.thread.join();self.sock.close();self.raw.close();self.decoded.close()

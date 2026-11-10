"""Observed rates from counters, rather than requested configuration values."""
class StateRates:
 def __init__(self):self.previous=None;self.value={}
 def update(self,pc_time,counts,sent,snapshot):
  diag=snapshot.get("Diagnostics",{});current=(pc_time,dict(counts),sent,diag,snapshot.get("ArmState",{}),snapshot.get("HandState",{}))
  if self.previous is None:self.previous=current;return self.value
  old=self.previous;dt=pc_time-old[0]
  if dt<1:return self.value
  r={"pc_probe_tx_hz":(sent-old[2])/dt,"pc_arm_rx_hz":(counts.get("ArmState",0)-old[1].get("ArmState",0))/dt,"pc_hand_rx_hz":(counts.get("HandState",0)-old[1].get("HandState",0))/dt}
  mdt=(diag.get("header",{}).get("tx_monotonic_us",0)-old[3].get("header",{}).get("tx_monotonic_us",0))/1e6
  if mdt>0:
   for key in ("hand_control_tx","hand_control_ack","hand_full_query_tx","hand_query_timeouts","hand_control_negative_ack"):
    if key in diag and key in old[3]:r[key+"_hz"]=((diag[key]-old[3][key])&0xffffffff)/mdt
   if "motor_control_tx" in diag and "motor_control_tx" in old[3]:r["can_motor_control_hz"]=[((x-y)&0xffffffff)/mdt for x,y in zip(diag["motor_control_tx"],old[3]["motor_control_tx"])]
   r["board_probe_rx_hz"]=((diag.get("state_probe_received",0)-old[3].get("state_probe_received",0))&0xffffffff)/mdt
   for name,key in (("can_motor_feedback_hz","motor_feedback_count"),("can_hand_fields_hz","hand_field_feedback_count")):
    a=diag.get(key);b=old[3].get(key)
    if a is not None and b is not None:r[name]=[((x-y)&0xffffffff)/mdt for x,y in zip(a,b)]
  for name,new,past in (("board_arm_publish_hz",current[4],old[4]),("board_hand_publish_hz",current[5],old[5])):
   nh=new.get("header",{});oh=past.get("header",{});span=(nh.get("tx_monotonic_us",0)-oh.get("tx_monotonic_us",0))/1e6
   if span>0 and nh.get("source_boot_id")==oh.get("source_boot_id"):r[name]=((nh.get("sequence",0)-oh.get("sequence",0))&0xffffffff)/span
  self.previous=current;self.value=r;return r

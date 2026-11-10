"""Optional low-current feedback transport behind the proven legacy arm loop."""
from types import SimpleNamespace
import time
import numpy as np
import GELLO力反馈参数 as defaults
from tools.hd1910_haptic import HapticBus


class LegacyHapticReader:
    def __init__(self,joints,include_hand=False):
        self.config=SimpleNamespace(**{k:getattr(defaults,k) for k in dir(defaults) if k.isupper()})
        # Same 150ms acquisition horizon as the existing read-only GELLO path.
        self.config.SERVO_MAX_AGE_S=.15
        self.config.COMMAND_TIMEOUT_S=.15
        self.include_hand=include_hand
        if include_hand:self.config.FEEDBACK_IDS=list(range(1,8))
        self.master=HapticBus(self.config,joints)
        self.active=False;self.sample=None

    @property
    def is_connected(self):return self.master.b is not None and self.master.error is None

    def start(self):self.master.open();self.master.prepare()

    def activate(self):
        self.master.enable();sample=self.master.read_all()
        self.master.publish(sample['ticks'][:len(self.master.ids)],source_ticks=sample['ticks'][:len(self.master.ids)]);self.master.start();self.active=True

    def latest(self,group):
        if group!=0:return None
        if self.master.error is not None:raise RuntimeError('主手反馈输出停止：'+str(self.master.error))
        try:self.sample=self.master.snapshot() if self.active else self.master.read_all()
        except RuntimeError:return None
        s=self.sample
        return SimpleNamespace(raw_positions=tuple(s['ticks']),unwrapped_ticks=tuple(s['ticks']),
            servo_captured_at=(s['oldest'],)*7,raw_velocities=(None,)*7,arm_radians=(None,)*6)

    def feedback(self,requested,actual,factor,following,hand_percent=None,trigger=None):
        if not self.active or self.sample is None:return
        source=np.array(self.sample['ticks'][:6],dtype=float)
        if following:
            offset=self.config.FEEDBACK_STRENGTH*(np.asarray(actual)-np.asarray(requested))/np.asarray(factor)
            offset=np.clip(offset,-np.deg2rad(self.config.FEEDBACK_OFFSET_DEG),np.deg2rad(self.config.FEEDBACK_OFFSET_DEG))
            goal=source+offset/(2*np.pi/4096)
        else:goal=source
        goal=np.rint(goal).astype(int)
        for i in range(6):
            j=self.master.joints[i+1];goal[i]=int(np.clip(goal[i],j['min_ticks'],j['max_ticks']))
        if self.include_hand:
            raw7=self.sample['ticks'][6];ref7=raw7
            if hand_percent is not None and trigger is not None:
                cap=self.config.FEEDBACK_OFFSET_DEG*4096/360
                ref7=raw7+np.clip(self.config.FEEDBACK_STRENGTH*(trigger.ticks(hand_percent)-raw7),-cap,cap)
            j=self.master.joints[7];ref7=int(np.clip(round(ref7),j['min_ticks'],j['max_ticks']))
            goal=np.append(goal,ref7);source=np.append(source,raw7)
        self.master.publish(goal,source_ticks=source)

    def stop(self):self.master.close()

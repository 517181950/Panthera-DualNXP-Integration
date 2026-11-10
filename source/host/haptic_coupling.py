"""Bounded bilateral position coupling; not an external-wrench estimator."""
from dataclasses import dataclass
import numpy as np

TICK_RAD = 2*np.pi/4096


def vector(x, name):
    a=np.asarray(x,dtype=float)
    if a.shape!=(6,) or not np.all(np.isfinite(a)):
        raise ValueError(name+' requires six finite values')
    return a


@dataclass
class Coupling:
    signs: list
    lower_rad: list
    upper_rad: list
    master_min: list
    master_max: list
    speed_deg_s: float = 5.0
    strength: float = .2
    reflection_deg: float = 3.0
    tracking_deg: float = 10.0
    max_dt: float = .05

    def __post_init__(self):
        self.signs=vector(self.signs,'signs')
        if not np.all(np.isin(self.signs,[-1,1])):raise ValueError('Confirm every axis direction first')
        for key in ('lower_rad','upper_rad','master_min','master_max'):
            setattr(self,key,vector(getattr(self,key),key))
        if np.any(self.lower_rad>=self.upper_rad) or np.any(self.master_min>=self.master_max):raise ValueError('invalid range')
        if not 0<self.speed_deg_s<=10 or not 0<self.strength<=.3 or not 0<self.reflection_deg<=5:
            raise ValueError('invalid first-stage coupling limits')
        self.anchor=None

    def align(self, master, arm, now):
        master=vector(master,'master');arm=vector(arm,'arm')
        self.check_master(master)
        if np.any(arm<self.lower_rad-np.deg2rad(.5)) or np.any(arm>self.upper_rad+np.deg2rad(.5)):raise ValueError('Arm outside soft range')
        self.anchor=(master.copy(),arm.copy());self.target=arm.copy();self.now=now

    def check_master(self, master):
        if np.any(master<self.master_min) or np.any(master>self.master_max):raise RuntimeError('GELLO超出人工记录行程')

    def update(self, master, arm, now):
        if self.anchor is None:raise RuntimeError('Not aligned')
        master=vector(master,'master');arm=vector(arm,'arm');self.check_master(master)
        dt=now-self.now
        if not np.isfinite(dt) or not 0<dt<=self.max_dt:raise RuntimeError('控制周期超时')
        if np.any(np.abs(arm-self.target)>np.deg2rad(self.tracking_deg)):raise RuntimeError('机械臂跟随误差过大')
        if np.any(arm<self.lower_rad-np.deg2rad(2)) or np.any(arm>self.upper_rad+np.deg2rad(2)):
            raise RuntimeError('机械臂超出软限位容差')
        m0,q0=self.anchor
        requested=q0+self.signs*(master-m0)*TICK_RAD
        bounded=np.clip(requested,self.lower_rad,self.upper_rad)
        step=np.deg2rad(self.speed_deg_s)*dt
        self.target+=np.clip(bounded-self.target,-step,step)
        # Use requested (not clipped) target: joint limits also reflect resistance.
        error=(arm-requested)/self.signs
        offset=np.clip(self.strength*error,-np.deg2rad(self.reflection_deg),np.deg2rad(self.reflection_deg))
        reflected=np.clip(master+offset/TICK_RAD,self.master_min,self.master_max)
        self.now=now
        return self.target.copy(),np.rint(reflected).astype(int),requested

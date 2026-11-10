"""Coordinated upright waypoint and bounded cosine-acceleration homing path."""
import math
import numpy as np
UPRIGHT=np.array([0.,math.pi/2,math.pi/2+math.atan2(.23,.06),math.pi/2-math.atan2(.23,.06),0.,0.])
SPEED=math.radians(30);ACCEL=math.radians(30)
def start_ok(start):
    """Accept any finite six-axis starting pose; do not gate on model limits."""
    q=np.asarray(start,dtype=float)
    return bool(q.shape==(6,) and np.all(np.isfinite(q)))
def reference(raw):
    if not start_ok(raw):raise ValueError('Six finite joint positions required')
    return 2*math.pi*np.floor((np.asarray(raw)-UPRIGHT)/(2*math.pi)+.5)
def stages(origin):
    origin=np.asarray(origin,dtype=float);zero=reference(origin)
    aligned=UPRIGHT+zero;raised=aligned.copy();raised[0]=origin[0]
    return [('raise',origin.copy(),raised),('align',raised.copy(),aligned),('lower',aligned.copy(),zero)]
def timing(start,end):
    distance=float(np.max(np.abs(np.asarray(end)-start)))
    ramp=math.pi*SPEED/(2*ACCEL)
    speed=SPEED
    if distance<speed*ramp:
        ramp=max(.3,math.sqrt(math.pi*distance/(2*ACCEL)));speed=distance/ramp
    duration=distance/speed+ramp if speed>0 else .6
    return distance,ramp,speed,duration

def trajectory(t,start,end):
    start,end=np.asarray(start),np.asarray(end);distance,ramp,speed,duration=timing(start,end)
    if distance<1e-9:return end.copy(),np.zeros(6)
    flat=distance/speed-ramp;t=max(0.,t)
    if t<ramp:
        x=speed/2*(t-ramp/math.pi*math.sin(math.pi*t/ramp));v=speed/2*(1-math.cos(math.pi*t/ramp))
    elif t<ramp+flat:x=speed*(t-ramp/2);v=speed
    elif t<duration:
        u=t-ramp-flat;x=speed*(ramp/2+flat)+speed/2*(u+ramp/math.pi*math.sin(math.pi*u/ramp));v=speed/2*(1+math.cos(math.pi*u/ramp))
    else:x=distance;v=0.
    delta=(end-start)/distance
    return start+x*delta,v*delta

class Still:
    """20ms velocity smoothing plus <=0.5deg measured position span for 0.5s."""
    def __init__(self):self.velocity=None;self.updated=None;self.since=None
    def update(self,q,dq,now,position_ok=True):
        q,dq=np.asarray(q),np.asarray(dq)
        if self.velocity is None:self.velocity=dq.copy();self.updated=now
        dt=max(0,now-self.updated);self.updated=now
        self.velocity+=dt/(.02+dt)*(dq-self.velocity)
        if not position_ok or np.max(np.abs(self.velocity))>math.radians(3):self.since=None;return False
        if self.since is None:self.since=now;self.low=q.copy();self.high=q.copy()
        self.low=np.minimum(self.low,q);self.high=np.maximum(self.high,q)
        if np.max(self.high-self.low)>math.radians(.5):self.since=now;self.low=q.copy();self.high=q.copy()
        return now-self.since>=.5

class Tracking:
    """Single >5deg check before the next target; no extra persistence timer."""
    def update(self,actual,target,now):
        error=float(np.max(np.abs(np.asarray(actual)-target)))
        return math.isfinite(error) and error<=math.radians(5)

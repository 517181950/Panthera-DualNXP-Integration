"""ID7 grasp mapping and position-based hand progress estimation."""
import math
import numpy as np


class TriggerMapping:
    def __init__(self,open_ticks,closed_ticks):
        if not all(math.isfinite(v) for v in (open_ticks,closed_ticks)) or not 20<abs(closed_ticks-open_ticks)<2048:
            raise ValueError('扳机端点需要已确认的短行程')
        self.open=float(open_ticks);self.closed=float(closed_ticks)
    def percent(self,ticks):
        if ticks is None or not math.isfinite(ticks):return None
        p=float(np.clip((ticks-self.open)/(self.closed-self.open)*100,0,100))
        return 0. if p<2 else 100. if p>98 else p
    def ticks(self,percent):
        if not math.isfinite(percent):raise ValueError('开合比例无效')
        return self.open+(self.closed-self.open)*np.clip(percent,0,100)/100


class HandProgress:
    """Precomputed closest-point projection; inexpensive in the control loop."""
    def __init__(self,tuning):
        self.knots=np.linspace(0,100,101)
        points=np.array([tuning.target(float(p)) for p in self.knots])
        self.a=points[:-1];self.d=np.diff(points,axis=0)
        self.norm=np.sum(self.d*self.d,axis=1)
    def __call__(self,actual):
        actual=np.asarray(actual,dtype=float)
        if actual.shape!=(6,) or not np.all(np.isfinite(actual)):raise ValueError('手指角度无效')
        u=np.clip(np.sum((actual-self.a)*self.d,axis=1)/np.maximum(self.norm,1e-12),0,1)
        errors=np.sum((actual-(self.a+u[:,None]*self.d))**2,axis=1)
        i=int(np.argmin(errors))
        return float(self.knots[i]+u[i]*(self.knots[i+1]-self.knots[i]))


def hand_progress(actual,tuning):return HandProgress(tuning)(actual)

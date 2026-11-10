"""Position-command admittance using a quasi-static current residual.

URDF convention: M*qdd+C+g = tau_motor+tau_external.
At low speed tau_external ~= g - tau_motor. Friction/inertia remain errors.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
import pinocchio as pin
TICK_RAD=2*np.pi/4096

def vector(x,name):
    a=np.asarray(x,dtype=float)
    if a.shape!=(6,) or not np.all(np.isfinite(a)):raise ValueError(name+': six finite values required')
    return a

class Model:
    def __init__(self,path,gravity):
        self.sha=hashlib.sha256(Path(path).read_bytes()).hexdigest()
        self.model=pin.buildModelFromUrdf(str(path));self.data=self.model.createData()
        if self.model.nq!=6 or self.model.nv!=6 or list(self.model.names)[1:]!=['joint'+str(n) for n in range(1,7)]:
            raise ValueError('URDF must contain joint1..joint6 in order')
        g=np.asarray(gravity,float)
        if g.shape!=(3,) or not np.all(np.isfinite(g)) or not 9<np.linalg.norm(g)<10:raise ValueError('Invalid gravity')
        self.model.gravity.linear=g
        self.gravity_vector=g
    def gravity(self,q):return pin.computeGeneralizedGravity(self.model,self.data,vector(q,'q')).copy()
    def potential(self,q):return float(pin.computePotentialEnergy(self.model,self.data,vector(q,'q')))

class Mapping:
    def __init__(self,doc,model):
        if doc.get('urdf_sha256')!=model.sha or doc.get('ids')!=list(range(1,7)) or doc.get('verified')!=[True]*6:
            raise ValueError('URDF零位/方向映射尚未验证')
        if doc.get('gravity_verified') is not True or not np.allclose(doc.get('gravity_base',[]),model.gravity_vector):
            raise ValueError('URDF底座重力方向尚未确认')
        self.zero=vector(doc['zero_ticks'],'zero_ticks');self.sign=vector(doc['sign'],'sign')
        if not np.all(np.isin(self.sign,[-1,1])):raise ValueError('Invalid axis sign')
        self.lower=model.model.lowerPositionLimit.copy();self.upper=model.model.upperPositionLimit.copy()
        self.identity=hashlib.sha256(json.dumps(doc,sort_keys=True).encode()).hexdigest()
    def q(self,ticks):return self.sign*(vector(ticks,'ticks')-self.zero)*TICK_RAD
    def ticks(self,q):return np.rint(self.zero+self.sign*vector(q,'q')/TICK_RAD).astype(int)
    def bounds(self,q):
        q=vector(q,'q')
        if np.any(q<self.lower-.03) or np.any(q>self.upper+.03):raise ValueError('反馈坐标超出URDF范围，检查零位/整圈计数')
        return q

class TorqueCalibration:
    def __init__(self,doc,model,mapping):
        if doc.get('usable') is not True or doc.get('urdf_sha256')!=model.sha or doc.get('mapping_identity')!=mapping.identity:
            raise ValueError('缺少与当前URDF/零位一致的电流力矩标定')
        self.k=vector(doc['nm_per_amp'],'nm_per_amp');self.bias=vector(doc['bias_amp'],'bias_amp')
        self.active=np.asarray(doc['active_axes'],bool)
        if self.active.shape!=(6,) or np.any(self.active & (abs(self.k)<.01)):raise ValueError('Invalid current conversion')
    def torque(self,current_ma):return self.k*(vector(current_ma,'current_ma')/1000-self.bias)

class Admittance:
    def __init__(self,model,mapping,calibration,settings,q,current_ma):
        self.model=model;self.mapping=mapping;self.cal=calibration
        self.mass=vector(settings.VIRTUAL_MASS,'mass');self.damping=vector(settings.DAMPING,'damping')
        self.deadband=vector(settings.TORQUE_DEADBAND_NM,'deadband')
        self.vmax=np.deg2rad(vector(settings.MAX_SPEED_DEG_S,'speed'))
        self.amax=np.deg2rad(vector(settings.MAX_ACCEL_DEG_S2,'acceleration'))
        self.offset=np.deg2rad(vector(settings.MAX_TARGET_OFFSET_DEG,'offset'))
        self.filter_seconds=float(settings.CURRENT_FILTER_SECONDS)
        if np.any(self.mass<=0) or np.any(self.damping<=0) or np.any(self.deadband<0) or np.any(self.vmax<=0) or np.any(self.amax<=0) or np.any(self.offset<=0) or not self.filter_seconds>0:
            raise ValueError('Invalid admittance settings')
        self.target=mapping.bounds(q).copy();self.velocity=np.zeros(6);self.filtered=vector(current_ma,'current').copy()
    def update(self,q,current_ma,dt):
        if not np.isfinite(dt) or not 0<dt<=.15:raise ValueError('导纳采样间隔无效或过期')
        q=self.mapping.bounds(q)
        self.filtered+=(1-np.exp(-dt/self.filter_seconds))*(vector(current_ma,'current')-self.filtered)
        gravity=self.model.gravity(q);motor=self.cal.torque(self.filtered);residual=gravity-motor
        tau=np.sign(residual)*np.maximum(abs(residual)-self.deadband,0.)
        tau=np.where(self.cal.active,tau,0.)
        v=(self.velocity+dt*tau/self.mass)/(1+dt*self.damping/self.mass)
        v=np.clip(v,self.velocity-self.amax*dt,self.velocity+self.amax*dt)
        v=np.clip(v,-self.vmax,self.vmax)
        proposed=self.target+dt*v
        low=np.maximum(self.mapping.lower,q-self.offset);high=np.minimum(self.mapping.upper,q+self.offset)
        target=np.clip(proposed,low,high)
        # No hidden integrated velocity at a target/physical limit.
        v=np.where(abs(target-proposed)>1e-12,0.,v)
        target=np.where(self.cal.active,target,self.target)
        self.target=target;self.velocity=v
        return target.copy(),{'q':q.tolist(),'g_nm':gravity.tolist(),'motor_nm':motor.tolist(),
            'residual_nm':residual.tolist(),'velocity':v.tolist(),'target':target.tolist()}

def fit_mapping(samples,model,gravity_verified):
    if len(samples)<2:raise ValueError('需要两个或更多已知URDF姿态')
    if any(x['urdf_sha256']!=model.sha or x['modes']!=[0]*6 for x in samples):raise ValueError('模型或坐标模式不一致')
    ticks=np.array([vector(x['ticks'],'ticks') for x in samples]);q=np.array([vector(x['q_rad'],'q') for x in samples])
    zero=[];sign=[];verified=[];errors=[]
    for i in range(6):
        candidates=[]
        for s in (-1,1):
            z=np.mean(ticks[:,i]-s*q[:,i]/TICK_RAD)
            err=np.max(abs(s*(ticks[:,i]-z)*TICK_RAD-q[:,i]));candidates.append((err,z,s))
        err,z,s=min(candidates);zero.append(z);sign.append(s);errors.append(float(err))
        verified.append(bool(np.ptp(q[:,i])>=np.deg2rad(8) and err<=np.deg2rad(1.5)))
    return {'urdf_sha256':model.sha,'ids':list(range(1,7)),'zero_ticks':zero,'sign':sign,'verified':verified,
            'max_error_rad':errors,'gravity_base':model.gravity_vector.tolist(),'gravity_verified':bool(gravity_verified)}

def fit_current(samples,model,mapping):
    if len(samples)<10:raise ValueError('至少需要10组不同静态姿态样本')
    if any(x.get('saturated_axes') or x.get('usable') is False for x in samples):raise ValueError('饱和或已判定无效的样本不能用于力矩辨识')
    if any(x['mapping_identity']!=mapping.identity or x.get('no_external_force') is not True for x in samples):
        raise ValueError('静态标定样本映射不一致或有外力')
    qs=np.array([vector(x['q_rad'],'q') for x in samples]);amps=np.array([vector(x['current_ma'],'current')/1000 for x in samples])
    gravity=np.array([model.gravity(q) for q in qs]);ks=[];biases=[];active=[];quality=[]
    for i in range(6):
        if np.ptp(gravity[:,i])<.04:
            ks.append(0.);biases.append(0.);active.append(False);quality.append({'reason':'重力激励不足，该轴固定保持'});continue
        if np.ptp(amps[:,i])<.08:raise ValueError(f'J{i+1}电流变化不足以辨识')
        k,b=np.linalg.lstsq(np.c_[amps[:,i],np.ones(len(samples))],gravity[:,i],rcond=None)[0]
        pred=k*amps[:,i]+b;rmse=float(np.sqrt(np.mean((pred-gravity[:,i])**2)))
        r2=float(1-np.sum((pred-gravity[:,i])**2)/np.sum((gravity[:,i]-gravity[:,i].mean())**2))
        if not .01<abs(k)<5 or rmse>.04 or r2<.8:raise ValueError(f'J{i+1}力矩辨识不可信：R²={r2:.3f} RMSE={rmse:.3f}Nm')
        ks.append(float(k));biases.append(float(-b/k));active.append(True);quality.append({'rmse_nm':rmse,'r2':r2})
    if not any(active):raise ValueError('没有关节获得足够重力激励')
    return {'usable':True,'urdf_sha256':model.sha,'mapping_identity':mapping.identity,'nm_per_amp':ks,'bias_amp':biases,'active_axes':active,'quality':quality}

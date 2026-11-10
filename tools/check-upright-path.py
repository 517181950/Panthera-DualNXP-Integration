"""Compiled MCU path vs Python, acceleration/speed and model upright geometry."""
from pathlib import Path
import ctypes as C,sys,json,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'source/host'))
import home_path as path
from arm_dynamics import Gravity
import pinocchio as pin
A=C.c_float*6
class Plan(C.Structure):_fields_=[('start',A),('goal',A),('distance',C.c_float),('ramp',C.c_float),('speed',C.c_float),('duration',C.c_float)]
lib=C.CDLL(str(ROOT/'evidence/home-path.so'));lib.home_path_begin.argtypes=[C.POINTER(Plan),A,C.c_uint];lib.home_path_sample.argtypes=[C.POINTER(Plan),C.c_float,A,A]
lib.home_path_reference.argtypes=[A,A];lib.home_path_reference.restype=C.c_bool
model=Gravity().model;d=model.createData();rng=np.random.default_rng(947);maxerr=0;maxspeed=0;maxacc=0;count=0
for origin in [np.zeros(6),np.deg2rad([-22,53,95,0,-12,5]),*rng.uniform(-8*np.pi,8*np.pi,(100,6))]:
 zero=path.reference(origin);cz=A();assert lib.home_path_reference(A(*origin),cz)
 assert np.max(np.abs(np.array(cz)-zero))<1e-5
 assert np.max(np.abs(origin-(path.UPRIGHT+zero)))<=np.pi+1e-6
 for stage,(_,begin,end) in enumerate(path.stages(origin)):
  p=Plan();lib.home_path_begin(C.byref(p),A(*(begin-zero)),stage)
  assert np.max(np.abs(np.array(p.goal)+zero-end))<1e-6
  dist,ramp,speed,dur=path.timing(begin,end)
  assert abs(p.duration-dur)<1e-5
  ts=np.linspace(0,dur,501);lastv=None
  for t in ts:
   q,v=path.trajectory(t,begin,end);cq,cv=A(),A();lib.home_path_sample(C.byref(p),float(t),cq,cv)
   maxerr=max(maxerr,np.max(np.abs(q-(np.array(cq)+zero))),np.max(np.abs(v-np.array(cv))));count+=1
   maxspeed=max(maxspeed,np.max(np.abs(v)))
   if lastv is not None:maxacc=max(maxacc,np.max(np.abs(v-lastv))/(ts[1]-ts[0]))
   lastv=v
assert maxerr<1e-5 and maxspeed<=path.SPEED+1e-9 and maxacc<=path.ACCEL+1e-5
pin.forwardKinematics(model,d,path.UPRIGHT)
upper=d.oMi[3].translation-d.oMi[2].translation;lower=d.oMi[4].translation-d.oMi[3].translation;axis=d.oMi[6].rotation[:,0]
for vector in (upper,lower,axis):assert np.linalg.norm(vector[:2])<1e-8 and vector[2]>0
report={'samples':count,'C_Python_max_error':float(maxerr),'max_speed_deg_s':math.degrees(maxspeed),'max_accel_deg_s2':math.degrees(maxacc),'upright_deg':np.rad2deg(path.UPRIGHT).tolist(),'wrist_height_m':float(d.oMi[6].translation[2]),'tool_axis':axis.tolist(),'geometry':'joint centres and local tool +X only; no environment/hand-mesh collision certification'}
(ROOT/'evidence/upright-path-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
def fk(q):
 pin.forwardKinematics(model,d,q);pts=np.array([x.translation.copy() for x in d.oMi[1:]]);return pts,pts[-1]+.15*d.oMi[6].rotation[:,0]
def xy(v):return (430+v[0]*650,650-v[2]*650)
def line(points,color,width=4,dash=''):
 return '<polyline points="'+' '.join('%.2f,%.2f'%xy(v) for v in points)+'" fill="none" stroke="'+color+'" stroke-width="'+str(width)+'" '+('stroke-dasharray="'+dash+'"' if dash else '')+'/>'
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="760" viewBox="0 0 900 760">','<rect width="900" height="760" fill="white"/>','<g font-family="sans-serif" fill="#182536">','<text x="45" y="45" font-size="25">Upright waypoint → encoder zero</text>','<text x="45" y="78" font-size="17">J2 90° · J3 165.4° · J4 14.6° · J5/J6 0°</text>',line([np.array([-.45,0,0]),np.array([.5,0,0])],'#cad2dc',2)]
trail=[]
for t in np.linspace(0,path.timing(np.zeros(6),path.UPRIGHT)[3],201):
 q,_=path.trajectory(t,np.zeros(6),path.UPRIGHT);pts,tip=fk(q);trail.append(tip)
svg.append(line(trail,'#70b6a0',2,'5 4'))
for q,color,label in [(np.zeros(6),'#9ba7b6','Zero'),(path.UPRIGHT,'#2464c4','Upright')]:
 pts,tip=fk(q);svg.append(line(pts,color,7));svg.append(line([pts[-1],tip],color,3,'6 4'))
 for v in pts:
  x,y=xy(v);svg.append(f'<circle cx="{x}" cy="{y}" r="6" fill="white" stroke="{color}" stroke-width="3"/>')
 x,y=xy(tip);svg.append(f'<text x="{x+15}" y="{y}" font-size="18" fill="{color}">{label}</text>')
svg+=['<text x="45" y="695" font-size="16">Dashed tool extension = 150 mm illustration, not the measured hand envelope.</text>','<text x="45" y="722" font-size="16">Side view; joint-centre geometry only. Physical clearance must be checked.</text>','</g></svg>']
(ROOT/'evidence/upright-path-preview.svg').write_text('\n'.join(svg))

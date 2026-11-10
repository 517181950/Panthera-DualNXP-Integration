"""Render the supplied CAD geometry in two known URDF poses, for manual mapping."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'source/host')]
import numpy as np
import pinocchio as pin
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import GELLO导纳参数 as cfg
from gello_admittance import Model
m=Model(cfg.URDF,cfg.GRAVITY_BASE)
source=Path(json.loads((cfg.URDF.parent/'source.json').read_text())['source']);meshes=source.parent.parent/'meshes'
out=ROOT/'evidence/gello-admittance';out.mkdir(parents=True,exist_ok=True)
colors=['#6c7886','#4a90d9','#d86934','#55a86e','#ba79c7','#c5a143','#56aeba']
fig=plt.figure(figsize=(15,9));poses=[np.zeros(6),np.deg2rad([10,-10,10,10,10,10])]
for row,q in enumerate(poses):
 pin.forwardKinematics(m.model,m.data,q);pin.updateFramePlacements(m.model,m.data)
 surfaces=[]
 for i,name in enumerate(['base_link']+['link'+str(n) for n in range(1,7)]):
  mesh=trimesh.load(meshes/(name+'.STL'),force='mesh').simplify_quadric_decimation(face_count=2500);T=m.data.oMf[m.model.getFrameId(name)]
  vertices=mesh.vertices@T.rotation.T+T.translation
  faces=mesh.faces[::max(1,len(mesh.faces)//2500)]
  surfaces.append((vertices[faces],colors[i]))
 for col,(elev,azim,title) in enumerate([(22,-60,'Perspective'),(0,-90,'XZ view'),(0,0,'YZ view')]):
  ax=fig.add_subplot(2,3,row*3+col+1,projection='3d')
  for faces,color in surfaces:ax.add_collection3d(Poly3DCollection(faces,facecolor=color,edgecolor='none',alpha=1.,rasterized=True))
  points=np.concatenate([x[0].reshape(-1,3) for x in surfaces]);center=(points.min(0)+points.max(0))/2;extent=max(np.ptp(points,axis=0))*.6
  for setter,c in zip([ax.set_xlim,ax.set_ylim,ax.set_zlim],center):setter(c-extent,c+extent)
  for n in range(1,7):
   p=m.data.oMi[n].translation;ax.scatter(*p,color='black',s=12);ax.text(*p,' J'+str(n),fontsize=8)
  ax.set_box_aspect((1,1,1));ax.view_init(elev=elev,azim=azim);ax.set_xlabel('base X (m)');ax.set_ylabel('base Y (m)');ax.set_zlabel('base Z (m)');ax.set_title(('A: ' if row==0 else 'B: ')+title)
  if col==1:ax.set_yticks([]);ax.set_ylabel('')
  if col==2:ax.set_xticks([]);ax.set_xlabel('')
fig.suptitle('GELLO URDF reference poses (degrees)\nA = [0, 0, 0, 0, 0, 0]     B = [10, -10, 10, 10, 10, 10]\nGravity assumed along base -Z; geometry simplified for display',y=1.,fontsize=14)
fig.tight_layout(rect=[0,0,1,.90]);fig.savefig(out/'urdf-reference-poses.png',dpi=140,bbox_inches='tight');print(out/'urdf-reference-poses.png')

"""Derive model-coordinate sign conversion from aligned joint axes.
Does not move hardware. Preserve conflicting manual evidence as unverified.
"""
from pathlib import Path
import sys,json,hashlib,xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
ROOT=Path(__file__).resolve().parents[1]

def axes(path):
    model=pin.buildModelFromUrdf(str(path));data=model.createData();pin.forwardKinematics(model,data,np.zeros(model.nq))
    elements={j.attrib['name']:j for j in ET.parse(path).getroot().findall('joint')}
    out=[];points=[]
    for n in range(1,7):
        name='joint'+str(n);j=elements[name];idx=model.getJointId(name)
        axis=np.fromstring(j.find('axis').attrib['xyz'],sep=' ')
        out.append(data.oMi[idx].rotation@axis);points.append(data.oMi[idx].translation.copy())
    return np.array(out),np.array(points)

def derive():
    arm=ROOT/'source/host/Assets/PantheraHT_v4_hand620g.urdf';gello=ROOT/'source/models/gello_hd1910/gello_hd1910.urdf'
    old=json.loads((ROOT/'gello-haptic-directions.json').read_text())
    if old['side']!='right' or old['verified']!=[True]*6:raise ValueError('Legacy directions not verified')
    a,pa=axes(arm);g,pg=axes(gello)
    # Align the shoulder-to-elbow horizontal direction while preserving base +Z.
    va=pa[2]-pa[1];vg=pg[2]-pg[1]
    yaw=np.arctan2(vg[1],vg[0])-np.arctan2(va[1],va[0]);c,s=np.cos(yaw),np.sin(yaw)
    R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    dots=np.sum((a@R.T)*g,axis=1)
    if np.any(abs(dots)<.999):raise ValueError('Model axes do not align; cannot reuse')
    conversion=np.sign(dots).astype(int);derived=np.array(old['signs'])*conversion
    return {'source':'verified legacy teleop + URDF joint axis alignment',
            'assumption':'corresponding upright bases; shoulder-to-elbow directions aligned',
            'arm_urdf_sha256':hashlib.sha256(arm.read_bytes()).hexdigest(),
            'gello_urdf_sha256':hashlib.sha256(gello.read_bytes()).hexdigest(),
            'base_yaw_deg':float(np.rad2deg(yaw)),'aligned_axis_dot':dots.tolist(),
            'arm_to_gello_sign':conversion.tolist(),'legacy_encoder_to_arm_sign':old['signs'],
            'derived_encoder_to_gello_sign':derived.tolist()}

def run():
    evidence=derive();path=ROOT/'gello-urdf-mapping.json';doc=json.loads(path.read_text())
    if doc['urdf_sha256']!=evidence['gello_urdf_sha256']:raise ValueError('URDF identity mismatch')
    backup=ROOT/'evidence/gello-admittance/mapping-before-legacy-reuse.json'
    if not backup.exists():backup.write_text(json.dumps(doc,ensure_ascii=False,indent=2))
    conflict=[];sources={}
    for i,sign in enumerate(evidence['derived_encoder_to_gello_sign']):
        manual=doc.get('direction_evidence_by_axis',{}).get(str(i+1))
        if manual:
            observed=int(np.sign(manual['delta_ticks'])*manual['requested_urdf_direction'])
            if observed!=sign:conflict.append(i+1)
        doc['sign'][i]=sign;doc['verified'][i]=(i+1 not in conflict)
        sources[str(i+1)]='legacy+model inference; conflict unresolved' if i+1 in conflict else 'legacy+model inference'
    evidence['conflicts']=conflict;doc['legacy_reuse']=evidence;doc['direction_source_by_axis']=sources
    path.write_text(json.dumps(doc,ensure_ascii=False,indent=2))
    (ROOT/'evidence/gello-admittance/legacy-direction-reconciliation.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2))
    print(json.dumps(evidence,ensure_ascii=False,indent=2));print('verified',doc['verified'])

if __name__=='__main__':run()

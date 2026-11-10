"""Compare link6-only and all-link STATIC gravity fits, priority mass 90%.

Weights are dataset-level, and priority poses are balanced independently of
accepted repeat count. No dynamic inertia identification or hardware commands.
Only J2-J6 torques enter fitting and evaluation; all six angles define posture.
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
import arm_runtime
import numpy as np
from arm_dynamics import read_json,write_json
from arm_identification import model_from_metadata

FIT_JOINTS = np.arange(1, 6)


def load_points(path):
    rows=[p for p in read_json(Path(path)/'points.json') if p['accepted']]
    if not rows:raise ValueError('No accepted poses in '+str(path))
    return rows,np.array([r['q'] for r in rows]),np.array([r['tau'] for r in rows])


def group_weights(groups,share):
    groups=np.asarray(groups);unique=np.unique(groups)
    return np.array([share/len(unique)/np.sum(groups==g) for g in groups])


def solve(A,tau,prior,weights,scope):
    active=np.arange(20,24) if scope=='link6' else np.arange(24)
    scale=np.tile([1.,.1,.1,.1],6)[active]
    x=A[:,FIT_JOINTS,:].reshape(-1,24);y=tau[:,FIT_JOINTS].ravel()
    w=np.repeat(np.sqrt(weights),len(FIT_JOINTS))
    design=x[:,active]*scale*w[:,None]
    delta,_,rank,singular=np.linalg.lstsq(design,(y-x@prior)*w,rcond=1e-8)
    theta=prior.copy();theta[active]+=scale*delta
    return theta,int(rank),singular


def metrics(A,tau,theta,weights,nold,groups):
    err=(A@theta-tau)[:,FIT_JOINTS];newerr=err[nold:]
    return dict(joint_ids=(FIT_JOINTS+1).tolist(),
        weighted_rmse_nm=float(np.sqrt(np.sum(weights[:,None]*err**2)/len(FIT_JOINTS))),
        original_rmse_nm=float(np.sqrt(np.mean(err[:nold]**2))),
        priority_rmse_nm=float(np.sqrt(np.mean(newerr**2))),
        original_joint_rmse_nm=np.sqrt(np.mean(err[:nold]**2,axis=0)).tolist(),
        priority_joint_rmse_nm=np.sqrt(np.mean(newerr**2,axis=0)).tolist(),
        per_priority_pose=[dict(group=int(g)+1,repeat_count=int(np.sum(groups==g)),
             mean_residual_nm=newerr[groups==g].mean(axis=0).tolist(),
             rmse_nm=float(np.sqrt(np.mean(newerr[groups==g]**2)))) for g in np.unique(groups)])


def export_urdf(model,theta,path):
    root=ET.parse(model.path).getroot()
    if model.identity['payload_mass']!=0:return False
    for j in range(6):
        m=theta[4*j];com=theta[4*j+1:4*j+4]/m
        element=root.find(f"link[@name='link{j+1}']/inertial")
        if element is None:return False
        element.find('mass').set('value',str(m));element.find('origin').set('xyz',' '.join(map(str,com)))
    root.insert(0,ET.Comment('Weighted static gravity equivalent only. Rotational inertias retained uncalibrated; individual all-link masses and COMs may be nonunique.'))
    for mesh in root.iter('mesh'):
        p=Path(mesh.get('filename'))
        if not p.is_absolute():mesh.set('filename',(model.path.parent/p).resolve().as_posix())
    ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return True


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('priority_dataset');p.add_argument('--original',default=str(Path(__file__).parent/'examples/logs/gravity285_highgain_001'))
    p.add_argument('--priority-share',type=float,default=.9);p.add_argument('--output',required=True)
    args=p.parse_args()
    if not np.isfinite(args.priority_share) or not 0<args.priority_share<1:p.error('priority-share must be between 0 and 1')
    for label,directory in [('priority',args.priority_dataset),('original',args.original)]:
        folder=Path(directory).resolve()
        missing=[name for name in ('points.json','metadata.json','plan.json') if not (folder/name).is_file()]
        if missing:
            hint=('Run 11_priority-pose-capture.py --execute --output <dataset> first, or pass its actual saved directory.'
                  if label=='priority' else 'Pass the completed original capture directory with --original.')
            p.error(f'{label} dataset missing {missing} in {folder}. {hint}')
    old,oq,ot=load_points(args.original);new,nq,nt=load_points(args.priority_dataset)
    model=model_from_metadata(args.priority_dataset);original=model_from_metadata(args.original)
    if model.identity!=original.identity:raise ValueError('URDF/mapping/payload/gravity identity mismatch')
    meta=read_json(Path(args.priority_dataset)/'metadata.json')
    if meta['arguments'].get('feature')!='gravity_capture':raise ValueError('Expected a gravity capture')
    plan=np.array(read_json(Path(args.priority_dataset)/'plan.json')['poses_deg'])
    _,all_groups=np.unique(plan,axis=0,return_inverse=True)
    groups=np.array([all_groups[r['point']] for r in new])
    if len(np.unique(groups))!=len(np.unique(all_groups)):raise ValueError('At least one target pose has no accepted measurements')
    if any(np.sum(groups==g)<2 for g in np.unique(groups)):raise ValueError('Need at least two accepted repeats per target pose for repeat validation')
    nold=len(old);A=np.array([model.regressor(q) for q in np.vstack([oq,nq])]);tau=np.vstack([ot,nt])
    weights=np.r_[np.full(nold,(1-args.priority_share)/nold),group_weights(groups,args.priority_share)]
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    baseline=metrics(A,tau,model.theta,weights,nold,groups);results={}
    for scope in ('link6','all'):
        theta,rank,singular=solve(A,tau,model.theta,weights,scope)
        evaluation=metrics(A,tau,theta,weights,nold,groups)
        # One repeat per pose held out at a time; evaluates repeatability at the
        # same target poses, NOT generalization to unseen target poses.
        predictions=np.zeros_like(nt);folds=max(np.sum(groups==g) for g in np.unique(groups))
        order=np.zeros(len(new),dtype=int)
        for g in np.unique(groups):order[groups==g]=np.arange(np.sum(groups==g))
        for fold in range(folds):
            test=order==fold;train_new=~test
            keep=np.r_[np.arange(nold),nold+np.flatnonzero(train_new)]
            w=np.r_[np.full(nold,(1-args.priority_share)/nold),group_weights(groups[train_new],args.priority_share)]
            trial,_,_=solve(A[keep],tau[keep],model.theta,w,scope)
            predictions[test]=A[nold:][test]@trial
        evaluation['held_repeat_rmse_nm']=float(np.sqrt(np.mean((predictions[:,FIT_JOINTS]-nt[:,FIT_JOINTS])**2)))
        issues=[]
        if np.any(theta[::4]<=0):issues.append('Nonpositive equivalent link mass; inspect model/measurement residuals')
        if scope=='link6' and rank<4:issues.append('Link6 parameters not fully observable')
        changes=[]
        for j in range(6):
            m=float(theta[4*j]);initial=float(model.theta[4*j]);com=theta[4*j+1:4*j+4]/m*1000 if m>0 else None
            priorcom=model.theta[4*j+1:4*j+4]/initial*1000
            changes.append(dict(link=j+1,mass_kg=m,mass_change_g=(m-initial)*1000,
                                com_mm=com.tolist() if com is not None else None,
                                com_change_mm=(com-priorcom).tolist() if com is not None else None))
        doc=dict(format='panthera-gravity-v1',model_identity=model.identity,theta=theta.tolist(),
            fit_scope=scope,fit_joint_ids=(FIT_JOINTS+1).tolist(),rank=rank,parameters=4 if scope=='link6' else 24,
            fully_observable=rank==(4 if scope=='link6' else 24),usable=not issues,quality_issues=issues,
            weighting=dict(priority_total=args.priority_share,original_total=1-args.priority_share,
                           original_points=nold,priority_repeats=len(new),distinct_priority_poses=len(np.unique(groups))),
            evaluation=evaluation,link_changes=changes,
            interpretation='Static gravity fit. All-link solution is normalized minimum-change from the selected prior; generally nonunique. Dynamic rotational inertia NOT identified. Held-repeat metric is not unseen-pose validation.')
        write_json(out/(scope+'_identified.json'),doc)
        if not issues:doc['urdf_exported']=export_urdf(model,theta,out/(scope+'_identified.urdf'))
        results[scope]=doc
    comparison=dict(baseline=baseline,models=results,
        priority_poses_deg=np.unique(plan,axis=0).tolist(),
        caution='Compare both priority and original pose errors. Do not select by training error alone. Only three new unique poses; full-arm mass/COM parameters remain equivalent and may not represent physical masses.')
    write_json(out/'comparison.json',comparison)
    print('Baseline',baseline)
    for scope,result in results.items():print(scope,'usable=',result['usable'],'metrics=',result['evaluation'],'link6=',result['link_changes'][5])
    print('Saved',out.resolve())
    return 0


if __name__=='__main__':raise SystemExit(main())

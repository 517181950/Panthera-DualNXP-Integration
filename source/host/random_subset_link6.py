"""Repeat 50-pose link6-only fits; compare on the same full pose set.

Changing holdout composition and choosing the best trial are not an independent
generalization test. Full-data OLS is the lower bound for the full-data objective.
"""
import argparse
import csv
from pathlib import Path
import arm_runtime
import numpy as np
from arm_dynamics import read_json,write_json,fit_gravity
from arm_identification import model_from_metadata


def main():
    p=argparse.ArgumentParser();p.add_argument('dataset');p.add_argument('--output',required=True)
    p.add_argument('--trials',type=int,default=1000);p.add_argument('--size',type=int,default=50);p.add_argument('--seed',type=int,default=947)
    args=p.parse_args();root=Path(args.dataset);out=Path(args.output)
    points=[r for r in read_json(root/'points.json') if r['accepted']]
    if not 10<=args.size<len(points) or args.trials<1:p.error('Need 10 <= subset size < accepted poses, trials >= 1')
    out.mkdir(parents=True,exist_ok=False)
    model=model_from_metadata(root);q=np.array([r['q'] for r in points]);tau=np.array([r['tau'] for r in points])
    A=np.array([model.regressor(x) for x in q]);B=A[:,:,20:];target=tau-A[:,:,:20]@model.theta[:20]
    scale=np.array([1.,.1,.1,.1]);ids=np.arange(len(points));rng=np.random.default_rng(args.seed)
    reference=fit_gravity(q,tau,model,'link6');ref=np.array(reference['theta'][20:])
    def metrics(theta,train):
        err=B@theta-target;test=np.setdiff1d(ids,train)
        return dict(train_rmse_nm=float(np.sqrt(np.mean(err[train]**2))),
                    remaining_rmse_nm=float(np.sqrt(np.mean(err[test]**2))) if len(test) else None,
                    all_rmse_nm=float(np.sqrt(np.mean(err**2))),all_joint_rmse_nm=np.sqrt(np.mean(err**2,axis=0)).tolist())
    def describe(theta):
        com=1000*theta[1:]/theta[0];initial=1000*model.theta[21:24]/model.theta[20];refcom=1000*ref[1:]/ref[0]
        return dict(mass_g=float(theta[0]*1000),com_mm=com.tolist(),
                    delta_from_initial=dict(mass_g=float((theta[0]-model.theta[20])*1000),com_mm=(com-initial).tolist(),com_distance_mm=float(np.linalg.norm(com-initial))),
                    delta_from_full_fit=dict(mass_g=float((theta[0]-ref[0])*1000),com_mm=(com-refcom).tolist()))
    trials=[];selected=[]
    for trial in range(args.trials):
        train=np.sort(rng.choice(ids,args.size,replace=False));x=B[train].reshape(-1,4)*scale
        result,_,rank,_=np.linalg.lstsq(x,target[train].ravel(),rcond=1e-8);theta=scale*result
        trials.append(dict(trial=trial+1,rank=int(rank),valid=bool(rank==4 and theta[0]>0),theta=theta.tolist(),
                           **metrics(theta,train),**describe(theta)))
        selected.append(train)
    valid=[r for r in trials if r['valid']]
    if not valid:raise RuntimeError('No full-rank positive-mass trials')
    best_all=min(valid,key=lambda r:r['all_rmse_nm']);best_train=min(valid,key=lambda r:r['train_rmse_nm'])
    best_remaining=min(valid,key=lambda r:r['remaining_rmse_nm'])
    for label,row in [('best_all',best_all),('best_train',best_train),('best_remaining',best_remaining)]:
        train=selected[row['trial']-1]
        document=fit_gravity(q[train],tau[train],model,'link6')
        np.testing.assert_allclose(document['theta'][20:],row['theta'],atol=1e-10)
        np.testing.assert_array_equal(document['theta'][:20],model.theta[:20])
        document['random_trial']=dict(seed=args.seed,trial=row['trial'],selected_point_numbers=[points[i]['point']+1 for i in train],evaluation=row)
        write_json(out/(label+'_link6.json'),document)
    values=np.array([[r['mass_g'],*r['com_mm']] for r in valid])
    summary=dict(seed=args.seed,trials=args.trials,sample_size=args.size,total_accepted=len(points),valid_trials=len(valid),
        full_fit=dict(**metrics(ref,ids),**describe(ref)),best_all=best_all,best_train=best_train,best_remaining=best_remaining,
        distribution=dict(columns=['mass_g','com_x_mm','com_y_mm','com_z_mm'],minimum=values.min(axis=0).tolist(),
            maximum=values.max(axis=0).tolist(),percentiles_5_50_95=np.percentile(values,[5,50,95],axis=0).tolist()),
        note='Rank candidates by all_rmse_nm on the same 284 poses. This is in-dataset selection, not independent validation. The best training/remaining errors use different pose sets and are not directly comparable between trials. Full-data OLS is the minimum possible full-data SSE for the same fixed upstream model.')
    if any(r['all_rmse_nm']<summary['full_fit']['all_rmse_nm']-1e-10 for r in valid):raise AssertionError('OLS lower bound violated')
    write_json(out/'summary.json',summary)
    with (out/'trials.csv').open('w',newline='') as f:
        fields=['trial','rank','valid','train_rmse_nm','remaining_rmse_nm','all_rmse_nm','mass_g','com_x_mm','com_y_mm','com_z_mm','point_numbers']
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for row,train in zip(trials,selected):
            record={k:row[k] for k in fields if k in row}
            record.update(zip(['com_x_mm','com_y_mm','com_z_mm'],row['com_mm']))
            record['point_numbers']=' '.join(str(points[i]['point']+1) for i in train);writer.writerow(record)
    print(__import__('json').dumps(summary,indent=2))


if __name__=='__main__':main()

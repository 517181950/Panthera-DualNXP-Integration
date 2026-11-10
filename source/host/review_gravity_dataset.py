"""Offline audit of a gravity capture, with independent raw-data statistics."""
import argparse
import csv
from pathlib import Path
import arm_runtime
import numpy as np
from arm_dynamics import read_json,write_json,fit_gravity
from arm_identification import model_from_metadata


def main():
    p=argparse.ArgumentParser();p.add_argument('dataset');p.add_argument('--output',required=True)
    a=p.parse_args();root=Path(a.dataset);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    model=model_from_metadata(root);points=read_json(root/'points.json')
    buckets={};count=0;stale=0;faults=0;max_age=0;seq_duplicates=0;last_seq=None
    with (root/'raw.csv').open(encoding='utf-8') as f:
        for row in csv.DictReader(f):
            count+=1;stale+=int(row['fresh'])!=1
            seq=row['state_sequence'];seq_duplicates+=seq==last_seq;last_seq=seq
            faults+=any(float(row[f'fault{i}'])!=0 for i in range(1,7))
            max_age=max(max_age,max(float(row[f'age_us{i}']) for i in range(1,7)))
            if row['phase'] not in ('sample','post_hold'):continue
            values=[float(row['t']),float(row['mcu_us'])*1e-6]
            values.extend(float(row[f'{key}{i}']) for key in ('q','tau','dq','qref','mode') for i in range(1,7))
            buckets.setdefault((int(row['point']),row['phase']),[]).append(values)
    audit=[];Q=[];T=[];LQ=[];LT=[];P=[];PQ=[];PT=[]
    for point in points:
        index=point['point'];s=np.asarray(buckets[(index,'sample')]);post=np.asarray(buckets[(index,'post_hold')])
        q,tau=s[:,2:8],s[:,8:14];late=s[s[:,1]>=s[-1,1]-.25]
        dt=s[:,1]-s[:,1].mean();slope=dt@q/(dt@dt)
        item=dict(point=index+1,accepted=point['accepted'],samples=len(s),duration_s=float(np.ptp(s[:,1])),
            max_gap_s=float(np.diff(s[:,1]).max()),q_std_deg=np.rad2deg(q.std(axis=0)).tolist(),
            q_drift_deg_s=np.rad2deg(slope).tolist(),tau_std_nm=tau.std(axis=0).tolist(),
            q_mean_error_rad=float(np.max(abs(q.mean(axis=0)-point['q']))),
            tau_mean_error_nm=float(np.max(abs(tau.mean(axis=0)-point['tau']))),
            target_error_deg=np.rad2deg(q.mean(axis=0)-s[:,20:26].mean(axis=0)).tolist(),
            sample_to_post_q_deg=np.rad2deg(post[:,2:8].mean(axis=0)-q.mean(axis=0)).tolist(),
            sample_to_post_tau_nm=(post[:,8:14].mean(axis=0)-tau.mean(axis=0)).tolist(),
            modes=np.unique(s[:,26:32]).tolist(),finite=bool(np.all(np.isfinite(s))))
        audit.append(item)
        if point['accepted']:
            Q.append(q.mean(axis=0));T.append(tau.mean(axis=0));LQ.append(late[:,2:8].mean(axis=0));LT.append(late[:,8:14].mean(axis=0));P.append(index)
            PQ.append(post[:,2:8].mean(axis=0));PT.append(post[:,8:14].mean(axis=0))
    Q,T,LQ,LT,P,PQ,PT=map(np.asarray,(Q,T,LQ,LT,P,PQ,PT))
    A=np.array([model.regressor(q) for q in Q]);B=A[:,:,20:];fixed=A[:,:,:20]@model.theta[:20];res=T-fixed
    def solve(ids,kind='pure',dataA=A,dataT=T):
        b=dataA[ids,:,20:].reshape(-1,4);y=(dataT[ids]-dataA[ids,:,:20]@model.theta[:20]).reshape(-1)
        intercept=np.tile(np.eye(6),(len(ids),1));bias=np.zeros(6)
        if kind=='bias':
            x=np.column_stack([b,intercept]);sol=np.linalg.lstsq(x,y,rcond=1e-10)[0];theta=sol[:4];bias=sol[4:]
        elif kind=='fixed_mass':
            mass=model.theta[20];theta=np.r_[mass,np.linalg.lstsq(b[:,1:],y-b[:,0]*mass,rcond=1e-10)[0]]
        elif kind=='robust':
            theta=np.linalg.lstsq(b,y,rcond=1e-10)[0]
            for _ in range(20):
                residual=(y-b@theta).reshape(-1,6)
                scales=np.maximum(.02,1.4826*np.median(abs(residual-np.median(residual,axis=0)),axis=0))
                w=np.minimum(1,1.345*np.tile(scales,len(ids))/np.maximum(abs(residual.ravel()),1e-12))
                theta=np.linalg.lstsq(b*np.sqrt(w[:,None]),y*np.sqrt(w),rcond=1e-10)[0]
        else:theta=np.linalg.lstsq(b,y,rcond=1e-10)[0]
        return theta,bias
    ids=np.arange(len(Q));comparisons={}
    for kind in ('pure','fixed_mass','bias','robust'):
        theta,bias=solve(ids,kind);pred=B@theta+fixed+bias
        cv=np.zeros_like(T)
        for test in np.array_split(ids,5):
            train=np.setdiff1d(ids,test);th,bi=solve(train,kind);cv[test]=B[test]@th+fixed[test]+bi
        comparisons[kind]=dict(mass_kg=float(theta[0]),com_mm=(theta[1:]/theta[0]*1000).tolist(),
            first_moment_kg_m=theta[1:].tolist(),bias_nm=bias.tolist(),
            rmse_nm=np.sqrt(np.mean((pred-T)**2,axis=0)).tolist(),
            blocked_holdout_rmse_nm=np.sqrt(np.mean((cv-T)**2,axis=0)).tolist(),
            overall_rmse_nm=float(np.sqrt(np.mean((pred-T)**2))))
    fitted=fit_gravity(Q,T,model,'link6');write_json(out/'link6_equivalent.json',fitted)
    variants={}
    for label,q,t in [('last_025s',LQ,LT),('post_hold',PQ,PT)]:
        fit=fit_gravity(q,t,model,'link6');variants[label]=fit['link6']
    for label,mask in [('scans35',P<35),('combinations243',(P>=35)&(P<278)),('exclude_extra7',P<278)]:
        fit=fit_gravity(Q[mask],T[mask],model,'link6');variants[label]=dict(**fit['link6'],rank=fit['rank'])
    rng=np.random.default_rng(947);boot=[]
    for _ in range(500):
        th,_=solve(rng.choice(ids,len(ids),replace=True));boot.append(np.r_[th[0],1000*th[1:]/th[0]])
    base_error=A@model.theta-T;new_error=A@np.array(fitted['theta'])-T
    report=dict(raw_rows=count,raw_stale_rows=stale,raw_fault_nonzero_rows=faults,max_feedback_age_us=max_age,
        duplicate_state_sequences=seq_duplicates,points=len(points),accepted=len(Q),
        rejected_points=[r['point'] for r in audit if not r['accepted']],
        sample_rows=sum(r['samples'] for r in audit),sample_count_range=[min(r['samples'] for r in audit),max(r['samples'] for r in audit)],
        sample_duration_range_s=[min(r['duration_s'] for r in audit),max(r['duration_s'] for r in audit)],
        max_sample_gap_s=max(r['max_gap_s'] for r in audit),
        max_q_std_deg=np.max([r['q_std_deg'] for r in audit],axis=0).tolist(),
        max_abs_drift_deg_s=np.max(np.abs([r['q_drift_deg_s'] for r in audit]),axis=0).tolist(),
        max_summary_q_error_rad=max(r['q_mean_error_rad'] for r in audit),max_summary_tau_error_nm=max(r['tau_mean_error_nm'] for r in audit),
        max_target_error_deg=np.max(np.abs([r['target_error_deg'] for r in audit]),axis=0).tolist(),
        max_sample_to_post_tau_change_nm=np.max(np.abs([r['sample_to_post_tau_nm'] for r in audit]),axis=0).tolist(),
        baseline_overall_rmse_nm=float(np.sqrt(np.mean(base_error**2))),fitted_overall_rmse_nm=float(np.sqrt(np.mean(new_error**2))),
        comparisons=comparisons,window_and_pose_sensitivity=variants,
        conditional_pose_bootstrap_95_mass_com_mm=np.percentile(boot,[2.5,97.5],axis=0).tolist(),
        observability=fitted['observability'],
        caveat='Bootstrap is conditional on fixed upstream model and this dataset; it excludes model, torque-scale, friction and bias errors. Raw fault fields lack validity bits in CSV; zero values alone do not prove fault-free.')
    write_json(out/'report.json',report);write_json(out/'point_quality.json',audit)
    with (out/'residuals.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['point','joint','baseline_error_nm','fit_error_nm'])
        for n,point in enumerate(P):
            for j in range(6):w.writerow([point+1,j+1,base_error[n,j],new_error[n,j]])
    print(__import__('json').dumps(report,indent=2))


if __name__=='__main__':main()

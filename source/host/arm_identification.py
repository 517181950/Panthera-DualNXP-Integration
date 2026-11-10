"""Offline gravity / friction identification; consumes actual feedback only."""
import argparse
import csv
from pathlib import Path
import arm_runtime
import numpy as np
from arm_dynamics import Gravity, read_json, write_json, fit_gravity


def model_from_metadata(directory):
    meta=read_json(Path(directory)/'metadata.json'); a=meta['arguments']
    model=Gravity(a['urdf'], dict(sign=meta['model_identity']['sign'],offset_rad=meta['model_identity']['offset_rad']),
                  None,a['payload_mass'],a['payload_com'],a['gravity'])
    if model.identity!=meta['model_identity']: raise ValueError('Dataset model identity changed')
    if 'gravity_theta' in meta:
        model.theta=np.asarray(meta['gravity_theta'],float)
    return model


def load_samples(directory):
    rows=[]
    for path in sorted(Path(directory).glob('raw*.csv')):
        with path.open(encoding='utf-8') as f:
            attempt=list(csv.DictReader(f))
        # Keep the latest complete pass; never concatenate interrupted trajectories.
        completed={r['point'] for r in attempt if r['phase']=='post_hold'}
        rows=[r for r in rows if r['point'] not in completed]
        rows.extend(r for r in attempt if r['point'] in completed)
    return rows


def vectors(rows,key):
    return np.array([[float(r[f'{key}{i+1}']) for i in range(6)] for r in rows])


def nonnegative_line(v,y):
    """Two-parameter NNLS with exact boundary solutions, without SciPy."""
    X=np.column_stack([np.ones(len(v)),v]); raw=np.linalg.lstsq(X,y,rcond=None)[0]
    candidates=[np.array([max(0,float(np.mean(y))),0]),
                np.array([0,max(0,float(v@y/(v@v)))])]
    if np.all(raw>=0): candidates.append(raw)
    best=min(candidates,key=lambda p:np.sum((X@p-y)**2))
    return best,raw


def review_pass(rows,point,model,window=.2,tolerance=.15):
    rows=[r for r in rows if r['phase']=='cruise' and int(r['fresh'])]
    if len(rows)<10: return [],dict(status='too_few_cruise_samples',samples=len(rows))
    t=np.array([float(r['mcu_us'])*1e-6 for r in rows]); q=vectors(rows,'q'); tau=vectors(rows,'tau')
    j=point['joint']-1; target=np.deg2rad(point['speed_deg_s'])*point['direction']
    start=np.array(point['start_rad']); end=np.array(point['end_rad'])
    lo,hi=sorted([start[j],end[j]]); margin=(hi-lo)*.25
    gravity=np.array([model(x) for x in q]); accepted=[]
    for i in range(len(rows)):
        mask=abs(t-t[i])<=window/2
        if mask.sum()<5 or np.ptp(t[mask])<.75*window: continue
        dt=t[mask]-t[mask].mean(); slope=dt@q[mask,j]/(dt@dt)
        residual=q[mask,j]-(q[mask,j].mean()+slope*dt)
        if abs(slope-target)>abs(target)*tolerance or np.std(residual)>np.deg2rad(.2): continue
        if not lo+margin<=q[i,j]<=hi-margin: continue
        accepted.append(dict(q=q[i],v=float(slope),r=float(tau[i,j]-gravity[i,j])))
    return accepted,dict(status='usable' if len(accepted)>=10 else 'insufficient',samples=len(rows),accepted=len(accepted))


def fit_friction(directory,model):
    directory=Path(directory); plan=read_json(directory/'plan.json'); raw=load_samples(directory)
    grouped={}
    for row in raw: grouped.setdefault(row['point'],[]).append(row)
    passes=[]; diagnostics=[]
    for index,point in enumerate(plan):
        samples,quality=review_pass(grouped.get(str(index),[]),point,model)
        passes.append((point,samples)); diagnostics.append(dict(point=index,**quality))
    joints=[]
    for j in range(6):
        speeds=sorted(set(p['speed_deg_s'] for p,_ in passes if p['joint']==j+1))
        cells=[]
        for speed in speeds:
            positive=[s for p,ss in passes if p['joint']==j+1 and p['speed_deg_s']==speed and p['direction']==1 for s in ss]
            negative=[s for p,ss in passes if p['joint']==j+1 and p['speed_deg_s']==speed and p['direction']==-1 for s in ss]
            if not positive or not negative: continue
            # Actual joint coordinate bins, shared across all speeds; match all six angles.
            bins={int(np.floor(s['q'][j]/np.deg2rad(2))) for s in positive}
            for b in sorted(bins):
                pos=[s for s in positive if int(np.floor(s['q'][j]/np.deg2rad(2)))==b]
                neg=[s for s in negative if int(np.floor(s['q'][j]/np.deg2rad(2)))==b]
                if min(len(pos),len(neg))<3: continue
                qp=np.mean([s['q'] for s in pos],axis=0); qn=np.mean([s['q'] for s in neg],axis=0)
                vp=np.mean([s['v'] for s in pos]); vn=-np.mean([s['v'] for s in neg])
                if max(abs(qp-qn))>np.deg2rad(2) or abs(vp-vn)>.15*max(vp,vn): continue
                cells.append(dict(bin=b,speed=speed,v=float((vp+vn)/2),
                                  y=float((np.mean([s['r'] for s in pos])-np.mean([s['r'] for s in neg]))/2)))
        usable_speeds=sorted({c['speed'] for c in cells})
        if len(usable_speeds)<2:
            joints.append(None); continue
        common=set.intersection(*[{c['bin'] for c in cells if c['speed']==speed} for speed in usable_speeds])
        cells=[c for c in cells if c['bin'] in common]
        if len(common)<3:
            joints.append(None); continue
        # Each speed contributes the same number of common bins, hence equal weight.
        v=np.array([c['v'] for c in cells]); y=np.array([c['y'] for c in cells])
        best,unconstrained=nonnegative_line(v,y)
        residual=y-best[0]-best[1]*v
        rng=np.random.default_rng(947); estimates=[]; bins=sorted(common)
        for _ in range(200):
            selected=rng.choice(bins,len(bins),replace=True)
            sampled=[c for b in selected for c in cells if c['bin']==b]
            estimates.append(nonnegative_line(np.array([c['v'] for c in sampled]),np.array([c['y'] for c in sampled]))[0])
        joints.append(dict(fc_nm=float(best[0]),b_nm_s_rad=float(best[1]),unconstrained=unconstrained.tolist(),
                           rmse_nm=float(np.sqrt(np.mean(residual**2))),speed_range_rad_s=[float(v.min()),float(v.max())],
                           ci95=np.percentile(estimates,[2.5,97.5],axis=0).tolist(),common_bins=len(common),cells=cells))
    return dict(format='panthera-friction-v1',model_identity=model.identity,gravity_theta=model.theta.tolist(),joints=joints,pass_quality=diagnostics,
                interpretation='Coulomb + viscous moving friction. null = insufficient matched data; B=0 is a constrained fit, not proof of absent viscous friction.')


def summarize_static(directory):
    results=read_json(Path(directory)/'static_results.json'); joints=[]
    for j in range(1,7):
        p=[r['breakaway']['effort_nm'] for r in results if r['joint']==j and r['direction']==1 and r['status']=='detected']
        n=[r['breakaway']['effort_nm'] for r in results if r['joint']==j and r['direction']==-1 and r['status']=='detected']
        joints.append(dict(joint=j,positive_trials=p,negative_trials=n,
                           symmetric_threshold_nm=float((np.mean(p)-np.mean(n))/2) if p and n else None,
                           residual_bias_nm=float((np.mean(p)+np.mean(n))/2) if p and n else None))
    return dict(joints=joints,interpretation='Paired ramp thresholds at the tested pose; displacement detection lags physical onset. Repeat at multiple loads; censored/opposite-drift trials are excluded.')


def main():
    p=argparse.ArgumentParser(description='Offline identification; no robot connection')
    p.add_argument('kind',choices=['gravity','friction','static']);p.add_argument('dataset');p.add_argument('--output')
    p.add_argument('--fit-scope',choices=['link6','all'],help='Default to capture metadata; legacy captures use all')
    a=p.parse_args();directory=Path(a.dataset)
    if a.kind=='static': result=summarize_static(directory)
    else:
        model=model_from_metadata(directory)
        if a.kind=='gravity':
            points=[r for r in read_json(directory/'points.json') if r['accepted']]
            scope=a.fit_scope or read_json(directory/'metadata.json')['arguments'].get('fit_scope','all')
            result=fit_gravity([r['q'] for r in points],[r['tau'] for r in points],model,scope)
        else: result=fit_friction(directory,model)
    output=Path(a.output or directory/(a.kind+'_identified.json'))
    write_json(output,result);print(output)
    return 0

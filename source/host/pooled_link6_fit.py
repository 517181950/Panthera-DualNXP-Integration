"""Offline, equal-point-weight link6 identification using all accepted samples.

J2-J6 torque equations only. All six measured angles define each posture.
Upstream inertial parameters remain exactly those recorded during acquisition.
"""
import argparse
from pathlib import Path
import arm_runtime
import numpy as np
from arm_dynamics import read_json, write_json
from arm_identification import model_from_metadata
from priority_gravity_fit import load_points, solve, export_urdf, FIT_JOINTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('datasets', nargs='+')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    model = model_from_metadata(args.datasets[-1])
    arrays = []; observations = []; records = []; labels = []; groups = []
    for index, dataset in enumerate(args.datasets):
        other = model_from_metadata(dataset)
        if other.identity != model.identity:
            raise ValueError('Dataset model identity mismatch')
        if not np.array_equal(other.theta[:20], model.theta[:20]):
            raise ValueError('Dataset upstream link parameters differ')
        rows, q, tau = load_points(dataset)
        plan = read_json(Path(dataset)/'plan.json')['poses_deg']
        arrays.extend(model.regressor(x) for x in q)
        observations.extend(tau)
        labels.extend([index]*len(rows))
        for row in rows:
            groups.append(tuple(plan[row['point']]))
            records.append(dict(dataset=str(Path(dataset).resolve()), point=row['point']))
    A = np.array(arrays); tau = np.array(observations); labels = np.array(labels)
    weights = np.full(len(tau), 1/len(tau))
    theta, rank, singular = solve(A, tau, model.theta, weights, 'link6')
    assert np.array_equal(theta[:20], model.theta[:20])

    def evaluate(value):
        error = (A@value-tau)[:,FIT_JOINTS]
        return dict(joint_ids=[2,3,4,5,6], rmse_nm=float(np.sqrt(np.mean(error**2))),
                    joint_rmse_nm=np.sqrt(np.mean(error**2,axis=0)).tolist(),
                    datasets=[dict(path=str(Path(path).resolve()), points=int(np.sum(labels==i)),
                        rmse_nm=float(np.sqrt(np.mean(error[labels==i]**2))))
                        for i,path in enumerate(args.datasets)])

    # Keep repeats of the same requested pose together in validation folds.
    _, group_ids = np.unique(np.array(groups),axis=0,return_inverse=True)
    unique = np.unique(group_ids)
    rng = np.random.default_rng(20261030)
    assignment = np.empty(len(unique), dtype=int)
    assignment[rng.permutation(len(unique))] = np.arange(len(unique)) % 5
    predictions = np.zeros_like(tau)
    for fold in range(5):
        test = assignment[group_ids] == fold
        keep = ~test
        trial, _, _ = solve(A[keep],tau[keep],model.theta,
                             np.full(keep.sum(),1/keep.sum()),'link6')
        predictions[test] = A[test]@trial
    cv = (predictions-tau)[:,FIT_JOINTS]
    fitted_com = theta[21:24]/theta[20]*1000
    baseline_com = model.theta[21:24]/model.theta[20]*1000
    nominal_com = model.prior[21:24]/model.prior[20]*1000

    # Diagnostic only: honor the approximate 620 g hand assumption plus the
    # bare terminal link from the nominal URDF, while fitting its first moments.
    fixed = model.theta.copy(); fixed[20] = model.prior[20]
    x = A[:,FIT_JOINTS,:].reshape(-1,24)
    y = tau[:,FIT_JOINTS].ravel()
    fixed[21:24] += np.linalg.lstsq(x[:,21:24],y-x@fixed,rcond=1e-8)[0]
    issues = []
    if rank != 4: issues.append('Link6 static parameters are not fully observable')
    if theta[20] <= 0: issues.append('Nonpositive fitted link6 mass')
    result = dict(format='panthera-gravity-v1',model_identity=model.identity,
        theta=theta.tolist(),fit_scope='link6',fit_joint_ids=[2,3,4,5,6],
        fixed_links=[1,2,3,4,5],usable=not issues,quality_issues=issues,
        rank=rank,parameters=4,fully_observable=rank==4,
        normalized_condition=float(singular[0]/singular[-1]) if rank==4 else None,
        weighting='Equal weight per accepted point, independent of raw frame count',
        poses=len(tau),distinct_target_poses=len(unique),records=records,
        link6=dict(mass_kg=float(theta[20]),com_mm=fitted_com.tolist(),
            mass_change_from_393g_g=float((theta[20]-model.theta[20])*1000),
            com_change_from_393g_mm=(fitted_com-baseline_com).tolist(),
            nominal_mass_kg=float(model.prior[20]),nominal_com_mm=nominal_com.tolist(),
            mass_change_from_nominal_g=float((theta[20]-model.prior[20])*1000),
            com_change_from_nominal_mm=(fitted_com-nominal_com).tolist()),
        evaluation=evaluate(theta),baseline_evaluation=evaluate(model.theta),
        grouped_fivefold_rmse_nm=float(np.sqrt(np.mean(cv**2))),
        grouped_fivefold_joint_rmse_nm=np.sqrt(np.mean(cv**2,axis=0)).tolist(),
        fixed_nominal_mass_diagnostic=dict(mass_kg=float(fixed[20]),
            com_mm=(fixed[21:24]/fixed[20]*1000).tolist(),evaluation=evaluate(fixed)),
        interpretation='Static equivalent estimate conditional on correct upstream model, measured torque and encoder mapping. Rank does not establish physical accuracy; rotational inertia is not identified. Fixed-mass alternative is diagnostic only.')
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    write_json(out/'link6_identified.json',result)
    if not issues: export_urdf(model,theta,out/'link6_identified.urdf')
    print({k:v for k,v in result.items() if k not in ('records','model_identity','theta')})
    print('Saved',out.resolve())
    return 0


if __name__=='__main__':raise SystemExit(main())

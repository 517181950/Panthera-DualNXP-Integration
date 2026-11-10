"""Host-side gravity model. SI units, explicit encoder -> URDF mapping.

Static identification changes gravity parameters only, never claims identified
dynamic inertia. Pinocchio includes fixed-joint inertias when loading the URDF.
"""
from pathlib import Path
import hashlib
import json
import arm_runtime  # initialize native DLL lookup before NumPy/Pinocchio
import numpy as np
import pinocchio as pin

DEFAULT_URDF = Path(__file__).parent / 'Assets' / 'PantheraHT_v4_hand620g.urdf'


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    temp.replace(path)


class Gravity:
    def __init__(self, urdf=DEFAULT_URDF, mapping=None, identified=None,
                 payload_mass=0., payload_com=(0., 0., 0.), gravity=(0., 0., -9.81)):
        self.path = Path(urdf).resolve()
        self.model = pin.buildModelFromUrdf(str(self.path))
        if self.model.nq != 6 or self.model.nv != 6:
            raise ValueError('Model must have six scalar joints')
        mapping = mapping or {}
        self.sign = np.asarray(mapping.get('sign', [1]*6), float)
        self.offset = np.asarray(mapping.get('offset_rad', [0]*6), float)
        if self.sign.shape != (6,) or self.offset.shape != (6,) or not np.all(np.isin(self.sign, [-1, 1])) or not np.all(np.isfinite(self.offset)):
            raise ValueError('Mapping: six signs +/-1 and six finite offsets')
        self.model.gravity.linear = np.asarray(gravity, float)
        if payload_mass < 0 or not np.isfinite(payload_mass) or not np.all(np.isfinite(payload_com)):
            raise ValueError('Invalid payload')
        # Payload COM is expressed in the last moving joint frame (joint6).
        self.model.inertias[6] += pin.Inertia(payload_mass, np.asarray(payload_com, float), np.zeros((3, 3)))
        self.data = self.model.createData()
        self.indices = np.array([10*j+k for j in range(6) for k in range(4)])
        self.prior = np.concatenate([i.toDynamicParameters()[:4] for i in self.model.inertias[1:]])
        self.identity = dict(urdf_sha256=hashlib.sha256(self.path.read_bytes()).hexdigest(),
                             sign=self.sign.tolist(), offset_rad=self.offset.tolist(),
                             payload_mass=payload_mass, payload_com=list(payload_com), gravity=list(gravity))
        self.theta = self.prior.copy()
        if identified:
            doc = read_json(identified)
            if doc.get('usable') is False:
                raise ValueError('Gravity fit is not usable: '+str(doc.get('quality_issues',[])))
            if doc['model_identity'] != self.identity:
                raise ValueError('Identified model does not match URDF/mapping/payload/gravity')
            self.theta = np.asarray(doc['theta'], float)
            if self.theta.shape != (24,) or not np.all(np.isfinite(self.theta)):
                raise ValueError('Invalid gravity parameters')

    def regressor(self, q):
        qm = self.sign * np.asarray(q) + self.offset
        y = pin.computeJointTorqueRegressor(self.model, self.data, qm, np.zeros(6), np.zeros(6))
        return self.sign[:, None] * y[:, self.indices]

    def __call__(self, q):
        return self.regressor(q) @ self.theta


def quintic(t, duration, start, end):
    u = np.clip(t/duration, 0., 1.)
    blend = 10*u**3 - 15*u**4 + 6*u**5
    slope = (30*u**2 - 60*u**3 + 30*u**4)/duration
    delta = np.asarray(end) - start
    return np.asarray(start) + blend*delta, slope*delta


def cruise(t, start, end, speed, ramp):
    """Trapezoidal velocity with cosine acceleration; exact endpoint distance."""
    distance = abs(end-start)
    speed = abs(speed)
    direction = np.sign(end-start)
    if speed <= 0 or ramp <= 0 or distance < speed*ramp:
        raise ValueError('Travel must exceed speed * ramp time')
    flat = distance/speed-ramp
    total = flat+2*ramp
    t = max(0., t)
    if t < ramp:
        x = speed/2*(t-ramp/np.pi*np.sin(np.pi*t/ramp))
        v = speed/2*(1-np.cos(np.pi*t/ramp)); phase = 'accelerate'
    elif t < ramp+flat:
        x = speed*(t-ramp/2); v = speed; phase = 'cruise'
    elif t < total:
        u = t-ramp-flat
        x = speed*(ramp/2+flat)+speed/2*(u+ramp/np.pi*np.sin(np.pi*u/ramp))
        v = speed/2*(1+np.cos(np.pi*u/ramp)); phase = 'decelerate'
    else:
        x = distance; v = 0.; phase = 'done'
    return start+direction*x, direction*v, phase, total


def gravity_observability(q, model, scope='link6'):
    active=np.arange(20,24) if scope=='link6' else np.arange(24)
    scale=np.tile([1.,.1,.1,.1],6)[active]
    matrix=np.vstack([model.regressor(x)[:,active] for x in q])*scale
    singular=np.linalg.svd(matrix,compute_uv=False)
    rank=int(np.sum(singular>singular[0]*1e-8)) if singular[0]>0 else 0
    return dict(rank=rank,parameters=len(active),singular_values=singular.tolist(),
                normalized_condition=float(singular[0]/singular[-1]) if rank==len(active) else None,
                fully_observable=rank==len(active))


def fit_link6_mass_constrained(q, tau, model, mass=.624, tolerance=.03):
    """Fit terminal COM with a measured TOTAL mass interval, never additive.

The profiled least-squares objective is convex in mass: use its unconstrained
optimum if inside the interval, otherwise fix the nearest boundary and solve
the three first moments again. Only J2-J6 torques enter the objective.
"""
    q,tau=np.asarray(q),np.asarray(tau)
    if len(q)<10 or q.shape!=(len(q),6) or tau.shape!=q.shape or not np.all(np.isfinite(q)) or not np.all(np.isfinite(tau)):
        raise ValueError('Need at least 10 finite N x 6 position/torque measurements')
    if not np.isfinite(mass+tolerance) or tolerance<0 or mass<=tolerance:
        raise ValueError('Mass interval must be positive and finite')
    A=np.array([model.regressor(x) for x in q])[:,1:,:]
    base=model.theta.copy();lower=mass-tolerance;upper=mass+tolerance
    def solve(ids):
        x=A[ids].reshape(-1,24);y=tau[ids,1:].ravel()-x[:,:20]@base[:20]
        terminal=np.linalg.lstsq(x[:,20:24],y,rcond=1e-8)[0]
        terminal[0]=np.clip(terminal[0],lower,upper)
        terminal[1:]=np.linalg.lstsq(x[:,21:24],y-x[:,20]*terminal[0],rcond=1e-8)[0]
        return np.r_[base[:20],terminal]
    ids=np.arange(len(q));theta=solve(ids);prediction=np.zeros((len(q),5))
    for fold in range(5):
        test=ids%5==fold
        prediction[test]=A[test]@solve(ids[~test])
    matrix=A.reshape(-1,24)[:,21:24] if tolerance==0 else A.reshape(-1,24)[:,20:24]*[1,.1,.1,.1]
    singular=np.linalg.svd(matrix,compute_uv=False)
    rank=int(np.sum(singular>singular[0]*1e-8)) if singular[0]>0 else 0
    count=matrix.shape[1];issues=[]
    if rank<count:issues.append('Terminal parameters not fully observable')
    error=A@theta-tau[:,1:];held=prediction-tau[:,1:]
    com=theta[21:24]/theta[20];initial_com=base[21:24]/base[20]
    return dict(format='panthera-gravity-v1',model_identity=model.identity,theta=theta.tolist(),
        fit_scope='link6',fit_joint_ids=[2,3,4,5,6],fixed_links=[1,2,3,4,5],
        poses=len(q),rank=rank,parameters=count,fully_observable=rank==count,usable=not issues,quality_issues=issues,
        mass_constraint=dict(nominal_kg=mass,tolerance_kg=tolerance,lower_kg=lower,upper_kg=upper,
            at_boundary=bool(tolerance>0 and (np.isclose(theta[20],lower) or np.isclose(theta[20],upper))),
            interpretation='TOTAL replacement hand mass, not added to old link6 mass'),
        link6=dict(aggregate_mass_kg=float(theta[20]),first_moment_kg_m=theta[21:24].tolist(),
            aggregate_com_m=com.tolist(),aggregate_com_mm=(com*1000).tolist(),
            initial_mass_kg=float(base[20]),mass_change_kg=float(theta[20]-base[20]),initial_com_mm=(initial_com*1000).tolist()),
        training_rmse_nm=np.sqrt(np.mean(error**2,axis=0)).tolist(),training_overall_rmse_nm=float(np.sqrt(np.mean(error**2))),
        holdout_rmse_nm=np.sqrt(np.mean(held**2,axis=0)).tolist(),holdout_overall_rmse_nm=float(np.sqrt(np.mean(held**2))),
        interpretation='Equal point weights; J2-J6 torque only. Upstream links fixed; terminal total mass constrained by measurement. Rotational inertia not identified. A boundary solution indicates tension between torque data/model and mass prior.')


def fit_gravity(q, tau, model, scope='all'):
    q, tau = np.asarray(q), np.asarray(tau)
    if len(q) < 10:
        raise ValueError('At least 10 distinct stable poses are required')
    if q.shape!=(len(q),6) or tau.shape!=q.shape or not np.all(np.isfinite(q)) or not np.all(np.isfinite(tau)):
        raise ValueError('Expected finite N x 6 actual positions and measured torques')
    A = np.stack([model.regressor(x) for x in q])
    # Normalized minimum change selects one equivalent solution in null space.
    if scope not in ('all','link6'):raise ValueError('Gravity fit scope must be all or link6')
    active=np.arange(20,24) if scope=='link6' else np.arange(24)
    base=model.theta.copy()
    scale = np.tile([1., .1, .1, .1], 6)[active]
    def solve(ids):
        x = A[ids].reshape(-1, 24); y = tau[ids].reshape(-1)
        result=base.copy()
        if scope=='link6':
            # Subtract ONLY the fixed upstream links; solve terminal mass/first
            # moments outright. No penalty toward 620g or the old tool COM.
            result[active]=scale*np.linalg.lstsq(x[:,active]*scale, y-x[:,:20]@base[:20],rcond=1e-8)[0]
        else:
            result[active]+=scale*np.linalg.lstsq(x[:,active]*scale, y-x@base, rcond=1e-8)[0]
        return result
    predictions = np.zeros_like(tau)
    ids = np.arange(len(q))
    for fold in range(5):
        train, test = ids[ids % 5 != fold], ids[ids % 5 == fold]
        predictions[test] = A[test] @ solve(train)
    theta = solve(ids)
    observation=gravity_observability(q,model,scope)
    rank=observation['rank']
    issues=[]
    if scope=='link6' and rank<4:issues.append('Link6 mass and all first moments are not observable; vary arm and wrist poses.')
    if scope=='link6' and theta[20]<=0:issues.append('Fitted link6 mass is nonpositive; check torque scale/sign and upstream gravity model.')
    initial_com=base[21:24]/base[20] if base[20]>0 else None
    fitted_com=theta[21:24]/theta[20] if theta[20]>0 else None
    return dict(format='panthera-gravity-v1', model_identity=model.identity,
                theta=theta.tolist(), rank=rank,fit_scope=scope,active_parameter_indices=active.tolist(),
                parameters=len(active), poses=len(q),fully_observable=rank==len(active),
                observability=observation,usable=not issues,quality_issues=issues,
                fixed_links=[1,2,3,4,5] if scope=='link6' else [],
                link6_prior_penalty=0 if scope=='link6' else None,
                link6=dict(aggregate_mass_kg=float(theta[20]),first_moment_kg_m=theta[21:24].tolist(),
                           aggregate_com_m=fitted_com.tolist() if fitted_com is not None else None,
                           aggregate_com_mm=(1000*fitted_com).tolist() if fitted_com is not None else None,
                           initial_mass_kg=float(base[20]),mass_change_kg=float(theta[20]-base[20]),
                           initial_com_mm=(1000*initial_com).tolist() if initial_com is not None else None),
                baseline_rmse_nm=np.sqrt(np.mean((A@base-tau)**2, axis=0)).tolist(),
                holdout_rmse_nm=np.sqrt(np.mean((predictions-tau)**2, axis=0)).tolist(),
                training_rmse_nm=np.sqrt(np.mean((A@theta-tau)**2, axis=0)).tolist(),
                interpretation=('Only link6 aggregate m,mx,my,mz fitted; links1-5 fixed. Mass/COM interpretation requires rank 4, correct upstream model and torque feedback. Dynamic inertia not identified.' if scope=='link6' else 'Equivalent static gravity parameters; individual masses/COM and dynamic inertia are not uniquely identified.'))

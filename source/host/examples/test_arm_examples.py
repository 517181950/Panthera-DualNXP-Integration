"""Offline numerical + experiment lifecycle tests. Never connects to hardware."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import csv
import tempfile
import time
import unittest
from unittest.mock import patch
import arm_runtime
import numpy as np
import pinocchio as pin
import arm_lab as lab
from arm_dynamics import Gravity, quintic, cruise, fit_gravity, write_json, read_json, fit_link6_mass_constrained
from arm_identification import nonnegative_line, fit_friction, summarize_static
from robot_sdk import SendResult


class FakeRobot:
    instances=[]
    def __init__(self,**kwargs):
        self.q=np.zeros(6); self.v=np.zeros(6); self.tau=np.zeros(6)
        self.lease=0;self.seq=0;self.commands=[];self.operations=[];self.closed=False
        self.__class__.instances.append(self)
    def start(self): return self
    def discover(self,t): return True
    def start_state_stream(self,**kwargs): return {'status':0}
    def stop_state_stream(self): return {'status':0}
    def operation(self,op,**kwargs):
        self.operations.append((op,kwargs))
        if op==1:self.lease=1
        if op==2:self.lease=0
        return {'status':0}
    def send_arm(self,q,v,kp,kd,tau):
        self.commands.append((np.array(q),np.array(v),np.array(kp),np.array(kd),np.array(tau)))
        self.q=np.array(q);self.v=np.array(v);self.tau=np.array(tau)
        return SendResult(True,len(self.commands))
    def snapshot(self, *names):
        self.seq+=1
        return {'ArmState':dict(header={'sequence':self.seq,'tx_monotonic_us':int(time.perf_counter()*1e6)},
              online_mask=63,valid_q_mask=63,valid_dq_mask=63,valid_tau_mask=63,feedback_age_us=[0]*6,
              q_rad=self.q.tolist(),dq_rad_s=self.v.tolist(),tau_measured_nm=self.tau.tolist(),
              last_applied_command_sequence=len(self.commands))}
    def close(self):self.closed=True


class Clock:
    def __init__(self):self.t=1000.
    def perf_counter(self):
        self.t+=.00001
        return self.t
    def sleep(self,seconds):self.t+=max(0,seconds)


class NumericalTests(unittest.TestCase):
    def test_hand_mass_bounds_and_fixed_mass_ignore_j1(self):
        model=Gravity();q=np.random.default_rng(624).uniform(-2,2,(60,6))
        A=np.array([model.regressor(x) for x in q])
        for actual,expected in [(.624,.624),(.4,.594),(.8,.654)]:
            truth=model.theta.copy();truth[20:]=[actual,.05,.01,-.02]
            tau=A@truth;tau[:,0]=1e8
            fit=fit_link6_mass_constrained(q,tau,model,.624,.03)
            self.assertAlmostEqual(fit['theta'][20],expected)
            np.testing.assert_array_equal(fit['theta'][:20],model.theta[:20])
            theta=np.array(fit['theta']);residual=(A@theta-tau)[:,1:].ravel()
            design=A[:,1:,21:24].reshape(-1,3)
            np.testing.assert_allclose(design.T@residual,0,atol=1e-9)
            if actual==.624:
                np.testing.assert_allclose(theta,truth,atol=1e-10)
                self.assertLess(fit['training_overall_rmse_nm'],1e-10)
            fixed=fit_link6_mass_constrained(q,tau,model,.624,0.)
            self.assertEqual(fixed['theta'][20],.624)
            self.assertEqual(fixed['parameters'],3)

    def test_random50_capture_defaults(self):
        import runpy
        defaults=runpy.run_path(str(Path(__file__).parent/'14_random50-hand-com-capture.py'))['DEFAULTS']
        parser=lab.parser('gravity_capture');parser.set_defaults(**defaults);args=parser.parse_args([])
        plan=read_json(args.plan);q=np.array(plan['poses_deg'])
        self.assertEqual(q.shape,(50,6));self.assertEqual(len(np.unique(q,axis=0)),50)
        self.assertEqual((args.transition,args.settle,args.sample,args.post_hold),(1,1,1,1))
        self.assertEqual((args.link6_mass,args.link6_mass_tolerance),(.624,.03))
        self.assertEqual(Gravity(identified=args.identified).theta[20],.624)
        # The migration archive intentionally excludes historical run logs.
        # Validate the archived selection against the deterministic plan;
        # retain the original source_plan path as historical provenance.
        if Path(plan['source_plan']).is_file():
            original=np.array(read_json(plan['source_plan'])['poses_deg'])
        else:
            from arm_calibration_plan import legacy285
            original=np.array(legacy285())
        # JSON poses include round-trip degree/radian rounding (~1e-14 deg).
        np.testing.assert_allclose(q,original[plan['source_point_indices_zero_based']],rtol=0,atol=1e-12)

    def test_default_gravity_plan_and_timing_match_requested_285(self):
        a=lab.parser('gravity_capture').parse_args([])
        points=np.rad2deg(lab.plan_points(a,np.ones(6)))
        self.assertEqual(points.shape,(285,6))
        np.testing.assert_allclose(points[0],[0,90,180,-90,0,-135])
        np.testing.assert_allclose(points[6],[0,90,180,-90,0,135])
        np.testing.assert_allclose(points[7],[0,90,180,0,-90,0])
        np.testing.assert_allclose(points[13],[0,90,180,0,90,0])
        np.testing.assert_allclose(points[14],[0,90,180,-90,0,0])
        np.testing.assert_allclose(points[20],[0,90,180,90,0,0])
        np.testing.assert_allclose(points[21],[0,90,200,0,0,0])
        np.testing.assert_allclose(points[27],[0,90,45,0,0,0])
        np.testing.assert_allclose(points[28],[0,30,135,0,0,0])
        np.testing.assert_allclose(points[34],[0,160,135,0,0,0])
        np.testing.assert_allclose(points[35],[0,90,90,0,0,0])
        np.testing.assert_allclose(points[277],[0,60,60,-30,-30,-30])
        np.testing.assert_allclose(points[278],[0,5,10,30,0,90])
        np.testing.assert_allclose(points[-1],[0,70,10,30,0,90])
        self.assertEqual((a.transition,a.settle,a.sample,a.post_hold),(1,.5,.5,.5))
        self.assertEqual(285*(a.transition+a.settle+a.sample+a.post_hold),712.5)
        self.assertEqual(lab.parser('impedance').parse_args([]).transition,5)

    def test_large_link6_change_is_not_pulled_toward_initial_guess(self):
        seed=Gravity();truth=Gravity()
        truth.theta[20:]=[1.8,.36,-.144,.216]  # 1.8kg, COM [200,-80,120]mm
        q=np.random.default_rng(947).uniform(-2,2,(90,6))
        tau=np.array([truth(x) for x in q])
        fit=fit_gravity(q,tau,seed,'link6')
        seed.theta[20:]=[.08,-.04,.02,-.01]  # deliberately different, poor starting guess
        other=fit_gravity(q,tau,seed,'link6')
        np.testing.assert_allclose(fit['theta'][20:],truth.theta[20:],atol=1e-10)
        np.testing.assert_allclose(other['theta'][20:],truth.theta[20:],atol=1e-10)
        np.testing.assert_array_equal(other['theta'][:20],seed.theta[:20])
        self.assertTrue(fit['usable']);self.assertEqual(fit['link6_prior_penalty'],0)

    def test_unobservable_link6_fit_is_not_loadable(self):
        model=Gravity();q=np.zeros((12,6));tau=np.array([model(x) for x in q])
        fit=fit_gravity(q,tau,model,'link6')
        self.assertFalse(fit['usable']);self.assertLess(fit['rank'],4)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'unobservable.json';write_json(path,fit)
            with self.assertRaises(ValueError):Gravity(identified=path)

    def test_link6_only_fit_preserves_upstream_and_recovers_payload(self):
        model=Gravity();truth=Gravity(payload_mass=.12,payload_com=[.11,-.02,.04])
        q=np.random.default_rng(22).uniform(-1.5,1.5,(80,6))
        result=fit_gravity(q,np.array([truth(x) for x in q]),model,scope='link6')
        self.assertEqual(result['parameters'],4);self.assertEqual(result['rank'],4)
        np.testing.assert_array_equal(result['theta'][:20],model.theta[:20])
        np.testing.assert_allclose(result['theta'][20:],truth.theta[20:],atol=1e-9)
        self.assertLess(max(result['holdout_rmse_nm']),1e-9)

    def test_v4_asset_upstream_unchanged_and_old_tool_replaced(self):
        import xml.etree.ElementTree as ET
        from arm_dynamics import DEFAULT_URDF
        source=ET.parse(DEFAULT_URDF.parent/'PantheraHT_identified_v4_source.urdf').getroot()
        current=ET.parse(DEFAULT_URDF).getroot()
        for name in ['base_link']+[f'link{i}' for i in range(1,6)]:
            for child in ('origin','mass','inertia'):
                path=f"link[@name='{name}']/inertial/{child}"
                self.assertEqual(source.find(path).attrib,current.find(path).attrib)
        self.assertIsNone(current.find("link[@name='gripper_center_link']"))
        self.assertAlmostEqual(Gravity().model.inertias[6].mass,.62918329)

    def test_gravity_matches_pinocchio_with_mapping_and_payload(self):
        model=Gravity(mapping={'sign':[1,-1,1,-1,1,-1],'offset_rad':[.1]*6},payload_mass=.6,payload_com=[.1,.02,.03])
        rng=np.random.default_rng(42)
        for q in rng.uniform(-2,2,(20,6)):
            expected=model.sign*pin.computeGeneralizedGravity(model.model,model.data,model.sign*q+model.offset)
            np.testing.assert_allclose(model(q),expected,atol=1e-10)

    def test_gravity_identification_generalizes(self):
        base=Gravity();truth=Gravity(payload_mass=.45,payload_com=[.08,.01,-.04])
        rng=np.random.default_rng(1);q=rng.uniform(-2,2,(100,6));tau=np.array([truth(x) for x in q])
        fit=fit_gravity(q,tau,base)
        self.assertLess(max(fit['holdout_rmse_nm']),1e-8)
        self.assertLess(fit['rank'],24)
        for x in rng.uniform(-2,2,(10,6)):
            np.testing.assert_allclose(base.regressor(x)@fit['theta'],truth(x),atol=1e-8)

    def test_identified_model_roundtrip_and_identity_rejection(self):
        model=Gravity();q=np.random.default_rng(7).uniform(-2,2,(20,6))
        fit=fit_gravity(q,np.array([model(x) for x in q]),model)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'fit.json';write_json(p,fit)
            fitted=Gravity(identified=p)
            np.testing.assert_allclose(fitted(q[0]),model(q[0]))
            with self.assertRaises(ValueError):Gravity(identified=p,payload_mass=.2)

    def test_trajectories_and_velocity_derivative(self):
        a=np.zeros(6);b=np.ones(6)
        for t in (0,1,2,3):
            q,v=quintic(t,3,a,b)
            if 0<t<3:
                delta=(quintic(t+1e-5,3,a,b)[0]-quintic(t-1e-5,3,a,b)[0])/2e-5
                np.testing.assert_allclose(v,delta,atol=1e-8)
        np.testing.assert_allclose(quintic(3,3,a,b)[0],b)
        for start,end in ((-1,1),(1,-1)):
            total=cruise(0,start,end,.5,.2)[3]
            self.assertAlmostEqual(cruise(total,start,end,.5,.2)[0],end)
            for t in np.linspace(.001,total-.001,40):
                q,v,_,_=cruise(t,start,end,.5,.2)
                fd=(cruise(t+1e-6,start,end,.5,.2)[0]-cruise(t-1e-6,start,end,.5,.2)[0])/2e-6
                self.assertAlmostEqual(v,fd,places=6)

    def test_nonnegative_friction_fit(self):
        v=np.array([.1,.2,.4,.8])
        np.testing.assert_allclose(nonnegative_line(v,.2+.03*v)[0],[.2,.03],atol=1e-12)
        fit,raw=nonnegative_line(v,.3-.05*v)
        self.assertEqual(fit[1],0);self.assertLess(raw[1],0)

    def test_full_friction_pipeline_recovers_truth(self):
        model=Gravity();plan=[];records=[]
        for speed in (5,10,20):
            for direction in (1,-1):
                start=np.zeros(6);end=start.copy();start[0]=-direction*.35;end[0]=direction*.35
                plan.append(dict(joint=1,speed_deg_s=speed,direction=direction,start_rad=start.tolist(),end_rad=end.tolist()))
                velocity=np.deg2rad(speed)*direction;total=.7/abs(velocity)
                for t in np.arange(0,total,.01):
                    q=start.copy();q[0]+=velocity*t;tau=model(q);tau[0]+=.2*direction+.04*velocity
                    row=dict(point=len(plan)-1,phase='cruise',fresh=1,mcu_us=int(t*1e6))
                    row.update({f'q{i+1}':q[i] for i in range(6)})
                    row.update({f'tau{i+1}':tau[i] for i in range(6)})
                    records.append(row)
                records.append(dict(records[-1],phase='post_hold'))
        with tempfile.TemporaryDirectory() as d:
            write_json(Path(d)/'plan.json',plan)
            with (Path(d)/'raw.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
            result=fit_friction(d,model)
            self.assertIsNotNone(result['joints'][0])
            self.assertAlmostEqual(result['joints'][0]['fc_nm'],.2,places=8)
            self.assertAlmostEqual(result['joints'][0]['b_nm_s_rad'],.04,places=8)
            self.assertTrue(all(x is None for x in result['joints'][1:]))


class LifecycleTests(unittest.TestCase):
    def test_historical_friction_transfer_at_sixty_percent(self):
        profiles=Path(__file__).parent/'profiles'
        model=Gravity(identified=profiles/'gravity_link6_full284.json')
        clock=Clock();original=FakeRobot.snapshot
        def moving(robot, *names):
            state=original(robot);state['ArmState']['q_rad'][0]=.2*(clock.t-1000)
            state['ArmState']['dq_rad_s'][0]=.2
            return state
        with tempfile.TemporaryDirectory() as d:
            a=lab.parser('gravity_friction').parse_args([])
            a.output=str(Path(d)/'run');a.seconds=.06;a.friction_scale=.6;a.gravity_scale=0
            a.friction=str(profiles/'friction_legacy_follower.json')
            with patch.object(lab,'Robot',FakeRobot),patch.object(lab,'time',clock),patch.object(FakeRobot,'snapshot',moving):
                lab.gravity_run(a,model)
            effort=max(c[4][0] for c in FakeRobot.instances[-1].commands)
            expected=.6*(.184829*np.tanh(.2/.1)+.012507*.2)
            self.assertAlmostEqual(effort,expected,places=4)
            self.assertEqual(read_json(Path(a.output)/'summary.json')['friction_scale'],.6)

    def test_resume_skips_completed_and_rejects_changed_parameters(self):
        with tempfile.TemporaryDirectory() as d:
            a=lab.parser('gravity_capture').parse_args([])
            a.output=str(Path(d)/'run');a.hz=1000;a.transition=.01;a.settle=.01;a.sample=.05
            plan=Path(d)/'plan.json';write_json(plan,{'poses_deg':[[0]*6]});a.plan=str(plan)
            with patch.object(lab,'Robot',FakeRobot),patch.object(lab,'time',Clock()):lab.gravity_capture(a,Gravity())
            a.resume=True
            with patch.object(lab,'Robot',FakeRobot),patch.object(lab,'time',Clock()):lab.gravity_capture(a,Gravity())
            self.assertEqual(len(FakeRobot.instances[-1].commands),0)
            self.assertEqual(len(list(Path(a.output).glob('raw*.csv'))),2)
            a.kp=[20]*6
            with self.assertRaises(ValueError):lab.Session(a,Gravity())

    def test_all_experiments_mocked(self):
        model=Gravity()
        for feature in ('gravity','impedance','gravity_capture','friction_capture','static_capture','gravity_friction'):
            with self.subTest(feature=feature),tempfile.TemporaryDirectory() as d:
                a=lab.parser(feature).parse_args([])
                a.output=str(Path(d)/'run');a.hz=1000;a.seconds=.025;a.transition=.015;a.settle=.015;a.sample=.03
                a.joints=[1];a.speeds=[20];a.span_deg=.5;a.ramp=.01;a.repeat=1;a.max_extra_nm=.002
                if feature=='gravity_capture':
                    p=Path(d)/'plan.json';write_json(p,{'poses_deg':[[0]*6,[1]*6]});a.plan=str(p)
                if feature=='impedance':a.target_deg=[1]*6
                if feature=='gravity_friction':
                    f=Path(d)/'friction.json';write_json(f,{'model_identity':model.identity,'gravity_theta':model.theta.tolist(),
                        'joints':[dict(fc_nm=.1,b_nm_s_rad=.01)]*6});a.friction=str(f)
                with patch.object(lab,'Robot',FakeRobot),patch.object(lab,'time',Clock()):
                    {'gravity':lab.gravity_run,'impedance':lab.gravity_run,'gravity_capture':lab.gravity_capture,
                     'friction_capture':lab.friction_capture,'static_capture':lab.static_capture,'gravity_friction':lab.gravity_run}[feature](a,model)
                robot=FakeRobot.instances[-1]
                self.assertTrue(robot.closed);self.assertEqual(robot.lease,0)
                self.assertTrue(robot.commands);self.assertEqual(robot.operations[-1][0],18)
                self.assertEqual(read_json(Path(a.output)/'summary.json')['status'],'complete')
                if feature=='static_capture':
                    self.assertTrue(all(x['status']=='censored' for x in read_json(Path(a.output)/'static_results.json')))

    def test_cleanup_when_send_fails(self):
        with tempfile.TemporaryDirectory() as d:
            a=lab.parser('gravity').parse_args([]);a.output=str(Path(d)/'run')
            with patch.object(lab,'Robot',FakeRobot),patch.object(FakeRobot,'send_arm',return_value=SendResult(False,0,'test failure')):
                with self.assertRaises(RuntimeError):lab.gravity_run(a,Gravity())
            self.assertTrue(FakeRobot.instances[-1].closed)
            self.assertEqual(read_json(Path(a.output)/'summary.json')['status'],'failed')

    def test_unsupported_soft_stop_firmware_rejected_before_claim_or_motion(self):
        with tempfile.TemporaryDirectory() as d:
            a=lab.parser('gravity').parse_args([]);a.output=str(Path(d)/'run')
            with patch.object(lab,'Robot',FakeRobot),patch.object(FakeRobot,'operation',return_value={'status':-2}) as op:
                with self.assertRaisesRegex(RuntimeError,'配套固件'):lab.gravity_run(a,Gravity())
            op.assert_called_once_with(19)
            self.assertEqual(FakeRobot.instances[-1].commands,[])
            self.assertTrue(FakeRobot.instances[-1].closed)


if __name__=='__main__':unittest.main(verbosity=2)

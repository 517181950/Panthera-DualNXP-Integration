import sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'source/host')]
import GELLO导纳参数 as cfg
from gello_admittance import Model,Mapping,TorqueCalibration,Admittance,fit_mapping,fit_current,TICK_RAD

class AdmittanceTests(unittest.TestCase):
    def setUp(self):
        self.model=Model(cfg.URDF,cfg.GRAVITY_BASE)
        self.doc={'urdf_sha256':self.model.sha,'ids':list(range(1,7)),'verified':[True]*6,'zero_ticks':[100,200,300,400,5000,6000],
                  'sign':[1,-1,1,-1,1,-1],'gravity_verified':True,'gravity_base':cfg.GRAVITY_BASE}
        self.mapping=Mapping(self.doc,self.model)
        self.cal=TorqueCalibration({'usable':True,'urdf_sha256':self.model.sha,'mapping_identity':self.mapping.identity,
            'nm_per_amp':[.7,-.7,.7,-.7,.7,-.7],'bias_amp':[.02]*6,'active_axes':[True]*6},self.model,self.mapping)
        self.q=np.array([.1,-.5,.6,.2,.1,-.2])
    def test_gravity_matches_potential_gradient(self):
        eps=1e-6;grad=[]
        for j in range(6):
            dq=np.eye(6)[j]*eps;grad.append((self.model.potential(self.q+dq)-self.model.potential(self.q-dq))/(2*eps))
        np.testing.assert_allclose(self.model.gravity(self.q),grad,atol=1e-8)
    def test_mapping_roundtrip_signed_multiturn(self):
        ticks=self.mapping.ticks(self.q);self.assertGreater(ticks[4],4095)
        np.testing.assert_allclose(self.mapping.q(ticks),self.q,atol=TICK_RAD/2)
    def test_mapping_requires_each_axis_and_gravity_confirmation(self):
        for change in ({'verified':[False]*6},{'gravity_verified':False},{'urdf_sha256':'bad'}):
            with self.assertRaises(ValueError):Mapping(dict(self.doc,**change),self.model)
    def test_pose_fit_recovers_unknown_offsets_and_signs(self):
        poses=[np.zeros(6),np.array([.3,-.3,.3,.3,.3,.3])]
        samples=[{'ticks':self.mapping.ticks(q).tolist(),'q_rad':q.tolist(),'modes':[0]*6,'urdf_sha256':self.model.sha} for q in poses]
        fitted=fit_mapping(samples,self.model,True)
        self.assertEqual(fitted['verified'],[True]*6);self.assertEqual(fitted['sign'],self.doc['sign'])
        np.testing.assert_allclose(fitted['zero_ticks'],self.doc['zero_ticks'],atol=.5)
    def test_gravity_balanced_current_no_drift(self):
        current=(self.model.gravity(self.q)/self.cal.k+self.cal.bias)*1000
        c=Admittance(self.model,self.mapping,self.cal,cfg,self.q,current)
        for _ in range(200):target,_=c.update(self.q,current,.02)
        np.testing.assert_allclose(target,self.q,atol=1e-12)
    def test_external_force_moves_correct_direction_even_negative_current_sign(self):
        ext=np.array([.2,-.2,.2,-.2,.2,-.2]);current=((self.model.gravity(self.q)-ext)/self.cal.k+self.cal.bias)*1000
        c=Admittance(self.model,self.mapping,self.cal,cfg,self.q,current);target,_=c.update(self.q,current,.02)
        np.testing.assert_array_equal(np.sign(target-self.q),np.sign(ext))
    def test_position_offset_and_velocity_bounded_under_large_force(self):
        c=Admittance(self.model,self.mapping,self.cal,cfg,self.q,[10000]*6)
        for _ in range(1000):target,_=c.update(self.q,[10000]*6,.02)
        self.assertTrue(np.all(abs(target-self.q)<=np.deg2rad(3)+1e-12))
        self.assertTrue(np.all(abs(c.velocity)<=np.deg2rad(10)+1e-12))
    def test_invalid_dt_rejected_without_motion(self):
        c=Admittance(self.model,self.mapping,self.cal,cfg,self.q,[0]*6)
        for dt in (0,-1,.2,float('nan')):
            with self.assertRaises(ValueError):c.update(self.q,[0]*6,dt)
        np.testing.assert_equal(c.target,self.q)
    def test_calibration_from_varied_static_poses(self):
        rng=np.random.default_rng(7);samples=[]
        for _ in range(50):
            q=rng.uniform(self.mapping.lower,self.mapping.upper);current=(self.model.gravity(q)/self.cal.k+self.cal.bias)*1000
            samples.append({'q_rad':q.tolist(),'current_ma':current.tolist(),'mapping_identity':self.mapping.identity,'no_external_force':True})
        doc=fit_current(samples,self.model,self.mapping);active=np.array(doc['active_axes'])
        self.assertFalse(active[0]);self.assertTrue(active[1]);self.assertTrue(active[2])
        np.testing.assert_allclose(np.array(doc['nm_per_amp'])[active],self.cal.k[active],atol=1e-8)
    def test_simulated_position_servo_supports_gravity_and_yields_to_push(self):
        q=self.q.copy();qd=np.zeros(6);target=q.copy()
        c=Admittance(self.model,self.mapping,self.cal,cfg,q,self.cal.bias*1000)
        initial=q.copy();before_push=None
        for k in range(1600):
            motor=np.clip(20*(target-q)-.5*qd,-1.2,1.2)
            current=(motor/self.cal.k+self.cal.bias)*1000
            target,_=c.update(q,current,.005)
            force=np.zeros(6)
            if 800<=k<1200:force[1]=.1
            accel=(motor+force-self.model.gravity(q)-.1*qd)/.05
            qd+=accel*.005;q+=qd*.005
            if k==799:
                self.assertLess(np.max(abs(q-initial)),.05)
                self.assertLess(np.max(abs(qd)),.01)
                before_push=q.copy()
        self.assertGreater(q[1]-before_push[1],.03)
        self.assertLess(np.max(abs(qd)),.02)

    def test_wrong_sign_or_no_external_force_data_not_silently_used(self):
        with self.assertRaises(ValueError):TorqueCalibration({'usable':False},self.model,self.mapping)
        with self.assertRaises(ValueError):fit_current([],self.model,self.mapping)

class HardwareBoundaryTests(unittest.TestCase):
    def test_unverified_mapping_never_opens_serial(self):
        from unittest.mock import patch
        import gello_admittance_entry as entry
        with patch.object(entry.cfg,'MAPPING',ROOT/'logs/__nonexistent_mapping__.json'),patch.object(entry,'PositionBus') as bus:
            with self.assertRaises(ValueError):entry.run('admittance')
            bus.assert_not_called()
    def test_shutdown_attempts_all_axes_even_when_one_disable_fails(self):
        from gello_admittance_entry import PositionBus
        class Fake:
            node=1
            def __init__(self):self.attempts=[];self.closed=False
            def disable(self):
                self.attempts.append(self.node)
                if self.node==3:raise RuntimeError('injected failure')
            def integer(self,*args,**kwargs):return 1 if self.node==3 else 0
            def close(self):self.closed=True
        fake=Fake();bus=PositionBus(ROOT);bus.b=fake;bus.changed=True
        with self.assertRaises(RuntimeError):bus.close()
        self.assertEqual(fake.attempts,list(range(1,8)));self.assertTrue(fake.closed)

if __name__=='__main__':unittest.main()

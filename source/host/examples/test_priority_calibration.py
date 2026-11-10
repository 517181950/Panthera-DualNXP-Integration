"""Offline checks for weighted priority capture/identification."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import arm_runtime
import numpy as np
import tempfile
import unittest
import runpy
from unittest.mock import patch
from arm_dynamics import Gravity,read_json,write_json
from arm_lab import parser
from priority_gravity_fit import solve,group_weights,main,metrics


class PriorityTests(unittest.TestCase):
    def test_j1_torque_and_regressor_excluded(self):
        rng=np.random.default_rng(41)
        A=rng.normal(size=(20,6,24));prior=np.zeros(24)
        truth=rng.normal(size=24);tau=A@truth;w=np.full(20,1/20)
        groups=np.array([0,1,2,0,1,2])
        for scope in ('link6','all'):
            before,rank,_=solve(A,tau,prior,w,scope)
            score=metrics(A,tau,before,w,14,groups)
            changed=A.copy();changed[:,0,:]*=10000
            noisy=tau.copy();noisy[:,0]+=1e9
            after,newrank,_=solve(changed,noisy,prior,w,scope)
            np.testing.assert_allclose(before,after,atol=1e-12)
            self.assertEqual(rank,newrank)
            self.assertEqual(score,metrics(changed,noisy,after,w,14,groups))
            self.assertEqual(score['joint_ids'],[2,3,4,5,6])

    def test_capture_targets_gains_and_timing(self):
        defaults=runpy.run_path(str(Path(__file__).parent/'11_priority-pose-capture.py'))['DEFAULTS']
        p=parser('gravity_capture');p.set_defaults(**defaults);a=p.parse_args([])
        q=read_json(a.plan)['poses_deg']
        self.assertEqual(len(q),9);self.assertEqual(q[:3],q[3:6]);self.assertEqual(q[:3],q[6:])
        self.assertEqual(q[0],[-43.7,75.4,71.3,-50.8,-41.2,-2.6])
        self.assertEqual(a.kp,[24.48,61.2,61.2,12.24,12.24,12.24])
        self.assertEqual(9*(a.transition+a.settle+a.sample+a.post_hold),153)
        self.assertAlmostEqual(Gravity(identified=a.identified).theta[20],.3936301449807948)

    def test_group_weight_is_balanced_when_repeat_missing(self):
        groups=np.array([0,0,0,1,1,2,2,2]);w=group_weights(groups,.9)
        self.assertAlmostEqual(w.sum(),.9)
        for g in range(3):self.assertAlmostEqual(w[groups==g].sum(),.3)

    def test_offline_comparison_and_urdf_exports(self):
        model=Gravity();truth=Gravity(payload_mass=.1,payload_com=[.03,.02,.01])
        oldq=np.random.default_rng(10).uniform(-1,1,(30,6));newq=oldq[:3].tolist()*3
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for name,q in [('old',oldq),('new',newq)]:
                folder=root/name;folder.mkdir()
                write_json(folder/'metadata.json',dict(arguments=dict(urdf=str(model.path),payload_mass=0.,payload_com=[0,0,0],gravity=[0,0,-9.81],feature='gravity_capture'),model_identity=model.identity,gravity_theta=model.theta.tolist()))
                write_json(folder/'points.json',[dict(point=i,accepted=True,q=list(x),tau=truth(x).tolist()) for i,x in enumerate(q)])
                write_json(folder/'plan.json',dict(poses_deg=np.rad2deg(q).tolist()))
            args=['priority-fit',str(root/'new'),'--original',str(root/'old'),'--output',str(root/'result')]
            with patch.object(sys,'argv',args):self.assertEqual(main(),0)
            report=read_json(root/'result/comparison.json')
            for scope in ('link6','all'):
                result=report['models'][scope]
                self.assertTrue(result['usable'])
                self.assertLess(result['evaluation']['weighted_rmse_nm'],1e-9)
                self.assertLess(result['evaluation']['held_repeat_rmse_nm'],1e-9)
                jsonmodel=Gravity(identified=root/f'result/{scope}_identified.json')
                urdfmodel=Gravity(urdf=root/f'result/{scope}_identified.urdf')
                np.testing.assert_allclose(jsonmodel(oldq[0]),urdfmodel(oldq[0]),atol=1e-9)
            np.testing.assert_array_equal(report['models']['link6']['theta'][:20],model.theta[:20])


if __name__=='__main__':unittest.main(verbosity=2)

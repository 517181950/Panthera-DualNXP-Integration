"""Offline comparison/mapping and arm command-isolation checks."""
from types import SimpleNamespace
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import gello_arm_compare as demo
import numpy as np
import arm_lab
from arm_dynamics import Gravity
from test_hand_slider_demo import HandRobot
from gello_compare_driver.protocol import build_position_velocity_request,parse_position_velocity_response,checksum


class Tests(unittest.TestCase):
    def test_mujoco_mapping_matches_original_with_offsets(self):
        import sys
        import copy
        sys.path.insert(0,str(demo.MUJOCO_CONFIG.parent))
        from gello import load_config,map_group_to_targets
        config=demo.read_json(demo.MUJOCO_CONFIG)
        comparison=demo.Comparison(config)
        original=load_config(demo.MUJOCO_CONFIG)
        for group_id,name in ((0,'left'),(1,'right')):
            group=SimpleNamespace(raw_positions=[2048]*7,unwrapped_ticks=[2048]*7,
                raw_velocities=[10]*7,servo_captured_at=[10.]*7,
                arm_radians=[.1,-.2,.3,-.4,.5,-.6],data_valid=True,zeroed=True,gripper_normalized=.5)
            expected=map_group_to_targets(group,getattr(original,name),[0.]*6)
            result=comparison.calculate(group_id,demo.gello_values(group,10.01),None)
            np.testing.assert_allclose(result['g_mujoco'],np.rad2deg(expected.joints))
            axes=config[name]['axes']
            np.testing.assert_allclose(result['g_v'],np.array(demo.SERVO_SIGNS[:6])*43.92*[a['sign']*a['scale'] for a in axes])
        modified=copy.deepcopy(config);modified['right']['axes'][0]['offset_rad']=.25
        with_offset=demo.Comparison(modified).calculate(1,demo.gello_values(group,10.01),None)
        self.assertAlmostEqual(with_offset['g_mujoco'][0],result['g_mujoco'][0]+np.rad2deg(.25))

    def test_gui_switches_groups_with_native_bool_values(self):
        import dearpygui.dearpygui as dpg
        comparison=demo.Comparison();context=dict(group=0)
        args=SimpleNamespace(group=0,port='COM5',baud=1000000,velocity_rpm_per_unit=.732)
        try:
            demo.build_gui(dpg,args,comparison,context)
            comparison.flips[1][2]=-1
            callback=dpg.get_item_configuration('group_select')['callback']
            for group in (1,0,1,0,1):
                callback('group_select',demo.GROUP_LABELS[group])
                self.assertEqual(context['group'],group)
                for i in range(6):self.assertEqual(dpg.get_value(f'flip{i}'),bool(comparison.flips[group][i]<0))
            self.assertTrue(dpg.does_item_exist('gripper0'))
            self.assertTrue(dpg.does_item_exist('gripper1'))
        finally:dpg.destroy_context()

    def test_read_packet_and_signed_velocity(self):
        request=build_position_velocity_request(1)
        self.assertEqual(request,bytes.fromhex('ffff0104023804bc'))
        data=bytearray([255,255,1,6,0,0,8,7,128])
        data.append(checksum(data[2:]))
        reply=parse_position_velocity_response(bytes(data),expected_id=1)
        self.assertEqual(reply.raw_position,2048);self.assertEqual(reply.raw_velocity,-7)

    def test_missing_stale_data_and_direction_conversion(self):
        group=SimpleNamespace(raw_positions=[2048]*7,unwrapped_ticks=[4097]*7,
            raw_velocities=[10]*7,servo_captured_at=[10.]*7)
        values=demo.gello_values(group,10.01)
        np.testing.assert_allclose(values['raw_deg'],180)
        np.testing.assert_allclose(values['velocity_deg_s'],np.array([-1,-1,1,-1,1,-1])*43.92)
        self.assertTrue(np.isnan(demo.gello_values(group,10.3)['velocity_deg_s']).all())
        group.raw_velocities[2]=None
        self.assertTrue(np.isnan(demo.gello_values(group,10.01)['velocity_deg_s'][2]))
        compare=demo.Comparison();arm=dict(q=[0.]*6,dq=[.1]*6)
        self.assertTrue(compare.capture(0,values,arm))
        group.unwrapped_ticks=[4197]*7
        newer=demo.gello_values(group,10.01)
        result=compare.calculate(0,newer,arm)
        self.assertAlmostEqual(result['g_delta'][0],-100*360/4096)
        compare.flips[0][0]=-1
        flipped=compare.calculate(0,newer,arm)
        self.assertAlmostEqual(flipped['g_delta'][0],100*360/4096)
        self.assertAlmostEqual(flipped['g_v'][0],43.92)
        self.assertFalse(compare.capture(1,demo.gello_values(None,10.),arm))

    def test_arm_worker_sends_only_gravity_not_gello_targets(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(arm_lab,'Robot',HandRobot):
            args=arm_lab.parser('gravity').parse_args([])
            args.identified=str(demo.HERE/'profiles/gravity_selected.json');args.output=str(Path(tmp)/'arm')
            view=demo.ArmView();stop=threading.Event()
            worker=threading.Thread(target=demo.run_arm,args=(args,view,stop));worker.start()
            deadline=time.perf_counter()+2
            while view.snapshot()['sample'] is None and time.perf_counter()<deadline:time.sleep(.005)
            stop.set();worker.join(3)
            self.assertFalse(worker.is_alive());self.assertIsNone(view.snapshot().get('error'))
            robot=HandRobot.instances[-1];self.assertTrue(robot.closed)
            self.assertFalse(robot.hand_commands)
            expected=Gravity(identified=args.identified)(np.zeros(6))
            self.assertGreater(len(robot.commands),0)
            self.assertEqual(robot.operations[-1][0],18)
            for q,v,kp,kd,tau in robot.commands:
                for vector in (q,v,kp,kd):np.testing.assert_array_equal(vector,np.zeros(6))
                np.testing.assert_allclose(tau,expected)


if __name__=='__main__':unittest.main(verbosity=2)

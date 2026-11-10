"""Offline only: no serial device is opened."""
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch
import servo_travel_calibration as m


class FakeBus:
    def __init__(self):
        self.reg = {13:80,14:100,15:40,19:4,28:500,33:4,38:200,
                    40:1,41:0,42:1000,44:10,46:100,48:1000,
                    55:1,8:1,30:1,9:0,11:4095,18:52}
        self.identity = m.EXPECTED_IDENTITY
        self.f = m.Feedback(1000,0,0,7.4,30,0)
        self.events = []
        self.writes_allowed = False
        self.fail_off = False
    def read(self,a,n,**kw): return self.identity
    def integer(self,a,n,**kw): return self.reg[a]
    def feedback(self): return self.f
    def write(self,a,v,**kw):
        assert self.writes_allowed
        assert a in m.WIDTHS
        if a != 40:
            assert self.reg[40] == 0 or a == 42
        self.reg[a] = v
        self.events.append((a,v))
    def write_motion(self,position,settings):
        self.write(42,position)
    def disable(self):
        if self.fail_off: raise m.CalibrationError('off failed')
        self.write(40,0)


class Tests(unittest.TestCase):
    def test_port_discovery(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            (root/'usb-NXP_debug-if02').touch()
            with self.assertRaises(m.CalibrationError):m.resolve_port(directory=root)
            first=root/'usb-1a86_USB_Single_Serial_NEW-if00';first.touch()
            self.assertEqual(m.resolve_port(directory=root),str(first))
            (root/'usb-1a86_USB_Single_Serial_OTHER-if00').touch()
            with self.assertRaises(m.CalibrationError):m.resolve_port(directory=root)
            self.assertEqual(m.resolve_port(str(first),root),str(first))
            with self.assertRaises(m.CalibrationError):m.resolve_port(str(root/'missing'),root)

    def test_voltage_and_fault_preflight_no_writes(self):
        for volts,status in [(12.1,0),(7.4,1),(3.9,0)]:
            b=FakeBus();b.f.voltage_v=volts;b.f.status=status
            with self.assertRaises(m.CalibrationError):m.calibrate(b,m.Settings())
            self.assertEqual(b.events,[])
    def test_negative_position_preflight(self):
        b=FakeBus();b.f.position=-802
        f,lo,hi,original=m.preflight(b,m.Settings())
        self.assertEqual(f.position,-802)
        self.assertEqual((lo,hi),(-32767,32767))
        self.assertEqual(b.events,[])

    def test_signed_position_wire_encoding(self):
        for value in [-32767,-802,-1,0,1,4096,32767]:
            b=object.__new__(m.Bus);b.writes_allowed=True
            wire=m.encode_position(value)
            with patch.object(b,'exchange',return_value=(b'',0)) as exchange, patch.object(b,'integer',return_value=wire):
                b.write(42,value)
                self.assertEqual(exchange.call_args.args[1],bytes([42])+wire.to_bytes(2,'little'))
            self.assertEqual(m.signed(wire),value)
        for value in [-32768,32768]:
            with self.assertRaises(m.CalibrationError):m.encode_position(value)

    def test_coordinate_reset_uses_new_reference(self):
        b=FakeBus();b.f.position=-803;original=b.reg.copy();prepare=m.prepare
        def changed(*args):
            prepare(*args)
            b.f.position=0
        from dataclasses import replace
        b.feedback=lambda:replace(b.f)
        with patch.object(m,'prepare',side_effect=changed), patch.object(m,'seek',return_value={'raw_ticks':100}) as seek:
            result=m.calibrate(b,m.Settings(),zero_mode='current')
        self.assertEqual(result['coordinate_change_ticks'],803)
        self.assertEqual(result['calibration']['zero_raw_ticks'],0)
        self.assertEqual(result['calibration']['travel_ticks'],100)
        self.assertEqual(seek.call_args.kwargs['expected_start'],0)
        for a in m.RESTORE:self.assertEqual(b.reg[a],original[a])

    def test_unstable_reference_rejected(self):
        from dataclasses import replace
        b=FakeBus();b.reg[40]=0;r={}
        positions=iter([0,1,2,3,4])
        b.feedback=lambda:replace(b.f,position=next(positions))
        with self.assertRaisesRegex(m.CalibrationError,'未稳定'):
            m.establish_reference(b,m.Settings(),r,sleep=lambda dt:None)
        self.assertEqual(len(r['reference_samples']),5)
        self.assertNotIn((40,1),b.events)

    def test_drift_before_enable_rejected(self):
        b=FakeBus();b.writes_allowed=True;b.reg[40]=0
        with self.assertRaisesRegex(m.CalibrationError,'使能前位置'):
            m.seek(b,m.Settings(),1,-32767,32767,lambda row:None,expected_start=990)
        self.assertEqual(b.events,[])

    def test_custom_limits_and_single_angle_rejected(self):
        for address,value in [(9,100),(18,36)]:
            b=FakeBus();b.reg[address]=value
            with self.assertRaises(m.CalibrationError):m.calibrate(b,m.Settings())
            self.assertEqual(b.events,[])

    def test_identity_and_lock_no_writes(self):
        for field,value in [('identity',b'12345'),('lock',0)]:
            b=FakeBus()
            if field=='identity':b.identity=value
            else:b.reg[55]=value
            with self.assertRaises(m.CalibrationError):m.calibrate(b,m.Settings())
            self.assertEqual(b.events,[])
    def test_contact_both_directions(self):
        for direction in [-1,1]:
            d=m.ContactDetector(m.Settings(),1000,direction,0)
            f=m.Feedback(1000+direction*20,0,110,7.4,30,0)
            hits=[d.update(f,f.position+direction*10,i*.02) for i in range(6)]
            self.assertTrue(hits[-1])
    def test_initial_stall_is_not_endpoint(self):
        d=m.ContactDetector(m.Settings(),1000,1,0)
        f=m.Feedback(1000,0,110,7.4,30,0)
        with self.assertRaises(m.CalibrationError):
            for i in range(8):d.update(f,1010,i*.02)
    def test_no_current_stall_rejected(self):
        d=m.ContactDetector(m.Settings(),1000,1,0)
        f=m.Feedback(1000,0,0,7.4,30,0)
        with self.assertRaises(m.CalibrationError):
            for i in range(30):self.assertFalse(d.update(f,1010,i*.02))
    def test_faults_and_timing(self):
        for position,current,temperature,status,now in [(1000,210,30,0,.02),(1000,0,50,0,.02),(1000,0,30,8,.02),(1100,0,30,0,.02),(990,0,30,0,.02),(1000,0,30,0,.11)]:
            d=m.ContactDetector(m.Settings(),1000,1,0)
            with self.assertRaises(m.CalibrationError):
                d.update(m.Feedback(position,0,current,7.4,temperature,status),1012,now)
    def test_success_restore_off_and_conversion(self):
        for direction in [-1,1]:
            for zero_mode in ['lower','current']:
                b=FakeBus();original=b.reg.copy()
                ends=[{'raw_ticks':1000-direction*100},{'raw_ticks':1000+direction*100}]
                if zero_mode=='current':ends=ends[1:]
                with patch.object(m,'seek',side_effect=ends):
                    r=m.calibrate(b,m.Settings(direction=direction),zero_mode)
                self.assertEqual(r['status'],'completed_candidate')
                self.assertEqual(r['calibration']['travel_ticks'],200 if zero_mode=='lower' else 100)
                self.assertEqual(b.reg[40],0)
                for a in m.RESTORE:self.assertEqual(b.reg[a],original[a])
                self.assertNotIn((40,1),b.events)
    def test_exception_cleanup_including_interrupt(self):
        for exc in [m.CalibrationError('timeout'),KeyboardInterrupt()]:
            b=FakeBus();r={}
            with patch.object(m,'seek',side_effect=exc):
                with self.assertRaises(type(exc)):m.calibrate(b,m.Settings(),report=r)
            self.assertEqual(b.reg[40],0)
            self.assertNotIn('calibration',r)
            self.assertTrue(r['cleanup']['torque_disabled_confirmed'])
    def test_unconfirmed_off_never_restores(self):
        b=FakeBus();r={}
        def fail(*args,**kwargs):b.fail_off=True;raise m.CalibrationError('link lost')
        with patch.object(m,'seek',side_effect=fail):
            with self.assertRaises(m.CalibrationError):m.calibrate(b,m.Settings(),report=r)
        self.assertEqual(b.reg[48],100)
        self.assertFalse(r['cleanup']['torque_disabled_confirmed'])
    def test_seek_stops_before_endpoint_log(self):
        b=FakeBus();b.writes_allowed=True;b.reg[40]=0
        t=[0.];rows=[]
        def feedback():
            target=b.reg[42]
            b.f.position=min(target,1020)
            b.f.current_ma=110 if target-b.f.position>=8 else 0
            return b.f
        b.feedback=feedback
        def emit(row):rows.append((row,b.reg[40]))
        result=m.seek(b,m.Settings(),1,0,4095,emit,lambda:t[0],lambda dt:t.__setitem__(0,t[0]+dt))
        self.assertEqual(result['raw_ticks'],1020)
        self.assertEqual(rows[-1][1],0)
        self.assertTrue(result['torque_disabled_confirmed'])
    def test_signed_search_crosses_zero_both_directions(self):
        real_seek=m.seek
        from dataclasses import replace
        for start,stop,direction in [(-802,-702,1),(-50,50,1),(50,-50,-1),(4090,4190,1)]:
            b=FakeBus();b.f.position=start;b.reg[42]=start
            original=b.reg.copy();t=[0.]
            def feedback():
                if b.reg[40]:
                    target=b.reg[42]
                    b.f.position=min(target,stop) if direction==1 else max(target,stop)
                    b.f.current_ma=110 if direction*(target-b.f.position)>=8 else 0
                else:b.f.current_ma=0
                return replace(b.f)
            b.reg[40]=0;b.feedback=feedback
            def seek(*args,**kwargs):
                return real_seek(*args,**kwargs,clock=lambda:t[0],sleep=lambda dt:t.__setitem__(0,t[0]+dt))
            with patch.object(m,'seek',side_effect=seek):
                report=m.calibrate(b,m.Settings(direction=direction),zero_mode='current')
            self.assertEqual(report['calibration']['zero_raw_ticks'],start)
            self.assertEqual(report['calibration']['upper_raw_ticks'],stop)
            self.assertEqual(report['calibration']['travel_ticks'],100)
            self.assertEqual(b.reg[40],0)
            for a in m.RESTORE:self.assertEqual(b.reg[a],original[a])
            enabled=b.events.index((40,1))
            self.assertIn((9,0),b.events[:enabled])
            self.assertIn((11,0),b.events[:enabled])

    def test_stationary_target_is_not_repeated(self):
        b=FakeBus();b.reg[40]=0;b.writes_allowed=True;t=[0.]
        with self.assertRaisesRegex(m.CalibrationError,'长时间无有效位移'):
            m.seek(b,m.Settings(),1,-32767,32767,lambda row:None,
                   clock=lambda:t[0],sleep=lambda dt:t.__setitem__(0,t[0]+dt))
        goals=[v for a,v in b.events if a==42]
        self.assertEqual(goals,[1000,1012])
        self.assertEqual(b.reg[40],0)

    def test_software_boundary_not_endpoint(self):
        b=FakeBus();b.writes_allowed=True;b.reg[40]=0;t=[0.]
        with self.assertRaisesRegex(m.CalibrationError,'软件角度边界'):
            m.seek(b,m.Settings(),1,0,1002,lambda row:None,lambda:t[0],lambda dt:t.__setitem__(0,t[0]+dt))
    def test_fd_motion_block(self):
        b=object.__new__(m.Bus);b.writes_allowed=True
        # -802 => 0x8322 sign-magnitude; floor(150/6.5)=23; speed=1.
        data=bytes.fromhex('01 22 83 17 00 01 00')
        with patch.object(b,'exchange',return_value=(b'',0)) as exchange, patch.object(b,'read',return_value=data):
            b.write_motion(-802,m.Settings())
            exchange.assert_called_once_with(3,bytes([41])+data,0)
        with patch.object(b,'exchange',return_value=(b'',0)), patch.object(b,'read',return_value=b'bad'):
            with self.assertRaises(m.CalibrationError):b.write_motion(-802,m.Settings())
        b.writes_allowed=False
        with self.assertRaises(m.CalibrationError):b.write_motion(0,m.Settings())

    def test_bus_write_guard(self):
        b=object.__new__(m.Bus);b.writes_allowed=False
        with self.assertRaises(m.CalibrationError):b.write(40,1)
        b.writes_allowed=True
        for a in [5,31,55]:
            with self.assertRaises(m.CalibrationError):b.write(a,0)
    def test_readonly_feedback_decode(self):
        data=bytes([25,7,0,0,0,0,121,39,0,1,0,25,7,0,0])
        f=m.Feedback.decode(data,1)
        self.assertEqual((f.position,f.voltage_v,f.status),(1817,12.1,1))


if __name__=='__main__':unittest.main()

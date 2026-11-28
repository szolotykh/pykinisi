"""IMU-only ordinary API contracts, framing, gates and explicit lifecycle."""
import math
import struct
import unittest
from pykinisi import (KinisiCommands, ImuTrackingAction, ImuTrackingVectorKind,
    SET_IMU_TRACKING, ERROR, ProtocolError, ControllerError, ErrorCode)
from test_generator import recording_client
import test_sensors
from test_controller import frame

class ImuTrackingTests(unittest.TestCase):
    connect = test_sensors.SensorSessionTests.connect

    def test_schema_family_and_profile_capability_validation(self):
        from copy import deepcopy
        import json
        from test_generator import generator, ROOT, load_module
        schema = json.loads((ROOT / 'tools/commands.json').read_text(encoding='utf-8'))
        generator.validate_imu_tracking(schema)
        for mutate in (lambda s: s['commands'].remove(next(c for c in s['commands'] if c['command'] == 'GET_IMU_TRACKING_CONFIGURATION')), lambda s: s['imu_tracking_states'][0].update(id=True)):
            broken=deepcopy(schema); mutate(broken)
            with self.assertRaises(ValueError): generator.validate_imu_tracking(broken)
        updater=load_module('tracking_profile_updater',ROOT / 'tools/update-board-profiles.py')
        bundle=json.loads((ROOT / 'pykinisi/board_profiles.json').read_text(encoding='utf-8'))
        for board in bundle['boards']:
            self.assertEqual(board['capabilities']['imu_tracking'],board['board']['revision']=='0.4.0')
        bundle['boards'][-1]['capabilities']['imu_tracking']=1
        with self.assertRaises(ValueError):updater.generate(bundle)

    def test_wire_and_strict_validation(self):
        c = recording_client({'KinisiCommands': KinisiCommands}); c.reply = b''
        for action in (ImuTrackingAction.START, ImuTrackingAction.RESET, ImuTrackingAction.RECALIBRATE):
            c.set_imu_tracking(action, 1, math.pi)
            self.assertEqual(c.calls[-1], (0x63, struct.pack('<BBd', action, 1, math.pi), 0))
        c.set_imu_tracking(ImuTrackingAction.STOP, 0, 0)
        before = len(c.calls)
        for args in ((0,0,0),(1,1,0),(1,0,1),(4,1,0),(True,1,0),(0,True,0),(0,1,True),(0,1,math.nan),(0,1,math.inf),(0,1,4),(0,1,'0')):
            with self.subTest(args=args), self.assertRaises(ValueError): c.set_imu_tracking(*args)
        for kind in (-1,3,True,'0',.5):
            with self.assertRaises(ValueError): c.get_imu_tracking_vector(kind)
        self.assertEqual(len(c.calls), before)
        c.reply = struct.pack('<BBBBIQIII',2,0,3,0,42,2**60+7,500,10,20)
        s=c.get_imu_tracking_status()
        self.assertEqual((s.timestamp_us,s.elapsed_us,s.accel_age_us,s.gyro_age_us),(2**60+7,500,10,20))
        self.assertEqual(c.calls[-1],(0x64,b'',28))
        c.reply=struct.pack('<Iddd',42,1,-2,.5)
        self.assertEqual(c.get_imu_tracking_vector(ImuTrackingVectorKind.POSE).x,1)
        self.assertEqual(c.calls[-1],(0x65,b'\0',28))
        c.reply=struct.pack('<dI',math.pi,5000000)
        self.assertEqual(c.get_imu_tracking_configuration().window_us,5000000)
        self.assertEqual(c.calls[-1],(0x66,b'',12))
        for data,method in ((bytes([5])+bytes(27),c.get_imu_tracking_status),(bytes([0,0,0,1])+bytes(24),c.get_imu_tracking_status),(bytes(27),c.get_imu_tracking_status),(struct.pack('<Iddd',1,math.nan,0,0),lambda:c.get_imu_tracking_vector(0)),(struct.pack('<dI',0,0),c.get_imu_tracking_configuration)):
            c.reply=data
            with self.assertRaises(ValueError): method()

    def test_gates_reconnect_and_device_error_without_actions(self):
        for minor,patch,protocol,ok in ((4,0,20,True),(4,0,21,True),(4,0,19,False),(4,1,20,False),(3,0,20,False),(3,1,20,False),(99,0,20,False)):
            c,transport,board=self.connect(minor=minor,board_patch=patch,protocol=protocol)
            self.assertEqual(c.supports_imu_tracking(),ok)
            self.assertFalse(any(r[1]==SET_IMU_TRACKING for r in transport.frames))
            count=len(transport.frames)
            if not ok:
                for call in (lambda:c.set_imu_tracking(0,1,0),c.get_imu_tracking_status,lambda:c.get_imu_tracking_vector(0),c.get_imu_tracking_configuration):
                    with self.assertRaises(ProtocolError):call()
                self.assertEqual(len(transport.frames),count)
            else:
                board.command_hook=lambda command,mid,payload:transport.inject(frame(ERROR,mid,bytes([command,ErrorCode.INVALID_ARGUMENT])))
                with self.assertRaises(ControllerError):c.set_imu_tracking(0,1,0)
                self.assertEqual(len(transport.frames),count+1)
                self.assertTrue(c.ready)
            c.disconnect()
            c,transport,_=self.connect(protocol=20,client=c)
            self.assertFalse(any(r[1]==SET_IMU_TRACKING for r in transport.frames))
            c.disconnect()

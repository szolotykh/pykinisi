"""Fusion experiment codecs and exact profile gates through ordinary SDK calls."""
import math
import struct
import unittest
from unittest.mock import patch
from pykinisi import (KinisiCommands, FusionMode, FusionVectorKind, SET_FUSION_EXPERIMENT,
    GET_FUSION_STATUS, GET_FUSION_VECTOR, GET_FUSION_CONFIGURATION, ERROR,
    ProtocolError, ControllerError, ErrorCode)
from test_generator import recording_client
import test_sensors
from test_controller import frame, controller_module


class FusionTests(unittest.TestCase):
    connect = test_sensors.SensorSessionTests.connect

    def test_wire_size_precision_and_validation(self):
        c = recording_client({'KinisiCommands': KinisiCommands})
        c.reply = b''
        c.set_fusion_experiment(FusionMode.FUSED, -0.5)
        self.assertEqual(c.calls[-1], (0x5f, struct.pack('<Bd', 2, -0.5), 0))
        before = len(c.calls)
        for mode, yaw in ((3, 0), (-1, 0), (True, 0), ('2', 0), (2, True),
                          (2, math.inf), (2, math.nan), (2, 4), (2, '0')):
            with self.subTest(mode=mode, yaw=yaw), self.assertRaises(ValueError):
                c.set_fusion_experiment(mode, yaw)
        for kind in (-1, 5, True, '0', 1.5):
            with self.assertRaises(ValueError): c.get_fusion_vector(kind)
        self.assertEqual(len(c.calls), before)
        c.reply = struct.pack('<BBBBIQIII', 2, 2, 0, 3, 42, 2**60 + 7, 10, 20, 30)
        status = c.get_fusion_status()
        self.assertEqual(status.timestamp_us, 2**60 + 7)
        self.assertEqual((status.sequence, status.encoder_age_us), (42, 30))
        self.assertEqual(c.calls[-1], (0x60, b'', 28))
        c.reply = struct.pack('<Iddd', 42, 1.25, -2, .5)
        vector = c.get_fusion_vector(FusionVectorKind.FUSED_POSE)
        self.assertEqual((vector.sequence, vector.x, vector.y, vector.z), (42, 1.25, -2, .5))
        self.assertEqual(c.calls[-1], (0x61, b'\x02', 28))
        c.reply = struct.pack('<Bd', 2, -.5)
        self.assertEqual(c.get_fusion_configuration().mounting_yaw_rad, -.5)
        for reply, method in ((bytes([3])+bytes(27), c.get_fusion_status),
                (struct.pack('<Iddd', 1, math.nan, 0, 0), lambda: c.get_fusion_vector(0)),
                (struct.pack('<Bd', 2, math.inf), c.get_fusion_configuration),
                (bytes(27), c.get_fusion_status)):
            c.reply = reply
            with self.assertRaises(ValueError): method()

    def test_exact_gate_defaults_errors_and_reconnect(self):
        for minor, board_patch, protocol, supported in ((4,0,19,True), (4,0,20,True),
                (4,0,18,False), (4,1,19,False), (3,0,19,False), (3,1,19,False), (99,0,19,False)):
            with self.subTest(minor=minor, patch=board_patch, protocol=protocol):
                c, transport, board = self.connect(minor=minor, board_patch=board_patch, protocol=protocol)
                self.assertEqual(c.supports_fusion_experiment(), supported)
                self.assertFalse(any(raw[1] == SET_FUSION_EXPERIMENT for raw in transport.frames))
                before = len(transport.frames)
                if not supported:
                    for method in (lambda: c.set_fusion_experiment(2, 0), c.get_fusion_status,
                                   lambda: c.get_fusion_vector(0), c.get_fusion_configuration):
                        with self.assertRaises(ProtocolError): method()
                    self.assertEqual(len(transport.frames), before)
                    c.disconnect(); continue
                board.command_hook = lambda command, mid, payload: transport.inject(
                    frame(ERROR, mid, bytes([command, ErrorCode.INVALID_ARGUMENT])))
                with self.assertRaises(ControllerError) as error: c.set_fusion_experiment(2, 0)
                self.assertEqual(error.exception.error_code, ErrorCode.INVALID_ARGUMENT)
                self.assertTrue(c.ready)
                self.assertEqual(len(transport.frames), before + 1)  # No stop, retry or target.
                with patch.object(controller_module, 'get_board_profile', return_value={'capabilities': {'fusion_experiment': False}}):
                    self.assertFalse(c.supports_fusion_experiment())
                    with self.assertRaises(ProtocolError): c.get_fusion_status()
                c.disconnect()
                c, transport, _ = self.connect(protocol=19, client=c)
                self.assertFalse(any(raw[1] == SET_FUSION_EXPERIMENT for raw in transport.frames))
                c.disconnect()

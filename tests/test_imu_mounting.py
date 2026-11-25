"""Exercise the 2.23 mounting API through a READY controller and wire transport."""
import struct
import unittest
from unittest.mock import patch

from pykinisi import (
    ERROR, GET_IMU_MOUNT, SET_IMU_MOUNT, ControllerError, ErrorCode, ImuMountGeometry,
    KinisiCommands, KinisiController, ProtocolError,
)
import test_sensors
from test_controller import controller_module, frame
from test_generator import recording_client


class ImuMountingTests(unittest.TestCase):
    connect = test_sensors.SensorSessionTests.connect

    def test_install_readback_and_legacy_flag(self):
        client, transport, board = self.connect(protocol=23)
        self.assertTrue(client.supports_imu_mounting())
        raw = struct.pack('<hhhiii', 18000, -9000, -1, 10_000_000, -100_000, 1)
        before = len(transport.frames)
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid))
        client.set_imu_mount_geometry(180, -90, -.005, 10, -.1, .0000005)
        self.assertEqual(transport.frames[-1][4:], raw)
        for configured in (False, True):
            board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid, raw + bytes([configured])))
            expected = ImuMountGeometry(180, -90, -.01, 10, -.1, .000001, configured)
            self.assertEqual(client.get_imu_mount_geometry(), expected)
        self.assertEqual([p[1] for p in transport.frames[before:]], [0x6e, 0x6f, 0x6f])

    def test_limits_before_rounding_and_firmware_refusal(self):
        client, transport, board = self.connect(protocol=23)
        before = len(transport.frames)
        for i, limit in enumerate((180, 90, 180, 10, 10, 10)):
            for bad in (limit + .0000001, -limit - .0000001, float('nan'), float('inf'), True):
                args = [0] * 6
                args[i] = bad
                with self.assertRaises(ValueError):
                    client.set_imu_mount_geometry(*args)
            for bad in ((limit * (100 if i < 3 else 1_000_000)) + 1, 1.5, True):
                args[i] = bad
                with self.assertRaises(ValueError):
                    client.set_imu_mount(*args)
        self.assertEqual(len(transport.frames), before)
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(ERROR, mid, bytes((cmd, ErrorCode.SOURCE_SWITCH_UNSAFE))))
        with self.assertRaises(ControllerError) as caught:
            client.set_imu_mount_geometry(90, 0, 0, 0, -.1, 0)
        self.assertEqual(caught.exception.error_code, ErrorCode.SOURCE_SWITCH_UNSAFE)
        self.assertEqual([p[1] for p in transport.frames[before:]], [SET_IMU_MOUNT])

    def test_exact_profile_ready_and_protocol_gates(self):
        self.assertFalse(KinisiController().supports_imu_mounting())
        for minor, board_patch, protocol in ((4, 0, 22), (3, 0, 23), (4, 1, 23)):
            client, transport, _ = self.connect(minor=minor, board_patch=board_patch, protocol=protocol)
            self.assertFalse(client.supports_imu_mounting())
            before = len(transport.frames)
            for call in (client.get_imu_mount_geometry, lambda: client.set_imu_mount(0, 0, 0, 0, 0, 0)):
                with self.assertRaises(ProtocolError):
                    call()
            self.assertEqual(len(transport.frames), before)
        client, transport, _ = self.connect(protocol=23)
        before = len(transport.frames)
        profile = dict(client.board_profile)
        profile['capabilities'] = {key: value for key, value in profile['capabilities'].items()
                                   if key != 'imu_mounting'}
        with patch.object(controller_module, 'get_board_profile', return_value=profile):
            self.assertFalse(client.supports_imu_mounting())
            with self.assertRaises(ProtocolError):
                client.get_imu_mount()
        self.assertEqual(len(transport.frames), before)
        client.disconnect()
        self.assertFalse(client.supports_imu_mounting())

    def test_reject_malformed_mount_readback(self):
        client = recording_client({'KinisiCommands': KinisiCommands})
        for bad in (bytes(18), bytes(20), bytes(18) + b'\x02',
                    struct.pack('<hhhiii?', 0, 9001, 0, 0, 0, 0, True),
                    struct.pack('<hhhiii?', 0, 0, 0, 0, 0, -10_000_001, True)):
            client.reply = bad
            with self.assertRaises(ValueError):
                client.get_imu_mount()
        self.assertEqual(client.calls[-1], (GET_IMU_MOUNT, b'', 19))

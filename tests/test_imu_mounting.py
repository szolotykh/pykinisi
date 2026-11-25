"""Exercise board-center and legacy chip mounting APIs through a READY controller and wire transport."""
import struct
import unittest
from unittest.mock import patch

from pykinisi import (
    ERROR, GET_IMU_MOUNT, SET_IMU_MOUNT, GET_BOARD_MOUNT, SET_BOARD_MOUNT, BoardMountGeometry, ControllerError, ErrorCode, ImuMountGeometry,
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
                    struct.pack('<hhhiii?', 0, 0, 0, 0, 0, -10_031_099, True)):
            client.reply = bad
            with self.assertRaises(ValueError):
                client.get_imu_mount()
        self.assertEqual(client.calls[-1], (GET_IMU_MOUNT, b'', 19))

    def test_board_center_frames_and_distinct_chip_readback(self):
        client, transport, board = self.connect(protocol=24)
        self.assertTrue(client.supports_board_mounting() and client.supports_imu_mounting())
        center = struct.pack('<hhhiii', 9000, 0, 0, 0, -100_000, 0)
        chip = struct.pack('<hhhiii', 9000, 0, 0, -15_373, -102_075, 26_953)
        before = len(transport.frames)
        replies = {GET_BOARD_MOUNT: bytes(19), GET_IMU_MOUNT: chip + b'\1'}
        board.command_hook = lambda cmd, mid, data: transport.inject(frame(cmd, mid, replies.get(cmd, b'')))
        self.assertEqual(client.get_board_mount_geometry(), BoardMountGeometry(0, 0, 0, 0, 0, 0, False))
        client.set_board_mount_geometry(90, 0, 0, 0, -.1, 0)
        self.assertEqual(transport.frames[-1][4:], center)
        replies[GET_BOARD_MOUNT] = center + b'\1'
        self.assertEqual(client.get_board_mount_geometry(), BoardMountGeometry(90, 0, 0, 0, -.1, 0, True))
        self.assertEqual(client.get_imu_mount_geometry(), ImuMountGeometry(90, 0, 0, -.015373, -.102075, .026953, True))
        client.set_imu_mount_geometry(90, 0, 0, -.015373, -.102075, .026953)
        self.assertEqual(transport.frames[-1][4:], chip)
        self.assertEqual([p[1] for p in transport.frames[before:]], [0x86, 0x85, 0x86, 0x6f, 0x6e])
        board.command_hook = lambda cmd, mid, data: transport.inject(frame(ERROR, mid, bytes((cmd, ErrorCode.SOURCE_SWITCH_UNSAFE))))
        with self.assertRaises(ControllerError) as caught:
            client.set_board_mount_geometry(0, 0, 0, 0, 0, 0)
        self.assertEqual(caught.exception.error_code, ErrorCode.SOURCE_SWITCH_UNSAFE)
        self.assertEqual(len(transport.frames) - before, 6)

    def test_board_center_read_and_write_limits(self):
        client, transport, board = self.connect(protocol=24)
        before = len(transport.frames)
        for setter, invalid in ((client.set_board_mount, -10_000_001), (client.set_board_mount, True),
                                (client.set_board_mount_geometry, float('nan')), (client.set_board_mount_geometry, 10.0000001)):
            with self.assertRaises(ValueError):
                setter(0, 0, 0, 0, invalid, 0)
        self.assertEqual(len(transport.frames), before)
        board.command_hook = lambda cmd, mid, data: transport.inject(frame(cmd, mid))
        client.set_board_mount_geometry(180, -90, -.005, 10, -.0000005, 0)
        self.assertEqual(transport.frames[-1][4:], struct.pack('<hhhiii', 18000, -9000, -1, 10_000_000, -1, 0))
        raw_client = recording_client({'KinisiCommands': KinisiCommands})
        for getter in (raw_client.get_imu_mount, raw_client.get_board_mount):
            raw_client.reply = struct.pack('<hhhiiiB', 0, 0, 0, 10_031_098, -10_031_098, 0, 1)
            self.assertEqual(getter().x_um, 10_031_098)
            for reply in (bytes(18), bytes(20), bytes(18) + b'\x02',
                          struct.pack('<hhhiiiB', 0, 9001, 0, 0, 0, 0, 1),
                          struct.pack('<hhhiiiB', 0, 0, 0, -10_031_099, 0, 0, 1)):
                raw_client.reply = reply
                with self.assertRaises(ValueError):
                    getter()
        raw_client.reply = struct.pack('<hhhiiiB', 0, 0, 0, 0, 0, 1, 0)
        with self.assertRaises(ValueError):
            raw_client.get_board_mount()

    def test_board_center_gate_never_falls_back(self):
        self.assertFalse(KinisiController().supports_board_mounting())
        for minor, patch_no, proto in ((4, 0, 23), (3, 1, 24), (4, 1, 24)):
            client, transport, _ = self.connect(minor=minor, board_patch=patch_no, protocol=proto)
            self.assertFalse(client.supports_board_mounting())
            before = len(transport.frames)
            for call in (client.get_board_mount_geometry, lambda: client.set_board_mount_geometry(0, 0, 0, 0, 0, 0)):
                with self.assertRaises(ProtocolError):
                    call()
            self.assertEqual(len(transport.frames), before)
        client, transport, _ = self.connect(protocol=25)
        self.assertTrue(client.supports_board_mounting())
        before = len(transport.frames)
        with patch.object(controller_module, 'get_board_profile', return_value={'capabilities': {}}):
            self.assertFalse(client.supports_board_mounting())
            with self.assertRaises(ProtocolError):
                client.get_board_mount()
        self.assertEqual(len(transport.frames), before)

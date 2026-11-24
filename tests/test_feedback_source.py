"""Ordinary Python transport and source-episode correlation for the v0.4 selector."""
import math
import struct
import unittest

from pykinisi import (ERROR, SET_FEEDBACK_SOURCE, GET_FEEDBACK_SOURCE_STATUS, GET_FEEDBACK_SOURCE_VECTOR,
                      SET_FEEDBACK_FUSION_MOUNT, GET_FEEDBACK_FUSION_MOUNT, ErrorCode, FeedbackSource,
                      FeedbackSourceVectorKind, KinisiCommands, ControllerError, ProtocolError)
import test_sensors
from test_controller import frame
from test_generator import recording_client


STAMP = (1 << 60) + 7


def status(source, origin=11, valid=True, sequence=42, stamp=STAMP, age_us=937, limit_ms=None):
    return struct.pack('<BBBBIQIBIH', source, source if valid else 255, int(valid), 0 if valid else 5,
                       sequence if valid else 0, stamp if valid else 0, origin, 0, age_us if valid else 0xffffffff,
                       (300, 50, 60)[source] if limit_ms is None else limit_ms)


class FeedbackSourceTests(unittest.TestCase):
    connect = test_sensors.SensorSessionTests.connect

    def test_wire_reads_and_independent_sources(self):
        client, transport, board = self.connect(protocol=21)
        for source in (FeedbackSource.ENCODER, FeedbackSource.IMU_ONLY, FeedbackSource.FUSED):
            calls = []

            def reply(command, mid, payload):
                calls.append((command, payload))
                if command == GET_FEEDBACK_SOURCE_STATUS:
                    data = status(source)
                else:
                    self.assertEqual(command, GET_FEEDBACK_SOURCE_VECTOR)
                    data = struct.pack('<Iddd', 42, 1.25 if payload[0] == 0 else -0.5, 0, 0.3)
                transport.inject(frame(command, mid, data))

            board.command_hook = reply
            sample = client.read_feedback_source_snapshot()
            self.assertEqual((sample.status.requested, sample.status.active, sample.status.timestamp_us,
                              sample.status.age_us, sample.pose.x, sample.twist.x), (source, source, STAMP, 937, 1.25, -0.5))
            self.assertEqual(calls, [(GET_FEEDBACK_SOURCE_STATUS, b''), (GET_FEEDBACK_SOURCE_VECTOR, struct.pack('<BII', 0, 11, 42)),
                                     (GET_FEEDBACK_SOURCE_VECTOR, struct.pack('<BII', 1, 11, 42)), (GET_FEEDBACK_SOURCE_STATUS, b'')])

    def test_repeated_sequence_after_origin_change_and_expiry_mid_read(self):
        client, transport, board = self.connect(protocol=21)
        origins = iter(range(6))
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid,
            status(2, next(origins)) if cmd == GET_FEEDBACK_SOURCE_STATUS else struct.pack('<Iddd', 42, 8, 0, 0)))
        with self.assertRaisesRegex(ProtocolError, 'coherent snapshot'):
            client.read_feedback_source_snapshot()
        replies = iter([(GET_FEEDBACK_SOURCE_STATUS, status(1)),
                        (ERROR, bytes([GET_FEEDBACK_SOURCE_VECTOR, ErrorCode.SAMPLE_NOT_AVAILABLE])),
                        (GET_FEEDBACK_SOURCE_STATUS, status(1, valid=False))])
        def loss(cmd, mid, payload):
            code, data = next(replies)
            transport.inject(frame(code, mid, data))
        board.command_hook = loss
        sample = client.read_feedback_source_snapshot()
        self.assertEqual((sample.status.requested, sample.status.active, sample.status.age_us, sample.pose, sample.twist),
                         (1, 255, 0xffffffff, None, None))

    def test_advancing_producer_keeps_original_sample_and_measured_age(self):
        client, transport, board = self.connect(protocol=21)
        for source in (0, 1, 2):
            start_age = 150000 if source == 0 else 1000
            replies = iter([status(source, sequence=0xffffffff, age_us=start_age),
                            struct.pack('<Iddd', 0xffffffff, 7, 0, 0), struct.pack('<Iddd', 0xffffffff, -2, 0, 0),
                            status(source, sequence=0, stamp=STAMP+10000, age_us=start_age)])
            def reply(cmd, mid, payload):
                if cmd == GET_FEEDBACK_SOURCE_VECTOR:
                    self.assertEqual(struct.unpack('<II', payload[1:]), (11, 0xffffffff))
                transport.inject(frame(cmd, mid, next(replies)))
            board.command_hook = reply
            value = client.read_feedback_source_snapshot()
            self.assertEqual((value.status.timestamp_us, value.status.sequence, value.status.age_us,
                              value.pose.x, value.twist.x), (STAMP, 0xffffffff, start_age+10000, 7, -2))

    def test_cache_lifetime_source_specific_age_and_regressions(self):
        client, transport, board = self.connect(protocol=21)
        vector = struct.pack('<Iddd', 42, 7, 0, 0)
        for source, start_age, delta, age, seq, limit in [
            (1, 40000, 12000, 40000, 43, 50), (0, 150000, 60000, 150000, 43, 300),
            (0, 50000, 12000, 49500, 43, 60), (2, 1000, -1, 1000, 43, 60), (2, 1000, 1000, 1000, 41, 60),
        ]:
            replies = iter([status(source, age_us=start_age), vector, vector,
                            status(source, sequence=seq, stamp=STAMP+delta, age_us=age, limit_ms=limit)] * 3)
            board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid, next(replies)))
            with self.assertRaisesRegex(ProtocolError, 'coherent snapshot'):
                client.read_feedback_source_snapshot()

    def test_unsafe_switch_older_boards_and_reconnect_are_actuator_free(self):
        client, transport, board = self.connect(protocol=21)
        self.assertTrue(client.supports_feedback_source_selector())
        before = len(transport.frames)
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(ERROR, mid, bytes([cmd, ErrorCode.SOURCE_SWITCH_UNSAFE])))
        with self.assertRaises(ControllerError) as denied:
            client.set_feedback_source(FeedbackSource.IMU_ONLY)
        self.assertEqual(denied.exception.error_code, ErrorCode.SOURCE_SWITCH_UNSAFE)
        self.assertEqual(len(transport.frames), before + 1)
        for minor, patch, version in [(3, 0, 21), (4, 1, 21), (4, 0, 20)]:
            client.disconnect()
            client, transport, _ = self.connect(minor=minor, board_patch=patch, protocol=version, client=client)
            self.assertFalse(client.supports_feedback_source_selector())
            before = len(transport.frames)
            for call in (lambda: client.set_feedback_source(2), client.get_feedback_source_status,
                         lambda: client.get_feedback_source_vector(0, 11, 42), lambda: client.set_feedback_fusion_mount(0),
                         client.get_feedback_fusion_mount):
                with self.assertRaises(ProtocolError):
                    call()
            self.assertEqual(len(transport.frames), before)
            self.assertFalse(any(f[1] in (SET_FEEDBACK_SOURCE, SET_FEEDBACK_FUSION_MOUNT) for f in transport.frames))

    def test_strict_payloads_and_mount(self):
        client = recording_client({'KinisiCommands': KinisiCommands})
        for value in (True, '1', -1, 3, 255, 0.5):
            with self.assertRaises(ValueError):
                client.set_feedback_source(value)
        for value in (True, '0', -1, 2):
            with self.assertRaises(ValueError):
                client.get_feedback_source_vector(value, 11, 42)
        for value in (True, '0', -1, 0.5, 1 << 32):
            with self.assertRaises(ValueError):
                client.get_feedback_source_vector(0, value, 42)
            with self.assertRaises(ValueError):
                client.get_feedback_source_vector(1, 11, value)
        for value in (True, '0', math.nan, math.inf, 4):
            with self.assertRaises(ValueError):
                client.set_feedback_fusion_mount(value)
        self.assertEqual(client.calls, [])
        client.set_feedback_fusion_mount(-math.pi)
        self.assertEqual(client.calls[-1], (SET_FEEDBACK_FUSION_MOUNT, struct.pack('<d', -math.pi), 0))
        client.reply = struct.pack('<Bd', 1, -math.pi)
        self.assertEqual(client.get_feedback_fusion_mount().mounting_yaw_rad, -math.pi)
        self.assertEqual(client.calls[-1], (GET_FEEDBACK_FUSION_MOUNT, b'', 9))
        for data, method in ((status(2)[:-1], client.get_feedback_source_status),
                             (bytes([2, 1, 1]) + status(2)[3:], client.get_feedback_source_status),
                             (struct.pack('<Iddd', 42, 0, math.nan, 0),
                              lambda: client.get_feedback_source_vector(FeedbackSourceVectorKind.TWIST, 11, 42)),
                             (struct.pack('<Bd', 2, 0), client.get_feedback_fusion_mount)):
            client.reply = data
            with self.assertRaises(ValueError):
                method()

"""Protocol 2.22 source health through a connected controller and byte transport."""
import struct
import unittest

from pykinisi import (
    ERROR, GET_FEEDBACK_SOURCE_HEALTH, GET_FEEDBACK_IMU_CHANNELS, KinisiCommands,
    FeedbackSource, FeedbackSourceReason as Reason, FeedbackSourceHealthState as State,
    FeedbackSourceHealthFlag as Flag, FusionFault, ProtocolError, ControllerError, ErrorCode,
)
import test_sensors
from test_controller import frame
from test_generator import recording_client


STAMP = (1 << 60) + 33
HEALTH = struct.Struct('<BBBBIIQII')
CHANNELS = struct.Struct('<QQII')


class SourceHealthTests(unittest.TestCase):
    connect = test_sensors.SensorSessionTests.connect

    def test_read_only_source_health_and_recovery(self):
        client, transport, board = self.connect(protocol=22)
        self.assertTrue(client.supports_feedback_source_health())
        fresh = Flag.PROVENANCE | Flag.FRESH
        records = [
            (FeedbackSource.ENCODER, State.HEALTHY, Reason.OK, FusionFault.NONE, fresh, 90, STAMP, 100, 300000),
            (FeedbackSource.FUSED, State.UNAVAILABLE, Reason.MOUNT_REQUIRED, FusionFault.MOUNT_INVALID, 0, 0, 0, 0xffffffff, 60000),
            (FeedbackSource.IMU_ONLY, State.INDETERMINATE, Reason.OK, FusionFault.NONE,
             fresh | Flag.DISAGREEMENT | Flag.COMPARISON_AVAILABLE, 42, STAMP, 200, 35000),
            (FeedbackSource.IMU_ONLY, State.SUSPECT, Reason.FAULT, FusionFault.SATURATION,
             fresh | Flag.SATURATION, 42, STAMP, 2100, 35000),
            (FeedbackSource.IMU_ONLY, State.UNAVAILABLE, Reason.STALE, FusionFault.STALE,
             Flag.PROVENANCE | Flag.TIMESTAMP_FAULT, 42, STAMP, 80000, 35000),
            (FeedbackSource.IMU_ONLY, State.SUSPECT, Reason.WAITING_SAMPLE, FusionFault.NONE,
             fresh | Flag.RECOVERING, 43, STAMP + 100000, 300, 35000),
            (FeedbackSource.IMU_ONLY, State.HEALTHY, Reason.OK, FusionFault.NONE, fresh, 44, STAMP + 110000, 200, 35000),
        ]
        for record in records:
            def reply(command, mid, payload):
                self.assertEqual((command, payload), (GET_FEEDBACK_SOURCE_HEALTH, bytes([record[0]])))
                transport.inject(frame(command, mid, HEALTH.pack(*record)))
            board.command_hook = reply
            sample = client.get_feedback_source_health(record[0])
            self.assertEqual((sample.source, sample.health, sample.reason, sample.detail, sample.flags, sample.sequence,
                              sample.timestamp_us, sample.age_us, sample.max_age_us), record)
        self.assertEqual([f[1] for f in transport.frames[-len(records):]], [GET_FEEDBACK_SOURCE_HEALTH] * len(records))

    def test_independent_channel_reads_and_reconnect(self):
        client, transport, board = self.connect(protocol=22)
        for record in [(0, 0, 0xffffffff, 0xffffffff), (STAMP, 0, 35001, 0xffffffff),
                       (STAMP, STAMP + 10000, 40100, 30100), (STAMP + 50000, STAMP + 47500, 200, 2700)]:
            def reply(command, mid, payload):
                self.assertEqual((command, payload), (GET_FEEDBACK_IMU_CHANNELS, b''))
                transport.inject(frame(command, mid, CHANNELS.pack(*record)))
            board.command_hook = reply
            data = client.get_feedback_imu_channels()
            self.assertEqual((data.accel_timestamp_us, data.gyro_timestamp_us, data.accel_age_us, data.gyro_age_us), record)
        client.disconnect()
        self.assertFalse(client.supports_feedback_source_health())
        client, transport, board = self.connect(protocol=22, client=client)
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(cmd, mid, CHANNELS.pack(0, 0, 0xffffffff, 0xffffffff)))
        self.assertEqual(client.get_feedback_imu_channels().gyro_age_us, 0xffffffff)
        board.command_hook = lambda cmd, mid, payload: transport.inject(frame(ERROR, mid, bytes((cmd, ErrorCode.SENSOR_NOT_AVAILABLE))))
        with self.assertRaises(ControllerError) as caught:
            client.get_feedback_source_health(0)
        self.assertEqual(caught.exception.error_code, ErrorCode.SENSOR_NOT_AVAILABLE)

    def test_unsupported_identities_and_sources_do_not_transmit(self):
        for minor, patch, protocol in [(4, 0, 21), (3, 0, 22), (4, 1, 22)]:
            client, transport, _ = self.connect(minor=minor, board_patch=patch, protocol=protocol)
            self.assertFalse(client.supports_feedback_source_health())
            self.assertEqual(client.supports_feedback_source_selector(), (minor, patch, protocol) == (4, 0, 21))
            count = len(transport.frames)
            for call in (lambda: client.get_feedback_source_health(0), client.get_feedback_imu_channels):
                with self.assertRaises(ProtocolError):
                    call()
            self.assertEqual(len(transport.frames), count)
        client = recording_client({'KinisiCommands': KinisiCommands})
        for source in (True, -1, 3, FeedbackSource.NONE, 1.0, '1'):
            with self.assertRaises(ValueError):
                client.get_feedback_source_health(source)
        self.assertEqual(client.calls, [])

    def test_rejects_bad_health_identity_enums_and_lengths(self):
        client = recording_client({'KinisiCommands': KinisiCommands})
        data = HEALTH.pack(1, 0, 0, 0, 0x80000003, 42, STAMP, 500, 35000)
        client.reply = data
        self.assertEqual(client.get_feedback_source_health(1).flags, 0x80000003)
        for reply in (data[:-1], data + b'x', b'\x02' + data[1:], data[:1] + b'\x04' + data[2:],
                      data[:2] + b'\x0a' + data[3:], data[:3] + b'\x0a' + data[4:]):
            client.reply = reply
            with self.assertRaises(ValueError):
                client.get_feedback_source_health(1)
        for size in (23, 25):
            client.reply = bytes(size)
            with self.assertRaises(ValueError):
                client.get_feedback_imu_channels()

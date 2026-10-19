"""Offline tests; neither Arduino nor SpikerBox is required."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import emg_claw as e


def frame(*values):
    chunks = bytearray()
    for i, x in enumerate(values):
        assert 0 <= x < 16384
        high = ((x >> 7) & 0x7F) | (0x80 if i == 0 else 0)
        chunks.extend((high, x & 0x7F))
    return bytes(chunks)


class ProtocolTests(unittest.TestCase):
    def test_serial_14bit_two_channels_fragmentation(self):
        hw = e.Hardware("HUMANSB", 5000, 2, 8192, 1.0)
        decoder = e.FrameDecoder(hw, channel=0)
        a = frame(8192 + 120, 8192 - 10)
        b = frame(8192 - 300, 8192 + 45)
        self.assertEqual(decoder.feed(a[:2]), [])
        self.assertEqual(decoder.feed(a[2:] + b + b[:2]), [120.0, -300.0])
        self.assertEqual(decoder.feed(b[2:]), [-300.0])

    def test_10bit_second_channel_and_resync(self):
        hw = e.Hardware("MUSCLE", 10000, 2, 512, 30.0)
        decoder = e.FrameDecoder(hw, channel=1)
        self.assertEqual(decoder.feed(b"\x00\x7f" + frame(512, 517)), [150.0])
        self.assertEqual(decoder.feed(b"\x82" + frame(512, 510)), [-60.0])

    def test_escape_sequence_split_over_reads(self):
        demux = e.MessageDemux()
        first = frame(500, 510)
        last = frame(520, 530)
        data = first + e.START_MSG + b"HWT:HUMANSB;FWV:1.0;" + e.END_MSG + last
        raw = bytearray()
        messages = []
        for i in range(0, len(data), 3):
            r, m = demux.feed(data[i:i+3])
            raw.extend(r)
            messages.extend(m)
        self.assertEqual(raw, first + last)
        self.assertEqual(e.metadata_value(messages, "HWT"), "HUMANSB")
        self.assertEqual(e.metadata_value(messages, "FWV"), "1.0")

    def test_hid_packet(self):
        x = frame(512, 530)
        self.assertEqual(e.decode_hid_report(bytes([0x3f, len(x)]) + x), x)
        self.assertEqual(e.decode_hid_report(bytes([0, 0x3f, len(x)]) + x), x)
        self.assertEqual(e.decode_hid_report(bytes([0x3f, 60]) + x), b"")

    def test_out_of_range_channel_rejected(self):
        with self.assertRaises(ValueError):
            e.FrameDecoder(e.Hardware("x", 10000, 2, 512, 1.0), 2)


class DSPTests(unittest.TestCase):
    def test_rms_scales_with_activity(self):
        def levels(amplitude):
            processor = e.EmgEnvelope(1000)
            values = [amplitude * math.sin(2 * math.pi * 105 * i / 1000) + 512
                      for i in range(3000)]
            return processor.feed(values)[-10:]
        low = sum(levels(20)) / 10
        high = sum(levels(100)) / 10
        self.assertGreater(high, low * 4.5)
        self.assertLess(high, low * 5.5)

    def test_digital_dc_offset_removed(self):
        processor = e.EmgEnvelope(1000)
        levels = processor.feed([8192] * 2500)
        self.assertLess(levels[-1], 0.1)

    def test_calibration_round_trip(self):
        class Dummy:
            mode = "serial"
            hardware = e.Hardware("HUMANSB", 5000, 2, 8192, 1.0)
            channel = 0
        d = Dummy()
        prev = e.CONFIG_PATH
        with tempfile.TemporaryDirectory() as temp:
            e.CONFIG_PATH = Path(temp) / "calibration.json"
            try:
                e.save_calibration(d, 12, 80)
                self.assertEqual(e.load_calibration(d), (12.0, 80.0))
                d.channel = 1
                self.assertIsNone(e.load_calibration(d))
            finally:
                e.CONFIG_PATH = prev

    def test_calibration_rejects_bad_data(self):
        self.assertEqual(e.percentile([20, 30, 10, 40], 0.75), 30)
        with self.assertRaises(ValueError):
            e.percentile([], .5)


class OutputTests(unittest.TestCase):
    def test_serial_command_format_and_clamp(self):
        class FakePort:
            def __init__(self):
                self.data = []
            def write(self, x):
                self.data.append(x)
                return len(x)
        writer = e.ClawOutput(None, True)
        port = FakePort()
        writer.serial = port
        writer.send(55)
        writer.send(-8)
        writer.send(1000)
        self.assertEqual(port.data, [b"G,55\n", b"G,0\n", b"G,100\n"])
        writer.serial = None


class HardwareMockTests(unittest.TestCase):
    def test_serial_handshake_and_emg_frame(self):
        from unittest.mock import patch
        from types import SimpleNamespace

        class FakeSerial:
            def __init__(self, port, baudrate, timeout, write_timeout):
                self.is_open = True
                self.sent = []
                self.queue = bytearray()
            @property
            def in_waiting(self):
                return len(self.queue)
            def write(self, b):
                self.sent.append(b)
                if b.startswith(b"b:;"):
                    self.queue += e.START_MSG + b"HWT:HUMANSB;" + e.END_MSG
                return len(b)
            def read(self, n):
                result = self.queue[:n]
                del self.queue[:n]
                return bytes(result)
            def reset_input_buffer(self):
                self.queue.clear()
            def close(self):
                self.is_open = False

        with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=FakeSerial)}):
            with patch.object(e.time, "sleep", lambda x: None):
                dev = e.SerialSpikerBox("COM5", channel=0)
                try:
                    self.assertEqual(dev.hardware.name, "HUMANSB")
                    self.assertIn(b"conf s:5000;c:2;\n", dev.serial.sent)
                    dev.serial.queue.extend(frame(8200, 8192))
                    self.assertEqual(dev.read_samples(), [8.0])
                finally:
                    dev.close()

    def test_hid_startup_commands_and_report(self):
        from unittest.mock import patch
        from types import SimpleNamespace

        class FakeHID:
            def __init__(self):
                self.sent = []
                self.queue = []
                self.closed = False
            def open_path(self, path):
                self.path = path
            def write(self, packet):
                self.sent.append(bytes(packet))
                return len(packet)
            def read(self, n, ms):
                if not self.queue:
                    return []
                return list(self.queue.pop(0))
            def close(self):
                self.closed = True

        fake = FakeHID()
        module = SimpleNamespace(device=lambda: fake, enumerate=lambda vid, pid: [
            {"vendor_id": e.BYB_VID, "product_id": 1, "path": b"mock-hid", "product_string": "SpikeRecorder"}])
        with patch.dict(sys.modules, {"hid": module}):
            with patch.object(e.time, "sleep", lambda x: None):
                dev = e.HIDSpikerBox(channel=0)
                try:
                    self.assertEqual(len(fake.sent), 4)
                    self.assertEqual([p[2:p.index(b'\n')+1] for p in fake.sent],
                                     [b'?:;\n', b'max:;\n', b'board:;\n', b'start:;\n'])
                    payload = frame(515, 520)
                    fake.queue.append(bytes([0x3f, len(payload)]) + payload)
                    self.assertEqual(dev.read_samples(), [186.0])
                finally:
                    dev.close()
        self.assertTrue(fake.closed)


if __name__ == "__main__":
    unittest.main()

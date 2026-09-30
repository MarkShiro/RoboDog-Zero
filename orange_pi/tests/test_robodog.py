import struct
import tempfile
import unittest
from pathlib import Path

from robodog import protocol
from robodog.app import run
from robodog.lidar import validate_scan
from robodog.coin_d6 import CoinD6Parser
from robodog.screen import HEIGHT, WIDTH, page_from_touch, paint, rgb565
from robodog.state import Snapshot


class WireTests(unittest.TestCase):
    def test_crc_and_typed_frames(self):
        line = protocol.encode("C,65535,0000,7400,2,0")
        body = protocol.decode(line)
        self.assertEqual(protocol.control(body).seq, 65535)
        self.assertTrue(protocol.sequence_newer(0, 65535))
        self.assertFalse(protocol.sequence_newer(65535, 0))
        self.assertEqual(protocol.telemetry("T,2,1,25800,010F").battery_mv, 25800)
        self.assertEqual(protocol.imu("I,-12,3,29,0002").roll, -12)
        self.assertEqual(list(protocol.Framer().feed(line)), [line])

    def test_bad_frame_and_overflow(self):
        line = bytearray(protocol.encode("C,1,0000,7400,2,0"))
        line[4] = ord("2")
        with self.assertRaises(ValueError):
            protocol.decode(bytes(line))
        with self.assertRaises(ValueError):
            protocol.decode(b"C,1,0000,7400,2,0*+F\n")
        framer = protocol.Framer()
        self.assertEqual(list(framer.feed(b"x" * 60 + b"\n" + protocol.encode("X"))),
                         [protocol.encode("X")])


class ScreenTests(unittest.TestCase):
    def test_three_pages_and_bottom_touch(self):
        self.assertEqual((WIDTH, HEIGHT), (240, 320))
        for page in (0, 1, 2):
            picture = paint(Snapshot(), page, 100.0)
            self.assertEqual(picture.size, (WIDTH, HEIGHT))
            self.assertEqual(len(rgb565(picture)), WIDTH * HEIGHT * 2)
        self.assertEqual([page_from_touch(x, 300) for x in (40, 120, 200)],
                         [0, 1, 2])
        self.assertIsNone(page_from_touch(100, 260))
        self.assertEqual(page_from_touch(79, 270), 0)
        self.assertEqual(page_from_touch(80, 270), 1)
        self.assertEqual(page_from_touch(160, 270), 2)
        self.assertIsNone(page_from_touch(240, 300))

    def test_zero_wordmark_fills_main_area(self):
        picture = paint(Snapshot(), 0, 100.0)
        bright = []
        for y in range(40, 260):
            for x in range(10, 230):
                r, g, b = picture.getpixel((x, y))
                if r > 100 and g > 180 and b > 200:
                    bright.append((x, y))
        self.assertTrue(bright)
        self.assertLessEqual(min(x for x, _ in bright), 30)
        self.assertGreaterEqual(max(x for x, _ in bright), 210)
        self.assertLessEqual(min(y for _, y in bright), 65)
        self.assertGreaterEqual(max(y for _, y in bright), 245)

    def test_demo_snapshots_are_labeled(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "screen.png"
            for page in (0, 1, 2):
                run({"ui_backend": "png", "png_path": str(path)},
                    demo=True, snapshot=True, page=page)
                self.assertTrue(path.exists())
                self.assertGreater(path.stat().st_size, 1000)


class LidarTests(unittest.TestCase):
    def test_only_checked_points_are_used(self):
        self.assertEqual(validate_scan({"points": [[0, .05], [359, 12], [1, 13]]}),
                         [(0.0, .05), (359.0, 12.0)])
        for scan in (None, {"points": [[float("nan"), 1]]},
                     {"points": [[1]]}, {"points": [[True, 1]]}):
            with self.assertRaises(ValueError):
                validate_scan(scan)

    def test_coin_tof_packet_checksum_and_resynchronization(self):
        count = 8
        start = (10 * 64 << 1) | 1
        stop = (17 * 64 << 1) | 1
        sample = bytes((0, (1000 % 64) << 2, 1000 // 64))
        payload = sample * count
        check = 0x55AA ^ start ^ stop ^ (1 | count << 8)
        for at in range(0, len(payload), 3):
            check ^= payload[at]
            check ^= payload[at + 1] | payload[at + 2] << 8
        good = b"\xAA\x55" + bytes((1, count)) + struct.pack("<HHH", start, stop, check) + payload
        bad = bytearray(good)
        bad[-1] ^= 1
        parser = CoinD6Parser()
        self.assertEqual(list(parser.feed(b"junk" + bytes(bad)[:18])), [])
        self.assertEqual(list(parser.feed(bytes(bad)[18:] + good[:7])), [])
        scans = list(parser.feed(good[7:]))
        self.assertEqual(len(scans), 1)
        self.assertTrue(scans[0][0])
        self.assertEqual(len(scans[0][1]), count)
        self.assertEqual(scans[0][1][0], (10.0, 1.0))
        self.assertAlmostEqual(scans[0][1][-1][0], 17.0)
        self.assertGreaterEqual(parser.bad_packets, 1)


if __name__ == "__main__":
    unittest.main()

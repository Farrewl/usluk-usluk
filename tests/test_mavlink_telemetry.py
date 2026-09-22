"""Test app/mavlink_telemetry.py — logika yang bisa diuji tanpa Pixhawk.

Koneksi nyata (heartbeat, attitude) = butuh hardware, diuji manual.
Di sini: deteksi port serial (prioritas /dev/ttyACM* bila ada).

Jalankan:  python3 -m unittest discover -s tests -v
"""

import glob
import unittest

from app.mavlink_telemetry import detect_serial_port


class TestDetectSerialPort(unittest.TestCase):

    def test_selalu_mengembalikan_string(self):
        port = detect_serial_port()
        self.assertIsNotNone(port)
        self.assertIsInstance(port, str)

    def test_preferred_dipakai_jika_ada(self):
        # 'COM8' tidak ada di Linux -> harus jatuh ke port yang ada
        port = detect_serial_port("COM8")
        self.assertIsNotNone(port)
        if glob.glob("/dev/ttyACM*"):
            self.assertTrue(port.startswith("/dev/ttyACM"),
                            f"harus pilih ACM, dapat {port}")

    def test_preferred_linux_dihormati(self):
        if not glob.glob("/dev/ttyACM0"):
            self.skipTest("tidak ada /dev/ttyACM0 di mesin ini")
        port = detect_serial_port("/dev/ttyACM0")
        self.assertEqual(port, "/dev/ttyACM0")


if __name__ == "__main__":
    unittest.main()
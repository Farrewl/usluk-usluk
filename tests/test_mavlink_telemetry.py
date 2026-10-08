"""Test app/mavlink_telemetry.py — logika yang bisa diuji tanpa Pixhawk.

Koneksi nyata (heartbeat, attitude) = butuh hardware, diuji manual.
Di sini: deteksi port serial (prioritas /dev/ttyACM* bila ada).

Jalankan:  python3 -m unittest discover -s tests -v
"""

import glob
import os
import unittest

from app.mavlink_telemetry import (MavlinkTelemetry, detect_serial_port,
                                   GCS_DEFAULT_PORT)


class TestDetectSerialPort(unittest.TestCase):

    @staticmethod
    def _have_any_serial_port():
        # Mesin dev bisa tanpa port serial sama sekali (Pixhawk dicabut).
        if os.name == "nt":
            return any(os.path.exists(f"COM{n}") for n in range(3, 10))
        return bool(glob.glob("/dev/ttyACM*") or glob.glob("/dev/ttyUSB*"))

    def test_return_none_atau_string(self):
        # Tanpa port serial -> None sah (pemanggil fallback ke mock).
        port = detect_serial_port()
        self.assertTrue(port is None or isinstance(port, str),
                        f"harus None atau str, dapat {port!r}")

    def test_preferred_dipakai_jika_ada(self):
        # Linux: 'COM8' tidak ada -> harus jatuh ke port yang ada / None.
        # Windows: path /dev/ tak berlaku; detect_serial_port mengembalikan
        # 'COM8' sebagai fallback (driver membukanya saat koneksi) — yang
        # dipastikan hanya tipenya berupa str.
        port = detect_serial_port("COM8")
        if os.name == "nt":
            self.assertIsInstance(port, str)
            return
        if self._have_any_serial_port():
            self.assertTrue(port.startswith("/dev/ttyACM")
                            or port.startswith("/dev/ttyUSB"),
                            f"harus pilih port Linux, dapat {port}")
        else:
            self.assertIsNone(port)

    def test_preferred_linux_dihormati(self):
        if not glob.glob("/dev/ttyACM0"):
            self.skipTest("tidak ada /dev/ttyACM0 di mesin ini")
        port = detect_serial_port("/dev/ttyACM0")
        self.assertEqual(port, "/dev/ttyACM0")


class TestGcsForward(unittest.TestCase):
    """Forward QGC: default mati, nyala via enable, tanpa hardware."""

    def test_default_mati(self):
        mav = MavlinkTelemetry(port="/dev/ttyTIDAKADA")
        self.assertFalse(mav.gcs_active)
        self.assertEqual(GCS_DEFAULT_PORT, 14550)

    def test_enable_tanpa_connect_tetap_aman(self):
        mav = MavlinkTelemetry(port="/dev/ttyTIDAKADA")
        mav.enable_gcs_forward(True, ip="127.0.0.1", port=14550)
        # Belum connect -> socket belum dibuka, tapi flag tersimpan.
        self.assertFalse(mav.gcs_active)
        # poll() tanpa koneksi harus aman (tidak crash).
        mav.poll()
        mav.enable_gcs_forward(False)
        self.assertFalse(mav.gcs_active)

    def test_close_selalu_aman(self):
        mav = MavlinkTelemetry(port="/dev/ttyTIDAKADA")
        mav.enable_gcs_forward(True)
        mav.close()  # tanpa connect pun tidak boleh crash
        self.assertFalse(mav.gcs_active)


if __name__ == "__main__":
    unittest.main()
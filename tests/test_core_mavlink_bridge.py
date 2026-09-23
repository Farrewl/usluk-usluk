"""Test kunci — buktikan bahwa C (mavlink_bridge.c) menghasilkan frame MAVLink
yang SAMA PERSIS (byte demi byte) dengan pymavlink 2.4.49.

Cara kerja (pola sama dengan test_core_* lain):
  1. Compile core/src/mavlink_bridge.c -> libmavlink_bridge.so (gcc, sementara).
  2. Bangun frame dengan C dan dengan pymavlink dialect v10 (framing v1,
     magic 0xFE — yang dimengerti Pixhawk) untuk input IDENTIK.
  3. Bandingkan byte-nya; untuk input acak (100 kasus) + kasus nilai emas.
  4. Validasi CRC pakai pymavlink.mavutil.x25crc secara independen.
  5. Serial: tanpa hardware, mav_serial_open harus gagal halus (fd < 0).

Jalankan:  python3 -m unittest discover -s tests -v
"""

import ctypes
import os
import random
import shutil
import subprocess
import tempfile
import unittest

from pymavlink.dialects.v10 import ardupilotmega as ap_v1
from pymavlink.mavutil import x25crc

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_C = os.path.join(REPO, "core", "src", "mavlink_bridge.c")
INC_C = os.path.join(REPO, "core", "include")

_HEARTBEAT_ARGS = dict(type=2, autopilot=3, base_mode=81, custom_mode=0,
                       system_status=4, mavlink_version=3)


def _build_bridge_lib():
    cc = shutil.which("gcc") or shutil.which("cc")
    if not cc:
        raise unittest.SkipTest("gcc tidak tersedia di mesin ini")
    tmp = tempfile.mkdtemp(prefix="libmav_")
    so = os.path.join(tmp, "libmavlink_bridge.so")
    subprocess.run([cc, "-O2", "-fPIC", "-shared", "-I", INC_C,
                    SRC_C, "-o", so],
                   check=True, capture_output=True)
    lib = ctypes.CDLL(so)

    lib.mav_build_heartbeat_v1.restype = ctypes.c_size_t
    lib.mav_build_heartbeat_v1.argtypes = [
        ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
        ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
        ctypes.c_uint32, ctypes.c_uint8, ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t]

    lib.mav_build_set_attitude_target_v1.restype = ctypes.c_size_t
    lib.mav_build_set_attitude_target_v1.argtypes = [
        ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
        ctypes.c_uint32, ctypes.c_float * 4,
        ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t]

    lib.mav_crc_v1_frame.restype = ctypes.c_uint16
    lib.mav_crc_v1_frame.argtypes = [ctypes.c_uint8, ctypes.c_uint8,
                                     ctypes.c_uint8, ctypes.c_uint8,
                                     ctypes.POINTER(ctypes.c_uint8),
                                     ctypes.c_uint16, ctypes.c_uint8]

    lib.mav_serial_open.restype = ctypes.c_int
    lib.mav_serial_open.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.mav_serial_close.argtypes = [ctypes.c_int]
    return lib


def _c_frame(fn, args, cap=512):
    buf = (ctypes.c_uint8 * cap)()
    n = fn(*args, buf, cap)
    return bytes(buf[:n])


class TestCMavlinkVsPymavlink(unittest.TestCase):
    """Cross-check C vs pymavlink v1 pada vektor emas + input acak."""

    @classmethod
    def setUpClass(cls):
        cls.lib = _build_bridge_lib()
        cls.mav = ap_v1.MAVLink(None, srcSystem=1, srcComponent=1)

    def _py_pack(self, msg, sysid, compid, seq):
        """pack dengan header yang kita kendalikan (bukan default master)."""
        self.mav.seq = seq
        self.mav.srcSystem = sysid
        self.mav.srcComponent = compid
        return msg.pack(self.mav)

    def _crc(self, msgid, sysid, compid, seq, payload, extra):
        buf = (ctypes.c_uint8 * len(payload))(*payload)
        return self.lib.mav_crc_v1_frame(msgid, sysid, compid, seq, buf,
                                         len(payload), extra)

    # ---------------- vektor emas ----------------

    def test_heartbeat_emas(self):
        c = _c_frame(self.lib.mav_build_heartbeat_v1,
                     (1, 1, 0, 2, 3, 81, 0, 4, 3))
        py = self._py_pack(self.mav.heartbeat_encode(**_HEARTBEAT_ARGS),
                           1, 1, 0)
        self.assertEqual(c, py,
                         f"heartbeat C={c.hex()} py={py.hex()}")

    def test_set_attitude_emas(self):
        q = (ctypes.c_float * 4)(1.0, 0.0, 0.0, 0.0)
        c = _c_frame(self.lib.mav_build_set_attitude_target_v1,
                     (1, 1, 0, 7, q, 0.1, -0.2, 0.5, 0.65, 1, 1, 0))
        py = self._py_pack(self.mav.set_attitude_target_encode(
            time_boot_ms=7, target_system=1, target_component=1,
            type_mask=0, q=[1.0, 0.0, 0.0, 0.0],
            body_roll_rate=0.1, body_pitch_rate=-0.2, body_yaw_rate=0.5,
            thrust=0.65), 1, 1, 0)
        self.assertEqual(c, py, f"set_att C={c.hex()} py={py.hex()}")

    # ---------------- acak ----------------

    def test_heartbeat_acak(self):
        for _ in range(60):
            sysid, compid, seq = (random.randint(1, 250), random.randint(1, 250),
                                  random.randint(0, 255))
            custom_mode = random.randint(0, 2**31)
            type_ = random.randint(0, 40)
            autopilot = random.randint(0, 20)
            base_mode = random.randint(0, 255)
            status = random.randint(0, 7)
            ver = random.randint(1, 3)
            c = _c_frame(self.lib.mav_build_heartbeat_v1,
                         (sysid, compid, seq, type_, autopilot, base_mode,
                          custom_mode, status, ver))
            py = self._py_pack(self.mav.heartbeat_encode(
                type=type_, autopilot=autopilot, base_mode=base_mode,
                custom_mode=custom_mode, system_status=status,
                mavlink_version=ver), sysid, compid, seq)
            self.assertEqual(c, py, f"heartbeat acak: C={c.hex()} py={py.hex()}")

    def test_set_attitude_acak(self):
        for _ in range(60):
            sysid, compid, seq = (random.randint(1, 250), random.randint(1, 250),
                                  random.randint(0, 255))
            tmb = random.randint(0, 2**31)
            # normalisasi kuaternion supaya "masuk akal" tapi tetap acak
            qv = [random.uniform(-1, 1) for _ in range(4)]
            norm = (sum(x * x for x in qv)) ** 0.5 or 1.0
            qv = [x / norm for x in qv]
            rates = [random.uniform(-1.0, 1.0) for _ in range(3)]
            thrust = random.uniform(0.0, 1.0)
            ts, tc, mask = (random.randint(0, 250), random.randint(0, 250),
                             random.randint(0, 255))
            q = (ctypes.c_float * 4)(*qv)
            c = _c_frame(self.lib.mav_build_set_attitude_target_v1,
                         (sysid, compid, seq, tmb, q, rates[0], rates[1],
                          rates[2], thrust, ts, tc, mask))
            py = self._py_pack(self.mav.set_attitude_target_encode(
                time_boot_ms=tmb, target_system=ts, target_component=tc,
                type_mask=mask, q=qv,
                body_roll_rate=rates[0], body_pitch_rate=rates[1],
                body_yaw_rate=rates[2], thrust=thrust), sysid, compid, seq)
            self.assertEqual(c, py,
                             f"set_att acak: C={c.hex()} py={py.hex()}")

    # ---------------- CRC independen ----------------

    def test_crc_vs_x25crc(self):
        # CRC C harus sama dengan x25crc pymavlink yang dihitung mandiri
        # atas [LEN..payload + extra] (tanpa STX — perilaku MAVLink).
        payload = bytes([0, 0, 0, 0, 2, 3, 81, 4, 3])          # heartbeat
        head_tail = bytes([9, 0, 1, 1, 0])                      # LEN SEQ SYS COMP MSG
        c_crc = self._crc(0, 1, 1, 0, payload, 50)
        c = x25crc(head_tail + payload + bytes([50]))
        self.assertEqual(c_crc, c.crc,
                         f"crc hb C={hex(c_crc)} py={hex(c.crc)}")
        # set_attitude_target dengan payload kecil buatan
        payload2 = bytes(range(39))
        c_crc2 = self._crc(82, 1, 1, 1, payload2, 49)
        c2 = x25crc(bytes([39, 1, 1, 1, 82]) + payload2 + bytes([49]))
        self.assertEqual(c_crc2, c2.crc,
                         f"crc sat C={hex(c_crc2)} py={hex(c2.crc)}")

    # ---------------- serial tanpa hardware ----------------

    def test_serial_tanpa_hardware_gagal_halus(self):
        # Pixhawk boleh dicabut: open path yang tak ada harus < 0 tanpa
        # abort; bukan awal kegagalan bila kembali errno negatif.
        fd = self.lib.mav_serial_open(b"/dev/ttyACM0", 57600)
        self.assertLess(fd, 0, "tidak ada hardware -> fd harus negatif")
        self.lib.mav_serial_close(fd)   # aman untuk fd negatif


if __name__ == "__main__":
    unittest.main()
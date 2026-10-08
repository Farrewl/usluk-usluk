#!/usr/bin/env python3
"""
tests/test_qgc_offboard.py — Unit test manual QGC (MANUAL_CONTROL -> (v,w)).

Yang diuji (tanpa hardware, semua mock):
  1. Konversi protocol MAVLink MANUAL_CONTROL (z throttle, r yaw) -> [-1..1]
  2. Failsafe timeout QGC_MANUAL_TIMEOUT_S (0.5 dtk) -> netral
  3. Hysteresis mode: putus -> HOLD (MANUAL_LOST_HOLD_S) -> AUTO
  4. Arbitrasi dua sumber manual di C (arb_manual2): konflik -> netral

Jalankan:  python3 -m unittest discover -s tests -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import aterkia_core as core
from app.qgc_offboard import (QgcOffboard, convert_manual_control,
                              Z_NEUTRAL, R_FULL)


class _Cfg:
    """Config minimal untuk QgcOffboard (sama nilai dgn settings default)."""
    QGC_MANUAL_TIMEOUT_S = 0.5
    MANUAL_LOST_HOLD_S = 1.0


class _Msg:
    """Palsu pesan MAVLink MANUAL_CONTROL (field x,y,r,z,buttons)."""

    def __init__(self, z=Z_NEUTRAL, r=0):
        self.z = z
        self.r = r
        self.x = 0
        self.y = 0
        self.buttons = 0


class _MavStub:
    """Palsu koneksi pymavlink: recv_match menyerahkan antrean lalu None."""

    def __init__(self):
        self.queue = []

    def recv_match(self, type=None, blocking=False):
        if self.queue:
            return self.queue.pop(0)
        return None


def _qgc_with_stub():
    """QgcOffboard + master stub + jam palsu (default t=0)."""
    q = QgcOffboard(_Cfg())
    mav = _MavStub()
    q.attach_mav(mav)
    q._clock = lambda: 0.0
    return q, mav


class TestConvert(unittest.TestCase):

    def test_tengah_sama_dengan_netral(self):
        # Throttle di tengah + lurus -> v=0, w=0.
        v, w = convert_manual_control(_Msg(z=500, r=0))
        self.assertEqual((v, w), (0.0, 0.0))

    def test_maju_mundur_penuh(self):
        self.assertEqual(convert_manual_control(_Msg(z=1000))[0], 1.0)
        self.assertEqual(convert_manual_control(_Msg(z=0))[0], -1.0)

    def test_yaw_penuh_kanan_kiri(self):
        self.assertEqual(convert_manual_control(_Msg(r=127))[1], 1.0)
        self.assertEqual(convert_manual_control(_Msg(r=-127))[1], -1.0)
        # Di luar rentang protocol di-clamp, tidak pernah meledak.
        self.assertEqual(convert_manual_control(_Msg(r=9999))[1], 1.0)

    def test_z_di_luar_protocol_jadi_netral(self):
        # Driver nakal kirim z=9999 -> anggap netral (aman), bukan gas penuh.
        self.assertEqual(convert_manual_control(_Msg(z=9999)), (0.0, 0.0))

    def test_objek_rusak_jadi_netral(self):
        class Rusak:
            z = "bukan-angka"
            r = None
        self.assertEqual(convert_manual_control(Rusak()), (0.0, 0.0))


class TestPollFailsafe(unittest.TestCase):

    def test_tanpa_koneksi_selalu_netral(self):
        q = QgcOffboard(_Cfg())          # master = None
        out = q.poll(dt=0.05)
        self.assertFalse(out["active"])
        self.assertEqual((out["surge"], out["yaw"]), (0.0, 0.0))
        self.assertEqual(out["source"], "AUTO")

    def test_paket_segar_jadi_manual_qgc(self):
        q, mav = _qgc_with_stub()
        mav.queue.append(_Msg(z=750, r=64))
        out = q.poll(dt=0.05)
        self.assertTrue(out["active"])
        self.assertEqual(out["mode_name"], "MANUAL")
        self.assertEqual(out["source"], "QGC")
        self.assertAlmostEqual(out["surge"], 0.5)
        self.assertAlmostEqual(out["yaw"], 64 / 127.0)

    def test_timeout_setengah_detik_jadi_netral(self):
        # Failsafe inti: paket berhenti -> (v,w) dipaksa netral.
        q, mav = _qgc_with_stub()
        mav.queue.append(_Msg(z=1000, r=127))     # maju + kanan penuh
        q.poll(dt=0.05)
        q._clock = lambda: 0.6                    # 0.6 dtk kemudian > 0.5
        out = q.poll(dt=0.05)
        self.assertFalse(out["qgc_ok"])
        self.assertEqual((out["surge"], out["yaw"]), (0.0, 0.0))

    def test_putus_sejenak_masih_hold_lalu_auto(self):
        # Hysteresis sama dgn RC: putus -> HOLD selama MANUAL_LOST_HOLD_S,
        # baru jatuh ke AUTO (tidak langsung lompat).
        q, mav = _qgc_with_stub()
        mav.queue.append(_Msg(z=1000))
        q.poll(dt=0.05)
        q._clock = lambda: 0.6                    # lewat timeout -> tak fresh
        out = q.poll(dt=0.05)
        self.assertEqual(out["mode_name"], "HOLD")
        # Satu langkah besar (dt > MANUAL_LOST_HOLD_S) -> jatuh ke AUTO.
        out = q.poll(dt=2.0)
        self.assertEqual(out["mode_name"], "AUTO")
        self.assertFalse(out["active"])

    def test_poll_tanpa_core_tidak_crash(self):
        # Mode aman bila lib C tak ada (C_AVAILABLE=False) — kembali netral.
        q, mav = _qgc_with_stub()
        mav.queue.append(_Msg(z=1000))
        try:
            import app.aterkia_core as ac
            old = ac.C_AVAILABLE
            ac.C_AVAILABLE = False
            out = q.poll(dt=0.05)
            self.assertFalse(out.get("active", False))
        finally:
            ac.C_AVAILABLE = old


class TestArbManual2(unittest.TestCase):
    """Arbitrasi dua sumber manual — hitung di C (arbitrator.c)."""

    def test_keduanya_diam_none(self):
        o = core.arb_manual2(False, 0, 0, False, 0, 0)
        self.assertEqual(o["source"], core.MANUAL_NONE)
        self.assertFalse(o["conflict"])
        self.assertEqual((o["surge"], o["yaw"]), (0.0, 0.0))

    def test_qgc_menang_saat_lokal_diam(self):
        o = core.arb_manual2(True, 0.5, -0.25, False, 0, 0)
        self.assertEqual(o["source"], core.MANUAL_QGC)
        self.assertEqual(o["source_name"], "QGC")
        self.assertAlmostEqual(o["surge"], 0.5)
        self.assertAlmostEqual(o["yaw"], -0.25)

    def test_lokal_menang_saat_qgc_diam(self):
        o = core.arb_manual2(False, 0, 0, True, 0.3, 0.7)
        self.assertEqual(o["source"], core.MANUAL_LOCAL)
        self.assertEqual(o["source_name"], "LOKAL")

    def test_konflik_dipaksa_netral(self):
        # Dua operator memegang bersamaan -> TANPA pemenang: netral + flag.
        o = core.arb_manual2(True, 1.0, 1.0, True, -1.0, -1.0)
        self.assertTrue(o["conflict"])
        self.assertEqual(o["source"], core.MANUAL_CONFLICT)
        self.assertEqual((o["surge"], o["yaw"]), (0.0, 0.0))

    def test_output_di_clamp(self):
        o = core.arb_manual2(True, 9.0, -9.0, False, 0, 0)
        self.assertEqual((o["surge"], o["yaw"]), (1.0, -1.0))


if __name__ == "__main__":
    unittest.main()

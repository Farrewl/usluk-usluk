"""Test KILL GUI (estop) tanpa hardware: alur _send_disarm.

GroundSimNavigator.__new__ dipakai agar tanpa YOLO/kamera/MAVLink. Master
MAVLink diganti stub (command_long_send tercatat, tak ada I/O).

Yang diuji (lih. app/simulator.py):
- kill_request True  -> MAV_CMD_COMPONENT_ARM_DISARM (400) param1=0 terkirim
- throttled: tak ada kiriman kedua dalam 2 detik
- kill_request False -> tidak ada kiriman sama sekali

Mode KILL itu sendiri (prioritas KILL > MANUAL > AUTO) sudah dibuktikan
oleh tests/test_core_mode_manager.py + test_core_arbitrator.py.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import time
import unittest

from app.simulator import GroundSimNavigator

MAV_CMD_COMPONENT_ARM_DISARM = 400


class _MavStub:
    """Meniru pymavlink master: command_long_send tercatat saja."""

    def __init__(self):
        self.calls = []
        self.target_system = 1
        self.target_component = 1

    class _Mav:
        def __init__(self, outer):
            self._outer = outer

        def command_long_send(self, *args):
            self._outer.calls.append(args)

    @property
    def mav(self):
        return self._Mav(self)


def _nav_bare(kill=True):
    nav = GroundSimNavigator.__new__(GroundSimNavigator)
    nav.kill_request = kill
    nav._disarm_sent_mono = 0.0
    nav.master = _MavStub()
    nav.mav = None
    return nav


class TestKillDisarm(unittest.TestCase):

    def test_kill_aktif_mengirim_disarm(self):
        nav = _nav_bare(kill=True)
        nav._send_disarm()
        self.assertEqual(len(nav.master.calls), 1,
                         "kill aktif harus mengirim 1x disarm")
        args = nav.master.calls[0]
        self.assertEqual(args[2], MAV_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual(args[4], 0, "param1=0 berarti DISARM")
        self.assertEqual(args[0], 1, "target_system dari master")

    def test_kill_throttle_tidak_banjiri(self):
        nav = _nav_bare(kill=True)
        nav._send_disarm()
        nav._send_disarm()          # masih dalam jendela 2 detik
        nav._send_disarm()
        self.assertEqual(len(nav.master.calls), 1,
                         "maks 1x per 2 detik")

    def test_kill_nonaktif_tidak_mengirim(self):
        nav = _nav_bare(kill=False)
        nav._send_disarm()
        self.assertEqual(len(nav.master.calls), 0)

    def test_kill_pulih_kemudian_disarm_lagi(self):
        nav = _nav_bare(kill=True)
        nav._send_disarm()
        self.assertEqual(len(nav.master.calls), 1)
        nav.kill_request = False    # dilepas operator
        nav._send_disarm()
        self.assertEqual(len(nav.master.calls), 1, "false = jangan kirim")


if __name__ == "__main__":
    unittest.main()
"""Test app/settings.py — pastikan:

  1. save_params() HANYA menulis parameter tuning (TUNING_PARAM_KEYS),
     bukan atribut internal seperti path model / port serial / host.
  2. Config() membaca tuning tersimpan & memakai default bila file kosong.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import copy
import json
import os
import tempfile
import unittest

from app import settings as cfg


class MockConfig:
    """Objek kecil meniru Config: campur param tuning + atribut mesin."""

    def __init__(self):
        self.ACCEPTANCE_RADIUS_M = 1.8          # tuning -> boleh disimpan
        self.THRUST_VALUE = 0.8                 # tuning
        self.YOLO_INFERENCE_SIZE = 320          # tuning
        self.VISION_ENABLED_LEGS = [1, 3, 5]    # tuning (list)
        self.MODEL_PATH = "/tmp/abs/weights/buoy.pt"   # mesin -> TIDAK boleh
        self.SERIAL_PORT = "COM8"               # mesin -> TIDAK boleh
        self.REDIS_HOST = "localhost"           # mesin -> TIDAK boleh
        self.CAMERA_INDEX = 0                   # mesin -> TIDAK boleh


class TestSaveParams(unittest.TestCase):

    def setUp(self):
        self._orig_file = cfg.TUNING_FILE
        fd, self.tmp_file = tempfile.mkstemp(suffix="_tuning.json")
        os.close(fd)
        cfg.TUNING_FILE = self.tmp_file

    def tearDown(self):
        cfg.TUNING_FILE = self._orig_file
        if os.path.exists(self.tmp_file):
            os.unlink(self.tmp_file)

    def _saved_dict(self):
        with open(self.tmp_file) as f:
            return json.load(f)

    def test_saves_only_tuning_keys(self):
        cfg.save_params(MockConfig())
        saved = self._saved_dict()
        self.assertEqual(saved["ACCEPTANCE_RADIUS_M"], 1.8)
        self.assertEqual(saved["THRUST_VALUE"], 0.8)
        self.assertEqual(saved["VISION_ENABLED_LEGS"], [1, 3, 5])
        # atribut mesin/path HARUS tidak muncul
        for banned in ("MODEL_PATH", "SERIAL_PORT", "REDIS_HOST", "CAMERA_INDEX"):
            self.assertNotIn(banned, saved, f"'{banned}' tidak boleh tersimpan!")

    def test_saved_keys_are_subset_of_allowlist(self):
        cfg.save_params(MockConfig())
        saved = self._saved_dict()
        self.assertTrue(set(saved) <= cfg.TUNING_PARAM_KEYS)

    def test_empty_config_saves_no_tuning_keys(self):
        cfg.save_params(type("Empty", (), {})())
        self.assertEqual(self._saved_dict(), {})


class TestConfigLoad(unittest.TestCase):

    def setUp(self):
        # File tuning sementara berisi 1 nilai; selainnya default.
        fd, self.tmp_file = tempfile.mkstemp(suffix="_tuning.json")
        os.close(fd)
        with open(self.tmp_file, "w") as f:
            json.dump({"ACCEPTANCE_RADIUS_M": 3.5}, f)
        self._orig_file = cfg.TUNING_FILE
        cfg.TUNING_FILE = self.tmp_file

    def tearDown(self):
        cfg.TUNING_FILE = self._orig_file
        if os.path.exists(self.tmp_file):
            os.unlink(self.tmp_file)

    def test_loading_uses_saved_value_and_defaults(self):
        c = cfg.Config()
        self.assertEqual(c.ACCEPTANCE_RADIUS_M, 3.5)      # terbaca dari file
        self.assertEqual(c.THRUST_VALUE, 0.9)             # default (tidak ada di file)
        self.assertEqual(c.MODEL_PATH,                    # path tetap relatif root
                         os.path.join(cfg.WEIGHTS_DIR, "buoy.pt"))
        self.assertEqual(c.SERIAL_PORT, "COM8")

    def test_saved_keys_must_be_reloadable(self):
        # Round-trip: kunci yang boleh disimpan HARUS bisa dimuat kembali
        # oleh Config (tidak ada key loner yang tersimpan tapi tak terbaca).
        cfg.save_params(MockConfig())
        c2 = cfg.Config()
        for key in ("ACCEPTANCE_RADIUS_M", "THRUST_VALUE",
                    "YOLO_INFERENCE_SIZE", "VISION_ENABLED_LEGS"):
            self.assertTrue(hasattr(c2, key), f"Config tidak punya {key}")


if __name__ == "__main__":
    unittest.main()
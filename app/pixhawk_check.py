"""PixhawkCheck — verifikasi parameter failsafe wajib saat startup.

Otoritas: QGC / Mission Planner pegang ubah parameter. Modul ini HANYA
membaca (PARAM_REQUEST_READ) lalu melaporkan pelanggaran terhadap spec di
`config/pixhawk_required.txt` (ArduPilot) atau `config/pixhawk_required_px4.txt`
(PX4). Firmware dideteksi dari HEARTBEAT (field `autopilot`) supaya nama
parameter yang dipakai sesuai — PX4 dan ArduPilot punya nama berbeda.

Alur (dipanggil dari thread background navigator saat mav tersambung):
  1. deteksi firmware (autopilot id) -> pilih file spec
  2. kirim PARAM_REQUEST_READ untuk tiap nama (lewat master.mav.*)
  3. baca balasan PARAM_VALUE sampai timeout_s / semua lengkap
  4. bandingkan [min..max]; 'unread' bila Pixhawk tidak menjawab nama itu

Hasil: list dict {name, min, max, critical, value, status}
  status: 'ok' | 'violation' | 'unread'
"""
import os
import time
from struct import pack, unpack

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEFAULT_SPEC_PATH = os.path.join(_REPO_ROOT, "config", "pixhawk_required.txt")

# MAV_PARAM_TYPE -> (kode struct little-endian, ukuran byte). PX4 mengemas
# parameter INTEGER ke dalam bit float `param_value`; wajib di-decode balik
# atau nilainya muncul sebagai denormal (mis. int 1 -> 2.8e-45).
_PARAM_FMT = {1: ("B", 1), 2: ("b", 1), 3: ("H", 2), 4: ("h", 2),
              5: ("I", 4), 6: ("i", 4)}


def _decode_param(msg):
    """Ambil nilai numerik dari PARAM_VALUE, decode tipe integer bila perlu."""
    val = float(getattr(msg, "param_value", 0.0))
    ptype = int(getattr(msg, "param_type", 0) or 0)
    spec = _PARAM_FMT.get(ptype)
    if spec:
        fmt, size = spec
        try:
            return float(unpack("<" + fmt, pack("<f", val)[:size])[0])
        except Exception:
            return val
    return val

# ID autopilot MAVLink -> (nama tampil, file spec).
AUTOPILOT_FW = {
    3: ("ArduPilot", "pixhawk_required.txt"),
    12: ("PX4", "pixhawk_required_px4.txt"),
}


def firmware_name(autopilot):
    """Nama firmware dari id autopilot, mis. 'PX4' / 'ArduPilot' / '?'."""
    try:
        if autopilot is None:
            return "?"
        return AUTOPILOT_FW.get(int(autopilot), ("?", None))[0]
    except Exception:
        return "?"


def spec_path_for(autopilot=None):
    """Path file spec yang sesuai firmware. Fallback ke spec ArduPilot."""
    name = "pixhawk_required.txt"
    try:
        if autopilot is not None:
            name = AUTOPILOT_FW.get(int(autopilot), (None, name))[1] or name
    except Exception:
        pass
    return os.path.join(_REPO_ROOT, "config", name)


def detect_autopilot(master, timeout_s=3.0):
    """Baca id autopilot dari HEARTBEAT terakhir (blocking singkat).

    Return int (3=ArduPilot, 12=PX4, ...) atau None bila tak ada heartbeat.
    """
    val = getattr(master, "autopilot", None)
    if val is not None:
        try:
            return int(val)
        except Exception:
            pass
    try:
        hb = master.recv_match(type="HEARTBEAT", blocking=True,
                               timeout=timeout_s)
        if hb is not None:
            return int(getattr(hb, "autopilot", -1))
    except Exception:
        pass
    return None


def _request_param(master, name):
    """Minta satu parameter; toleran perbedaan API pymavlink.

    Wajib lewat `master.mav.param_request_read_send` (bukan `master.`),
    dan `param_id` harus bytes di pymavlink modern; beberapa versi/objek
    lain menerima str atau `master.param_fetch_one`.
    """
    ts = getattr(master, "target_system", 1)
    tc = getattr(master, "target_component", 1)
    mav = getattr(master, "mav", None)
    if mav is not None and hasattr(mav, "param_request_read_send"):
        for enc in (name.encode(), name):
            try:
                mav.param_request_read_send(ts, tc, enc, -1)
                return True
            except Exception:
                continue
    if hasattr(master, "param_fetch_one"):
        try:
            master.param_fetch_one(name)
            return True
        except Exception:
            pass
    return False


def _as_float(x):
    """'-' / 'inf' -> None (tanpa batas)."""
    return None if x in ("-", "inf", "") else float(x)


def load_spec(path=None):
    """Baca config/pixhawk_required.txt -> list spec dict."""
    path = path or _DEFAULT_SPEC_PATH
    spec = []
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            critical = (len(parts) > 3
                        and parts[3].strip().lower() in ("y", "yes", "1"))
            spec.append({
                "name": parts[0],
                "min": _as_float(parts[1]),
                "max": _as_float(parts[2]),
                "critical": critical,
            })
    return spec


def check_params(master, spec, poll_s=0.05, timeout_s=3.0):
    """Kirim PARAM_REQUEST_READ tiap nama; bandingkan nilai balasan.

    `master` = objek ala pymavlink: punya target_system, target_component,
    param_request_read_send(...), recv_match(type=..., blocking=...).
    """
    need = {p["name"]: p for p in spec}
    got = {}
    for name in need:
        _request_param(master, name)
    deadline = time.time() + timeout_s
    while time.time() < deadline and len(got) < len(need):
        try:
            msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        except Exception:
            msg = None
        if msg is not None:
            got[str(getattr(msg, "param_id", "")).strip()] = _decode_param(msg)
        else:
            time.sleep(poll_s)

    out = []
    for name, p in need.items():
        if name not in got:
            out.append(dict(p, value=None, status="unread"))
            continue
        val = got[name]
        ok = True
        if p["min"] is not None and val < p["min"] - 1e-9:
            ok = False
        if p["max"] is not None and val > p["max"] + 1e-9:
            ok = False
        out.append(dict(p, value=val,
                        status="ok" if ok else "violation"))
    return out


def summarize(results):
    """Ringkas hasil: 'OK' atau daftar pelanggaran satu-baris."""
    bad = [r for r in results if r["status"] != "ok"]
    if not bad:
        return "OK"
    parts = []
    for r in bad:
        tag = "KRITIS" if r["critical"] else "advisory"
        if r["status"] == "unread":
            parts.append(f"{r['name']} TIDAK DIJAWAB ({tag})")
        else:
            parts.append(
                f"{r['name']}={r['value']} luar [{r['min']}..{r['max']}] ({tag})")
    return "; ".join(parts)
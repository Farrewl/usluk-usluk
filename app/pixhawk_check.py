"""PixhawkCheck — verifikasi parameter failsafe wajib saat startup.

Otoritas: QGC / Mission Planner pegang ubah parameter. Modul ini HANYA
membaca (PARAM_REQUEST_READ) lalu melaporkan pelanggaran terhadap spec di
`config/pixhawk_required.txt`. Tidak pernah menulis parameter.

Alur (dipanggil dari thread background simulator saat mav tersambung):
  1. baca spec  config/pixhawk_required.txt
  2. kirim PARAM_REQUEST_READ untuk tiap nama
  3. baca balasan PARAM_VALUE sampai timeout_s / semua lengkap
  4. bandingkan [min..max]; 'unread' bila Pixhawk tidak menjawab nama itu

Hasil: list dict {name, min, max, critical, value, status}
  status: 'ok' | 'violation' | 'unread'
"""
import os
import time

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEFAULT_SPEC_PATH = os.path.join(_REPO_ROOT, "config", "pixhawk_required.txt")


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
        try:
            master.param_request_read_send(
                master.target_system, master.target_component, name, -1)
        except Exception:
            pass
    deadline = time.time() + timeout_s
    while time.time() < deadline and len(got) < len(need):
        try:
            msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        except Exception:
            msg = None
        if msg is not None:
            got[str(getattr(msg, "param_id", "")).strip()] = \
                float(getattr(msg, "param_value", 0.0))
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
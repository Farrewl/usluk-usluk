#!/usr/bin/env python3
"""scripts/monitor_read.py — lihat data ekspor monitoring tanpa web.

Pemakai: developer dashboard web (repo terpisah) / debug lapangan. Membaca
file bridge di `data/monitor/` (lihat README.md bagian Ekspor Monitoring).

Contoh:
    python3 scripts/monitor_read.py               # ringkas telemetry.json
    python3 scripts/monitor_read.py --json        # dump JSON penuh
    python3 scripts/monitor_read.py --watch       # refresh tiap 1 s
    python3 scripts/monitor_read.py --events      # 10 baris events.jsonl terakhir
    python3 scripts/monitor_read.py --photos      # indeks foto
    python3 scripts/monitor_read.py --dir <path>  # folder lain
"""

import argparse
import json
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _fmt(v, nd=2):
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def show_telemetry(data):
    ident = data.get("identity", {})
    pose = data.get("pose", {})
    motion = data.get("motion", {})
    power = data.get("power", {})
    link = data.get("link", {})
    mission = data.get("mission", {})
    vision = data.get("vision", {})
    print(f"== {ident.get('team','?')} / {ident.get('pt','?')} "
          f"[lintasan {ident.get('track','?')}] ==")
    print(f"  ts      : {_fmt(data.get('ts'))}  uptime {_fmt(data.get('health',{}).get('uptime_s'),1)} s")
    print(f"  pose    : lat {_fmt(pose.get('lat'),6)} lon {_fmt(pose.get('lon'),6)}"
          f"  yaw {_fmt(pose.get('yaw_deg'))} pitch {_fmt(pose.get('pitch_deg'))}"
          f" roll {_fmt(pose.get('roll_deg'))}")
    print(f"  motion  : {_fmt(motion.get('sog_kn'))} kn / {_fmt(motion.get('sog_kmh'))} km/h"
          f"  COG {_fmt(motion.get('cog_deg'))}")
    print(f"  power   : {_fmt(power.get('battery_pct'))} %  {_fmt(power.get('voltage_v'))} V"
          f"  {_fmt(power.get('current_a'))} A")
    print(f"  link    : mavlink_ok={link.get('mavlink_ok')} port={link.get('port')}"
          f"  reason={link.get('reason')!r}  qgc={link.get('qgc_active')}")
    print(f"  mission : {mission.get('op_mode')} / {mission.get('state')}"
          f"  wp {mission.get('wp_idx')}/{mission.get('wp_total')}"
          f"  d={_fmt(mission.get('dist_to_wp_m'))} m"
          f"  failsafe={mission.get('failsafe_active')}")
    flags = mission.get("flags", {})
    print(f"  progres : balls {flags.get('balls')} surface={flags.get('surface')}"
          f" under={flags.get('under')} dock={flags.get('dock')}")
    models = vision.get("models", {})
    if models:
        parts = [f"{k}:{'Y' if v.get('detected') else '-'}{v.get('n',0)}"
                 for k, v in models.items()]
        print(f"  visi    : gate_passed={vision.get('gate_passed')}  " + " ".join(parts))
    print(f"  params  : {len(data.get('params', {}))} parameter kapal ada di JSON")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(_ROOT, "data", "monitor"))
    ap.add_argument("--json", action="store_true", help="dump JSON penuh")
    ap.add_argument("--watch", action="store_true", help="refresh tiap 1 s")
    ap.add_argument("--events", action="store_true", help="10 baris events terakhir")
    ap.add_argument("--photos", action="store_true", help="indeks foto")
    args = ap.parse_args()

    if args.events:
        path = os.path.join(args.dir, "events.jsonl")
        if not os.path.exists(path):
            print(f"belum ada: {path}")
            return 0
        with open(path, "r", encoding="utf-8") as f:
            lines = f.readlines()[-10:]
        for ln in lines:
            print(ln.rstrip())
        return 0

    if args.photos:
        path = os.path.join(args.dir, "photos.json")
        if not os.path.exists(path):
            print(f"belum ada: {path}")
            return 0
        for p in _read_json(path):
            print(f"  {p.get('kind','?'):<8} {p.get('time','')}  "
                  f"{p.get('coord_dd','')}  {p.get('path','')}")
        return 0

    tpath = os.path.join(args.dir, "telemetry.json")
    while True:
        if not os.path.exists(tpath):
            print(f"belum ada: {tpath}\n(app kapal berjalan? MONITOR_ENABLED?)")
        else:
            try:
                data = _read_json(tpath)
            except Exception as e:
                print(f"gagal baca (mungkin sedang ditulis): {e}")
                data = None
            if data is not None:
                if args.json:
                    print(json.dumps(data, indent=2, ensure_ascii=False))
                else:
                    show_telemetry(data)
        if not args.watch:
            return 0
        print("-" * 60)
        time.sleep(1.0)


if __name__ == "__main__":
    sys.exit(main())

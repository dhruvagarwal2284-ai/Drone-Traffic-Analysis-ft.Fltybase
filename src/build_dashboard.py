"""Inject the analysis bundle into the dashboard template.

The dashboard is a single self-contained HTML file: all data is embedded as
JSON, so it opens straight from disk with no server. The only external files
it references are the conflict evidence clips, resolved relative to the HTML
via `clip_base` ("clips/" for out/, "" for demo/ where they sit alongside).
"""
from __future__ import annotations

import json
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parent.parent


def build(tag: str = "intersection", dest: Path | None = None,
          clip_base: str = "clips/") -> Path:
    tpl = (ROOT / "src" / "dashboard_template.html").read_text(encoding="utf-8")
    bundle = json.loads((ROOT / "out" / f"report_{tag}.json").read_text(encoding="utf-8"))

    # fold in the corridor's detector-free signal for comparison, if present
    cp = ROOT / "out" / "congestion_corridor.parquet"
    if cp.exists():
        c = pd.read_parquet(cp)
        cyc = ROOT / "out" / "cycle_corridor.json"
        k = json.loads(cyc.read_text(encoding="utf-8")) if cyc.exists() else {}
        bundle["corridor"] = {
            "t": [round(v, 1) for v in c.t],
            "queue_index": [round(v, 4) for v in c.queue_index],
            "motion": [round(v, 3) for v in c.motion],
            "occupancy": [round(v, 3) for v in c.occupancy],
            "cycle_s": k.get("cycle_s"), "peak_autocorr": k.get("peak_autocorr"),
        }
    mp = ROOT / "out" / f"mapnative_{tag}.json"
    if mp.exists():
        bundle["mapnative"] = json.loads(mp.read_text(encoding="utf-8"))
    bundle["clip_base"] = clip_base

    # keep the JSON inert inside <script type="application/json">
    data = json.dumps(bundle).replace("</", "<\\/")

    dest = dest or ROOT / "out" / f"dashboard_{tag}.html"
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(tpl.replace("__DATA__", data), encoding="utf-8")
    return dest


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("tag", nargs="?", default="intersection")
    ap.add_argument("--dest", type=Path, default=None,
                    help="output HTML (default out/dashboard_<tag>.html)")
    ap.add_argument("--clip-base", default="clips/",
                    help="path from the HTML to the conflict clips ('' when they sit alongside)")
    a = ap.parse_args()
    d = build(a.tag, a.dest, a.clip_base)
    print(f"-> {d}  ({d.stat().st_size/1e6:.2f} MB)")

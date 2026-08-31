"""Inject the analysis bundle into the dashboard template."""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def build(tag: str = "intersection") -> Path:
    import json
    import pandas as pd

    tpl = (ROOT / "src" / "dashboard_template.html").read_text(encoding="utf-8")
    bundle = json.loads((ROOT / "out" / f"report_{tag}.json").read_text(encoding="utf-8"))

    # fold in the corridor's detector-free signal for comparison, if present
    cp = ROOT / "out" / "congestion_corridor.parquet"
    if cp.exists():
        c = pd.read_parquet(cp)
        cyc = ROOT / "out" / "cycle_corridor.json"
        k = json.loads(cyc.read_text()) if cyc.exists() else {}
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

    data = json.dumps(bundle)

    # keep the JSON inert inside <script type="application/json">
    data = data.replace("</", "<\\/")

    dest = ROOT / "out" / f"dashboard_{tag}.html"
    dest.write_text(tpl.replace("__DATA__", data), encoding="utf-8")
    return dest


if __name__ == "__main__":
    import sys

    tag = sys.argv[1] if len(sys.argv) > 1 else "intersection"
    d = build(tag)
    print(f"-> {d}  ({d.stat().st_size/1e6:.2f} MB)")

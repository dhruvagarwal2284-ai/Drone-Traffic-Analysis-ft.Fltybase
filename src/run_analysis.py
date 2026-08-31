"""End-to-end: detections -> trajectories -> insight bundle for the dashboard.

Writes out/report_<tag>.json, plus the trajectory Parquet that is the reusable
data product, plus short evidence clips for the worst conflicts.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))

import aggregate as agg        # noqa: E402
import attributes as attrmod   # noqa: E402
import conflicts as cf          # noqa: E402
import objects as objmod        # noqa: E402
import insights as ins          # noqa: E402
import telemetry as tel         # noqa: E402
import trajectories as tj       # noqa: E402
from geometry import GroundPlane  # noqa: E402


def _clean(o):
    """Make numpy/pandas types JSON-serialisable."""
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return None if not np.isfinite(o) else round(float(o), 4)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, float):
        return None if not np.isfinite(o) else round(o, 4)
    if isinstance(o, pd.DataFrame):
        return _clean(o.to_dict("records"))
    return o


def export_clips(top: pd.DataFrame, traj: pd.DataFrame, window: Path,
                 outdir: Path, n: int = 6, pad: float = 3.0, t0: float = 240.0):
    """Cut a short clip around each top conflict, cropped to where it happened."""
    outdir.mkdir(parents=True, exist_ok=True)
    made = []
    for i, r in top.head(n).iterrows():
        sub = traj[(traj.track_id.isin([r.id_a, r.id_b])) &
                   (traj.t.between(r.t - 0.6, r.t + 0.6))]
        if sub.empty:
            continue
        u, v = float(sub.u_px.mean()), float(sub.v_px.mean())
        cw, ch = 1280, 720
        x = int(np.clip(u - cw / 2, 0, 3840 - cw))
        y = int(np.clip(v - ch / 2, 0, 2160 - ch))
        start = max(0.0, r.t - t0 - pad)
        dest = outdir / f"conflict_{i+1:02d}_{r.measure}_{r.value:.2f}s.mp4"
        cmd = ["ffmpeg", "-v", "error", "-ss", f"{start:.2f}", "-i", str(window),
               "-t", f"{2*pad:.2f}", "-vf", f"crop={cw}:{ch}:{x}:{y}",
               "-c:v", "libx264", "-crf", "23", "-preset", "veryfast",
               "-pix_fmt", "yuv420p", str(dest), "-y"]
        try:
            subprocess.run(cmd, check=True, timeout=120)
            made.append({"file": dest.name, "measure": r.measure,
                         "value": round(float(r.value), 2),
                         "cls_a": r.cls_a, "cls_b": r.cls_b,
                         "t": round(float(r.t), 1)})
        except Exception as e:                       # noqa: BLE001
            print(f"  clip {i} failed: {e}")
    return made


def main(tag="intersection", video="Intersection_Merged-002",
         t0=240.0, fps=10.0):
    out = ROOT / "out"

    det = pd.read_parquet(out / f"detections_{tag}.parquet")
    tm = tel.load(ROOT / f"{video}.MP4", out / "cache")
    row = tm.iloc[int(t0 * 30000 / 1001)]

    gp = GroundPlane.from_telemetry(3840, 2160, row.focal_len, row.rel_alt,
                                    row.gb_yaw, row.gb_pitch, row.gb_roll,
                                    zoom=row.dzoom)
    gp.set_origin(np.array([[1920.0, 1080.0]]))

    print("building trajectories ...", flush=True)
    traj, cal = tj.build(det, gp, fps)
    traj.to_parquet(out / f"trajectories_{tag}.parquet", index=False)
    print(f"  {traj.track_id.nunique()} tracks, {len(traj)} rows", flush=True)

    print("scene geometry + movements ...", flush=True)
    legs = ins.find_legs(traj)
    moves = ins.assign_movements(traj, legs)
    moves = ins.delay(traj, moves)
    trav = moves[moves.traversed]            # only these have a real movement

    print("conflicts ...", flush=True)
    pet = cf.pet(traj)
    ttc = cf.ttc(traj)
    sev = cf.severity(pet, ttc)
    H, xe, ye = cf.heatmap(sev) if not sev.empty else (np.zeros((60, 60)), None, None)

    print("object-level attributes ...", flush=True)
    ap = out / f"attributes_{tag}.parquet"
    attrs = pd.read_parquet(ap) if ap.exists() else None
    objs = objmod.build(traj, moves, attrs, sev, fps)
    objs.to_parquet(out / f"objects_{tag}.parquet", index=False)

    print("aggregate insight ...", flush=True)
    lanes = [agg.lanes_for_leg(traj, legs, i) for i in range(len(legs["names"]))]
    lanes = [L for L in lanes if L]
    queues = {}
    for i, nm in enumerate(legs["names"]):
        Q = agg.queue_length(traj, legs, i)
        if not Q.empty:
            queues[nm] = {"t": Q.t.tolist(), "queue_m": Q.queue_m.tolist(),
                          "max_m": round(float(Q.queue_m.max()), 1),
                          "p85_m": round(float(Q.queue_m.quantile(.85)), 1),
                          "mean_m": round(float(Q.queue_m.mean()), 1)}
    fd = agg.edie(traj, legs, axis_leg=1)
    ic = agg.interval_counts(moves)
    dv = agg.directional_volumes(moves)

    print("anomalies ...", flush=True)
    anom = ins.anomalies(traj)

    clips = []
    if not sev.empty:
        print("evidence clips ...", flush=True)
        clips = export_clips(sev, traj, out / f"window_{tag}.mp4",
                             out / "clips", n=6, t0=t0)

    # a decimated sample of paths, for the "discovered network" plot
    ids = traj.track_id.drop_duplicates().sample(
        min(400, traj.track_id.nunique()), random_state=0)
    paths = []
    for tid, g in traj[traj.track_id.isin(ids)].groupby("track_id"):
        g = g.iloc[::4]
        if len(g) < 3:
            continue
        paths.append({"cls": g.cls.iloc[0],
                      "x": [round(v, 1) for v in g.x_m],
                      "y": [round(v, 1) for v in g.y_m]})

    cong, cyc = None, {}
    cp = out / f"congestion_{tag}.parquet"
    if cp.exists():
        c = pd.read_parquet(cp)
        cong = {"t": c.t.round(1).tolist(),
                "motion": c.motion.round(3).tolist(),
                "occupancy": c.occupancy.round(3).tolist(),
                "queue_index": c.queue_index.round(4).tolist()}
        cj = out / f"cycle_{tag}.json"
        if cj.exists():
            k = json.loads(cj.read_text())
            cyc = {"cycle_s": k.get("cycle_s"), "peak_autocorr": k.get("peak_autocorr")}

    grade = (sev.groupby("grade", observed=False).size().to_dict()
             if not sev.empty else {})

    bundle = {
        "meta": {
            "tag": tag, "video": video,
            "site": {"lat": float(row.lat), "lon": float(row.lon),
                     "rel_alt_m": float(row.rel_alt),
                     "gimbal": {"yaw": float(row.gb_yaw), "pitch": float(row.gb_pitch)}},
            "captured": str(row.ts),
            "window": {"t0_s": t0, "dur_s": float(traj.t.max() - traj.t.min()),
                       "fps": fps},
        },
        "quality": ins.quality(traj, moves, cal),
        "legs": {"names": legs["names"],
                 "bearings": [round(b, 1) for b in legs["bearings"]],
                 "centre": [round(v, 2) for v in legs["centre"]]},
        "counts": ins.counts(moves),
        "od": ins.od_matrix(trav).to_dict(),
        "turns": trav.groupby(["turn"]).size().to_dict(),
        "traversing_tracks": int(len(trav)),
        "movement_top": (trav.groupby("movement").size()
                         .sort_values(ascending=False).head(12).to_dict()),
        "flow": ins.flow_series(moves),
        "speed": ins.speed_stats(traj),
        "speed_hist": {
            c: np.histogram(g.speed_kph.clip(0, 80), bins=32, range=(0, 80))[0].tolist()
            for c, g in traj[traj.speed_kph > 1].groupby("cls")},
        "delay": {
            "mean_stopped_s": round(float(moves.stopped_s.mean()), 2),
            "p85_stopped_s": round(float(moves.stopped_s.quantile(.85)), 2),
            "frac_stopped": round(float((moves.stopped_s > 1).mean()), 3),
            "by_movement": trav.groupby("movement")["stopped_s"].mean()
                                .sort_values(ascending=False).head(10).round(1).to_dict()},
        "conflicts": {
            "n_total": int(len(sev)),
            "by_grade": grade,
            "top": sev.head(25),
            "by_class_pair": (sev[sev.value < 3.0]
                              .assign(pair=lambda d: d.cls_a + " / " + d.cls_b)
                              .groupby("pair").size().sort_values(ascending=False)
                              .head(10).to_dict()) if not sev.empty else {},
            "heatmap": {"z": H.T.tolist(),
                        "extent": [-70, 70, -70, 70]},
            "clips": clips,
        },
        "anomalies": {"n": int(len(anom)),
                      "by_kind": anom.kind.value_counts().to_dict() if not anom.empty else {},
                      "items": anom.sort_values("t").head(40) if not anom.empty else []},
        "objects": {
            "n": int(len(objs)),
            "validation": objmod.validate(objs),
            "colour": (objs.colour.value_counts().to_dict()
                       if "colour" in objs else {}),
            "colour_hex": ({c: g.hex.mode().iloc[0]
                            for c, g in objs.groupby("colour") if len(g)}
                           if "colour" in objs else {}),
            "body_type": (objs.body_type.value_counts().to_dict()
                          if "body_type" in objs else {}),
            "size_class": (objs.size_class.value_counts().to_dict()
                           if "size_class" in objs else {}),
            "dims_by_body": ({k: {"L": round(float(v.L_m.median()), 2),
                                  "W": round(float(v.W_m.median()), 2),
                                  "n": int(len(v))}
                              for k, v in objs.groupby("body_type")
                              if v.L_m.notna().any()}
                             if "body_type" in objs else {}),
            "kinematics_by_body": ({k: {"v_mean_kph": round(float(v.v_mean_kph.median()), 1),
                                        "v_max_kph": round(float(v.v_max_kph.quantile(.85)), 1),
                                        "a_max": round(float(v.a_max_ms2.median()), 2),
                                        "a_min": round(float(v.a_min_ms2.median()), 2)}
                                    for k, v in objs.groupby("body_type") if len(v) >= 5}
                                   if "body_type" in objs else {}),
            "sample": objs.sort_values("distance_m", ascending=False)
                          .head(24)[[c for c in
                              ["track_id", "descriptor", "colour", "hex", "body_type",
                               "size_class", "L_m", "W_m", "v_mean_kph", "v_max_kph",
                               "a_min_ms2", "distance_m", "movement", "n_conflicts"]
                              if c in objs]],
            "plate_feasibility": attrmod.plate_requirements(),
        },
        "aggregate": {
            "speed_field": agg.speed_field(traj),
            "lanes": lanes,
            "queues": queues,
            "fundamental": {
                "summary": agg.summarise(fd),
                "points": ([{"dir": r.dir, "s": float(r.sbin), "t": float(r.tbin),
                             "q": round(float(r.flow_vph), 1),
                             "k": round(float(r.density_vpkm), 2),
                             "v": round(float(r.speed_kph), 1),
                             "occ": round(float(r.occupancy_pct), 1)}
                            for r in fd.itertuples()] if not fd.empty else []),
            },
            "interval_counts": (ic.to_dict("records") if not ic.empty else []),
            "directional_volumes": (dv.to_dict("records") if not dv.empty else []),
        },
        "congestion": cong, "cycle": cyc,
        "paths": paths,
    }

    dest = out / f"report_{tag}.json"
    dest.write_text(json.dumps(_clean(bundle)))
    print(f"\n-> {dest}  ({dest.stat().st_size/1e6:.1f} MB)")

    print("\n== quality ==")
    print(json.dumps(_clean(bundle["quality"]), indent=2))
    print("\n== legs ==", legs["names"], [round(b) for b in legs["bearings"]])
    print("\n== counts ==")
    print(ins.counts(moves).to_string(index=False))
    print("\n== conflicts by grade ==", grade)
    if not sev.empty:
        print(sev.head(10)[["measure", "value", "cls_a", "cls_b", "t"]].to_string(index=False))
    return bundle


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="intersection")
    ap.add_argument("--video", default="Intersection_Merged-002")
    ap.add_argument("--t0", type=float, default=240.0)
    ap.add_argument("--fps", type=float, default=10.0)
    a = ap.parse_args()
    main(a.tag, a.video, a.t0, a.fps)

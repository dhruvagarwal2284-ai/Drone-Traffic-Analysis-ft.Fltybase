"""Metric-frame multi-object tracker: Step B of the tracker re-architecture.

Step A (detect.py) already back-projected every detection to ground-plane
metres (`x_m,y_m`) and a calibrated footprint (`w_m,h_m`) -- this stage
associates *those*, not pixels, so track identity survives the tree
occlusion and the steep-oblique parallax that image-space ByteTrack
(track.py) cannot see past.

Per-track state: constant-velocity Kalman filter (x, y, vx, vy) in metres,
majority-vote class, hit/miss counters. Two-stage matching per frame
(ByteTrack-style: high-confidence detections first, then low-confidence),
solved with `scipy.optimize.linear_sum_assignment`. Gating uses the filter's
own Mahalanobis distance -- its covariance inflates every frame a track goes
unmatched, so the acceptance radius grows through an occlusion on its own,
no separate "growing gate" schedule needed. Cost adds a class-pair term
(cheap for motorcycle<->bicycle, pricier for car<->motorcycle) and a size
term from `w_m,h_m` (the `--embed` HSV vector is optional and unavailable in
the `detraw_intersection.parquet` this step was told to use, so measured
footprint size stands in as the cheap appearance cue).

Output matches track.py's schema (fi, t, track_id, cls, conf, x1..y2,
u_px, v_px) so `run_analysis.py --tag <tag>` runs unchanged. x_m/y_m/w_m/h_m
are dropped from the output -- trajectories.py recomputes them itself from
x1..y2 via its own calibrate()/_box_ground(), same as it does for track.py's
output today.
"""
from __future__ import annotations

from collections import Counter
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment

BIG = 1.0e6  # cost sentinel for a gated-out (infeasible) pair

# Ordinal size/kind group per class -- distance between groups drives the
# class-mismatch cost, so motorcycle<->bicycle (same group) is free and
# car<->motorcycle (one group apart) costs lambda_cls, without hand-writing
# a full pairwise cost table.
CLASS_GROUP = {
    "person": 0,
    "bicycle": 1, "motorcycle": 1,
    "car": 2, "autorickshaw": 2,
    "truck": 3, "bus": 3,
}

DEFAULT_CFG = dict(
    max_lost_frames=60,   # 6 s @ 10 fps
    min_hits=3,
    gate_chi2=9.21,        # chi-square 99% cutoff, 2 dof
    lambda_cls=1.0,
    lambda_app=0.5,
    conf_high=0.5,
    q=1.5,                 # process noise, accel std, m/s^2 (matches trajectories.py)
    r=0.35,                # measurement noise, position std, m (matches trajectories.py)
)


def _kf_matrices(dt: float, q: float, r: float):
    F = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]], dtype=float)
    g = np.array([dt ** 2 / 2, dt])
    Qb = np.outer(g, g) * q ** 2
    Q = np.zeros((4, 4))
    Q[:2, :2] = Qb
    Q[2:, 2:] = Qb
    H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]], dtype=float)
    R = np.eye(2) * r ** 2
    return F, Q, H, R


def _group_dist(cls_a: str, cls_b: str) -> float:
    return float(abs(CLASS_GROUP.get(cls_a, 2) - CLASS_GROUP.get(cls_b, 2)))


class Track:
    """One tracked object. `pending` buffers rows until `min_hits` confirms it,
    so a detection that never confirms leaves nothing in the output."""

    def __init__(self, tid: int, x_m: float, y_m: float, wh: tuple[float, float], r: float):
        self.id = tid
        self.x = np.array([x_m, 0.0, y_m, 0.0])
        self.P = np.diag([r ** 2, 25.0, r ** 2, 25.0])
        self.cls_counts: Counter = Counter()
        self.hits = 0
        self.time_since_update = 0
        self.confirmed = False
        self.last_wh = wh
        self.pending: list[dict] = []

    @property
    def cls(self) -> str:
        return self.cls_counts.most_common(1)[0][0]


def _record_hit(tr: Track, row: dict, min_hits: int, out_rows: list[dict]) -> None:
    if tr.confirmed:
        out_rows.append(row)
        return
    tr.pending.append(row)
    if tr.hits >= min_hits:
        tr.confirmed = True
        out_rows.extend(tr.pending)
        tr.pending = []


def _row_dict(fi: int, tid: int, cls: str, det: pd.Series) -> dict:
    return {
        "fi": fi, "t": float(det.t), "track_id": tid, "cls": cls, "conf": float(det.conf),
        "x1": float(det.x1), "y1": float(det.y1), "x2": float(det.x2), "y2": float(det.y2),
        "u_px": float(det.u_px), "v_px": float(det.v_px),
    }


def _apply_match(tr: Track, det: pd.Series, fi: int, H, R, cfg: dict, out_rows: list[dict]) -> None:
    z = np.array([det.x_m, det.y_m])
    y = z - H @ tr.x
    S = H @ tr.P @ H.T + R
    K = tr.P @ H.T @ np.linalg.inv(S)
    tr.x = tr.x + K @ y
    tr.P = (np.eye(4) - K @ H) @ tr.P
    tr.hits += 1
    tr.time_since_update = 0
    tr.cls_counts[det.cls] += 1
    tr.last_wh = (det.w_m, det.h_m)
    _record_hit(tr, _row_dict(fi, tr.id, tr.cls, det), cfg["min_hits"], out_rows)


def _associate(tracks: dict, cand_ids: list, dets: pd.DataFrame, H, R, cfg: dict):
    """Gate + cost + Hungarian match `cand_ids` (track ids) against `dets`.

    Returns (matches as [(track_id, positional_det_index), ...],
             unmatched track ids, unmatched detections DataFrame).
    """
    n_t, n_d = len(cand_ids), len(dets)
    if n_t == 0 or n_d == 0:
        return [], list(cand_ids), dets

    det_xy = dets[["x_m", "y_m"]].to_numpy()
    det_wh = dets[["w_m", "h_m"]].to_numpy()
    det_cls = dets["cls"].to_numpy()

    cost = np.full((n_t, n_d), BIG)
    for i, tid in enumerate(cand_ids):
        tr = tracks[tid]
        pred = H @ tr.x
        S = H @ tr.P @ H.T + R
        Sinv = np.linalg.inv(S)
        diff = det_xy - pred
        d2 = np.einsum("ij,jk,ik->i", diff, Sinv, diff)
        feasible = d2 < cfg["gate_chi2"]
        if not feasible.any():
            continue
        pos_dist = np.linalg.norm(diff, axis=1)
        cls_cost = cfg["lambda_cls"] * np.array([_group_dist(tr.cls, c) for c in det_cls])
        app_cost = cfg["lambda_app"] * np.linalg.norm(det_wh - np.asarray(tr.last_wh), axis=1)
        row_cost = pos_dist + cls_cost + app_cost
        cost[i, feasible] = row_cost[feasible]

    row_ind, col_ind = linear_sum_assignment(cost)
    matches, matched_t, matched_d = [], set(), set()
    for r, c in zip(row_ind, col_ind):
        if cost[r, c] < BIG:
            matches.append((cand_ids[r], int(c)))
            matched_t.add(cand_ids[r])
            matched_d.add(c)

    unmatched_tracks = [tid for tid in cand_ids if tid not in matched_t]
    unmatched_dets = dets.iloc[[i for i in range(n_d) if i not in matched_d]]
    return matches, unmatched_tracks, unmatched_dets


def run(det: pd.DataFrame, fps: float, cfg: dict = DEFAULT_CFG) -> pd.DataFrame:
    """Associate detections into tracks. `det` needs fi,t,cls,conf,x1..y2,
    u_px,v_px,x_m,y_m,w_m,h_m (detect.py's detraw schema). One row per frame
    a track was actually matched -- gaps are left for trajectories.py to
    interpolate, exactly like track.py's output today."""
    dt = 1.0 / fps
    F, Q, H, R = _kf_matrices(dt, cfg["q"], cfg["r"])
    conf_high = cfg["conf_high"]

    by_fi = {fi: g for fi, g in det.groupby("fi", sort=True)}
    fi_lo, fi_hi = int(det.fi.min()), int(det.fi.max())

    tracks: dict[int, Track] = {}
    next_id = 1
    out_rows: list[dict] = []

    for fi in range(fi_lo, fi_hi + 1):
        for tr in tracks.values():
            tr.x = F @ tr.x
            tr.P = F @ tr.P @ F.T + Q

        g = by_fi.get(fi)
        if g is None or g.empty:
            g_high = g_low = det.iloc[0:0]
        else:
            g_high = g[g.conf >= conf_high]
            g_low = g[g.conf < conf_high]

        cand = list(tracks.keys())
        matches1, unmatched, unused_high = _associate(tracks, cand, g_high, H, R, cfg)
        for tid, c in matches1:
            _apply_match(tracks[tid], g_high.iloc[c], fi, H, R, cfg, out_rows)

        matches2, unmatched, unused_low = _associate(tracks, unmatched, g_low, H, R, cfg)
        for tid, c in matches2:
            _apply_match(tracks[tid], g_low.iloc[c], fi, H, R, cfg, out_rows)

        dead = []
        for tid in unmatched:
            tr = tracks[tid]
            tr.time_since_update += 1
            if not tr.confirmed or tr.time_since_update > cfg["max_lost_frames"]:
                dead.append(tid)
        for tid in dead:
            del tracks[tid]

        # births: unmatched HIGH-confidence detections only (ByteTrack-style --
        # low-confidence boxes are for re-acquiring existing tracks, not new ones)
        for _, det_row in unused_high.iterrows():
            tr = Track(next_id, det_row.x_m, det_row.y_m, (det_row.w_m, det_row.h_m), cfg["r"])
            tr.hits = 1
            tr.cls_counts[det_row.cls] = 1
            _record_hit(tr, _row_dict(fi, next_id, tr.cls, det_row), cfg["min_hits"], out_rows)
            tracks[next_id] = tr
            next_id += 1

    cols = ["fi", "t", "track_id", "cls", "conf", "x1", "y1", "x2", "y2", "u_px", "v_px"]
    return pd.DataFrame(out_rows, columns=cols)


if __name__ == "__main__":
    import argparse

    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="mtrack")
    ap.add_argument("--from", dest="src", default="out/detraw_intersection.parquet",
                    help="detect.py detraw parquet (needs x_m,y_m,w_m,h_m)")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--max-lost-s", type=float, default=6.0)
    ap.add_argument("--min-hits", type=int, default=3)
    ap.add_argument("--gate-chi2", type=float, default=9.21)
    ap.add_argument("--lambda-cls", type=float, default=1.0)
    ap.add_argument("--lambda-app", type=float, default=0.5)
    ap.add_argument("--conf-high", type=float, default=0.5)
    ap.add_argument("--q", type=float, default=1.5)
    ap.add_argument("--r", type=float, default=0.35)
    a = ap.parse_args()

    cfg = dict(
        max_lost_frames=round(a.max_lost_s * a.fps), min_hits=a.min_hits,
        gate_chi2=a.gate_chi2, lambda_cls=a.lambda_cls, lambda_app=a.lambda_app,
        conf_high=a.conf_high, q=a.q, r=a.r,
    )

    det = pd.read_parquet(a.src)
    n0 = len(det)
    det = det.dropna(subset=["x_m", "y_m", "w_m", "h_m"]).reset_index(drop=True)
    if len(det) < n0:
        print(f"dropped {n0 - len(det)} detections with no ground-plane position")

    out = run(det, a.fps, cfg)
    dest = root / "out" / f"detections_{a.tag}.parquet"
    out.to_parquet(dest, index=False)
    print(f"\n{len(out)} detections, {out.track_id.nunique()} tracks -> {dest}")
    print(out.cls.value_counts().to_string())

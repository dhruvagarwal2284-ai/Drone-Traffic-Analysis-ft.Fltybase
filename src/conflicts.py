"""Interaction and surrogate-safety metrics computed in the shared metric frame.

This is the layer the brief names first: a perspective view of one approach
cannot place two road users in a common coordinate frame, so near-misses,
conflicts and failed merges go unmeasured by everything in the field. Once
trajectories are in metres on a common ground plane, they fall out directly.

Two complementary measures:

PET (post-encroachment time)
    Time between one road user leaving a point in space and another arriving at
    it. The standard junction conflict measure. Only meaningful for *crossing*
    paths, so car-following (which produces small time gaps at the same point
    but is not a conflict) is excluded by a heading-difference test.

TTC (time to collision)
    Time until collision if both parties hold their current velocity. Computed
    per frame for closing pairs, using class-dependent footprint radii.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

# Effective conflict radius, metres -- a LATERAL half-width, not half the
# vehicle length. Using a length-scale radius makes two cars in adjacent lanes
# register as already touching, which in lane-sharing traffic manufactures
# thousands of phantom conflicts.
RADIUS = {"car": 0.95, "two_wheeler": 0.45, "motorcycle": 0.45,
          "bicycle": 0.40, "person": 0.30, "bus": 1.30, "truck": 1.25,
          "autorickshaw": 0.65}

CELL_M = 1.5          # spatial grid for PET
PET_MAX = 5.0         # seconds; above this it is not an interaction
MIN_CROSS_ANGLE = 40.0  # deg; below this it is car-following, not a conflict
TTC_MAX = 8.0
NEIGHBOUR_M = 30.0    # pruning radius for the TTC pass
MOVING_MS = 1.2       # ignore parked/stopped users when scoring conflicts
PET_MIN = 0.4         # below this, almost always two ids on one vehicle
TTC_MIN = 0.30        # below this the pair is already overlapping, not closing
MIN_GAP_M = 1.0       # TTC must predict a future contact, not report a present one


def _radii(cls: pd.Series) -> np.ndarray:
    return cls.map(RADIUS).fillna(1.45).to_numpy()


def pet(traj: pd.DataFrame) -> pd.DataFrame:
    """Minimum PET per crossing track pair."""
    d = traj[(~traj.imputed) & (np.hypot(traj.vx, traj.vy) > MOVING_MS)]
    if d.empty:
        return pd.DataFrame()

    gx = np.floor(d.x_m / CELL_M).astype(int)
    gy = np.floor(d.y_m / CELL_M).astype(int)
    d = d.assign(cell=list(zip(gx, gy)))

    # first arrival of each track in each cell, with its heading there
    first = (d.sort_values("t")
               .groupby(["cell", "track_id"], sort=False)
               .agg(t=("t", "first"), hd=("heading", "first"),
                    x=("x_m", "first"), y=("y_m", "first"),
                    cls=("cls", "first"), spd=("speed_kph", "first"))
               .reset_index())

    out = []
    for cell, g in first.groupby("cell", sort=False):
        if len(g) < 2:
            continue
        g = g.sort_values("t")
        t = g["t"].to_numpy()
        hd = g["hd"].to_numpy()
        n = len(g)
        # only compare each arrival with the next few in time
        for i in range(n - 1):
            for j in range(i + 1, min(i + 6, n)):
                dt = t[j] - t[i]
                if dt > PET_MAX:
                    break
                if dt < PET_MIN:
                    continue      # residual duplicate ids, not an interaction
                ang = abs((hd[j] - hd[i] + 180.0) % 360.0 - 180.0)
                if ang < MIN_CROSS_ANGLE:
                    continue          # car-following, not a conflict
                a, b = g.iloc[i], g.iloc[j]
                if a.cls == "person" and b.cls == "person":
                    continue
                out.append({
                    "id_a": a.track_id, "id_b": b.track_id,
                    "cls_a": a.cls, "cls_b": b.cls,
                    "pet": dt, "cross_angle": ang,
                    "x_m": a.x, "y_m": a.y, "t": b.t,
                    "spd_a": a.spd, "spd_b": b.spd,
                })

    if not out:
        return pd.DataFrame()
    df = pd.DataFrame(out)
    key = df.apply(lambda r: tuple(sorted((r.id_a, r.id_b))), axis=1)
    return (df.assign(pair=key)
              .sort_values("pet")
              .drop_duplicates("pair")
              .reset_index(drop=True))


def ttc(traj: pd.DataFrame) -> pd.DataFrame:
    """Minimum TTC per pair, scanned frame by frame."""
    d = traj[(~traj.imputed) & (np.hypot(traj.vx, traj.vy) > MOVING_MS)]
    best: dict[tuple, dict] = {}

    for fi, g in d.groupby("fi", sort=False):
        n = len(g)
        if n < 2:
            continue
        p = g[["x_m", "y_m"]].to_numpy()
        v = g[["vx", "vy"]].to_numpy()
        r = _radii(g["cls"])
        ids = g["track_id"].to_numpy()
        cl = g["cls"].to_numpy()
        sp = g["speed_kph"].to_numpy()

        i, j = np.triu_indices(n, k=1)
        dp = p[j] - p[i]
        dist = np.hypot(dp[:, 0], dp[:, 1])
        near = dist < NEIGHBOUR_M
        if not near.any():
            continue
        i, j, dp, dist = i[near], j[near], dp[near], dist[near]

        dv = v[j] - v[i]
        R = r[i] + r[j]
        a = np.einsum("ij,ij->i", dv, dv)
        b = 2.0 * np.einsum("ij,ij->i", dp, dv)
        c = np.einsum("ij,ij->i", dp, dp) - R ** 2

        disc = b ** 2 - 4 * a * c
        gap = dist - R
        ok = (a > 1e-6) & (b < 0) & (disc > 0) & (gap > MIN_GAP_M)
        if not ok.any():
            continue
        t_hit = np.full(len(a), np.inf)
        t_hit[ok] = (-b[ok] - np.sqrt(disc[ok])) / (2 * a[ok])
        ok &= (t_hit > TTC_MIN) & (t_hit < TTC_MAX)

        for k in np.flatnonzero(ok):
            if cl[i[k]] == "person" and cl[j[k]] == "person":
                continue
            key = (int(min(ids[i[k]], ids[j[k]])), int(max(ids[i[k]], ids[j[k]])))
            if key not in best or t_hit[k] < best[key]["ttc"]:
                best[key] = {
                    "id_a": key[0], "id_b": key[1],
                    "cls_a": cl[i[k]], "cls_b": cl[j[k]],
                    "ttc": float(t_hit[k]), "fi": int(fi),
                    "t": float(g["t"].iloc[0]),
                    "gap_m": float(dist[k] - R[k]),
                    "x_m": float((p[i[k], 0] + p[j[k], 0]) / 2),
                    "y_m": float((p[i[k], 1] + p[j[k], 1]) / 2),
                    "spd_a": float(sp[i[k]]), "spd_b": float(sp[j[k]]),
                }

    return (pd.DataFrame(best.values()).sort_values("ttc").reset_index(drop=True)
            if best else pd.DataFrame())


def severity(pet_df: pd.DataFrame, ttc_df: pd.DataFrame) -> pd.DataFrame:
    """Merge both measures into one ranked conflict list.

    Severity rises as either measure falls. PET thresholds follow common
    practice: <1.0 s critical, <1.5 s serious, <3.0 s a conflict worth review.
    """
    frames = []
    if not pet_df.empty:
        p = pet_df.rename(columns={"pet": "value"})[
            ["id_a", "id_b", "cls_a", "cls_b", "value", "x_m", "y_m", "t",
             "spd_a", "spd_b"]].copy()
        p["measure"] = "PET"
        frames.append(p)
    if not ttc_df.empty:
        q = ttc_df.rename(columns={"ttc": "value"})[
            ["id_a", "id_b", "cls_a", "cls_b", "value", "x_m", "y_m", "t",
             "spd_a", "spd_b"]].copy()
        q["measure"] = "TTC"
        frames.append(q)
    if not frames:
        return pd.DataFrame()

    df = pd.concat(frames, ignore_index=True)
    df["severity"] = np.clip(1.0 - df["value"] / 3.0, 0.0, 1.0)
    df["grade"] = pd.cut(df["value"], [-0.01, 1.0, 1.5, 3.0, 99],
                         labels=["critical", "serious", "conflict", "minor"])
    return df.sort_values("value").reset_index(drop=True)


def heatmap(conf: pd.DataFrame, bins: int = 60, extent: float = 70.0):
    """2-D histogram of conflict locations on the ground plane."""
    sel = conf[conf["value"] < 3.0]
    rng = [[-extent, extent], [-extent, extent]]
    H, xe, ye = np.histogram2d(sel.x_m, sel.y_m, bins=bins, range=rng)
    return H, xe, ye

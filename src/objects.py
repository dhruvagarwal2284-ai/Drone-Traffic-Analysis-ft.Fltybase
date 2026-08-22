"""One row per road user -- the object-level record.

Joins four things that were previously only available separately: what the
object *is* (class, body type, size, colour, measured dimensions), how it
*moved* (velocity and acceleration in real units), where it *went* (origin leg,
destination leg, turn), and what it *interacted with* (conflicts it appears in).

Kinematics are in SI throughout and come from the RTS-smoothed trajectory, not
from raw finite differences -- differencing noisy positions produces
accelerations that are pure noise, which is exactly the quantity most of these
columns depend on.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

STOP_MS = 0.85          # m/s, below this a road user counts as stopped
BRAKE_MS2 = -2.5        # m/s^2, a deliberate braking event
ACCEL_MS2 = 2.0
PLAUSIBLE_MS2 = 4.0     # beyond this, no road vehicle here is really accelerating
EDGE_TRIM = 2           # samples dropped at each end of a track before peaks


def kinematics(traj: pd.DataFrame, fps: float = 10.0) -> pd.DataFrame:
    """Per-object velocity and acceleration summary, in m/s and m/s^2."""
    dt = 1.0 / fps
    d = traj.copy()
    d["v"] = np.hypot(d.vx, d.vy)

    def agg(g: pd.DataFrame) -> pd.Series:
        v, a_all = g.v.to_numpy(), g.accel.to_numpy()
        # The smoother has no data beyond a track's ends, so the first and last
        # samples carry an edge transient that shows up as a spurious peak
        # acceleration. Trim them before taking extremes; keep them for means.
        a = a_all[EDGE_TRIM:-EDGE_TRIM] if len(a_all) > 2 * EDGE_TRIM + 1 else a_all
        step = np.hypot(np.diff(g.x_m.to_numpy()), np.diff(g.y_m.to_numpy()))
        jerk = np.diff(a) / dt if len(a) > 1 else np.array([0.0])
        moving = v > STOP_MS
        return pd.Series({
            "duration_s": len(g) * dt,
            "distance_m": float(step.sum()),
            "v_mean_ms": float(v[moving].mean()) if moving.any() else 0.0,
            "v_max_ms": float(v.max()),
            "v85_ms": float(np.quantile(v[moving], 0.85)) if moving.any() else 0.0,
            "v_mean_kph": float(v[moving].mean() * 3.6) if moving.any() else 0.0,
            "v_max_kph": float(v.max() * 3.6),
            "a_max_ms2": float(a.max()),
            "a_min_ms2": float(a.min()),
            "jerk_p95_ms3": float(np.quantile(np.abs(jerk), 0.95)),
            "a_p95_ms2": float(np.quantile(a, 0.95)),
            "a_p05_ms2": float(np.quantile(a, 0.05)),
            "kinematics_plausible": bool(max(abs(a.max()), abs(a.min())) < PLAUSIBLE_MS2),
            "stopped_s": float((~moving).sum() * dt),
            "n_brake_events": int(_events(a, BRAKE_MS2, less=True)),
            "n_accel_events": int(_events(a, ACCEL_MS2, less=False)),
            "imputed_frac": float(g.imputed.mean()),
        })

    return d.groupby("track_id").apply(agg, include_groups=False).reset_index()


def _events(a: np.ndarray, thr: float, less: bool) -> int:
    """Count distinct excursions past a threshold, not samples past it."""
    m = a < thr if less else a > thr
    if not m.any():
        return 0
    return int(np.sum(m & ~np.r_[False, m[:-1]]))


def build(traj: pd.DataFrame, moves: pd.DataFrame, attrs: pd.DataFrame | None,
          conflicts: pd.DataFrame | None, fps: float = 10.0) -> pd.DataFrame:
    k = kinematics(traj, fps)

    mv = moves[["track_id", "cls", "origin", "dest", "turn", "movement",
                "traversed"]].copy()
    out = k.merge(mv, on="track_id", how="left")

    if attrs is not None and not attrs.empty:
        a = attrs[["track_id", "colour", "hex", "L_m", "W_m",
                   "body_type", "size_class", "n_samples"]]
        out = out.merge(a, on="track_id", how="left")

    if conflicts is not None and not conflicts.empty:
        c = conflicts
        pair = pd.concat([
            c[["id_a", "measure", "value"]].rename(columns={"id_a": "track_id"}),
            c[["id_b", "measure", "value"]].rename(columns={"id_b": "track_id"}),
        ])
        g = pair.groupby("track_id").agg(
            n_conflicts=("value", "size"),
            worst_value_s=("value", "min")).reset_index()
        out = out.merge(g, on="track_id", how="left")
        out["n_conflicts"] = out.n_conflicts.fillna(0).astype(int)

    # a compact, human-readable identity for each object
    out["descriptor"] = [
        " ".join(x for x in [
            (col if isinstance(col, str) and col not in ("other", None) else None),
            (bt if isinstance(bt, str) and bt != "unknown" else cls),
        ] if x)
        for col, bt, cls in zip(out.get("colour", pd.Series([None] * len(out))),
                                out.get("body_type", pd.Series([None] * len(out))),
                                out.cls)
    ]
    return out


def validate(objs: pd.DataFrame) -> dict:
    """Physical plausibility of the kinematics, reported not hidden."""
    v = objs.v_max_kph.dropna()
    return {
        "objects": int(len(objs)),
        "v_max_kph_p50": round(float(v.median()), 1),
        "v_max_kph_p99": round(float(v.quantile(0.99)), 1),
        "v_over_120kph": int((v > 120).sum()),
        "accel_within_4ms2": round(float(
            ((objs.a_max_ms2 < 4) & (objs.a_min_ms2 > -4)).mean()), 4),
        "kinematics_plausible_frac": round(float(objs.kinematics_plausible.mean()), 4),
        "implausible_objects": int((~objs.kinematics_plausible).sum()),
        "median_distance_m": round(float(objs.distance_m.median()), 1),
        "with_dimensions": int(objs.L_m.notna().sum()) if "L_m" in objs else 0,
        "with_colour": int((objs.colour != "other").sum()) if "colour" in objs else 0,
    }

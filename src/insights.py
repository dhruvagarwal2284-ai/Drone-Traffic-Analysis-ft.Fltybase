"""Derived traffic insight, all computed from the metric trajectory table.

Nothing here needs the junction to be annotated. The approach legs are
*discovered* from where trajectories enter and leave, so the same code runs at
a site nobody has surveyed -- which is the point of using a drone at all.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

STOP_KPH = 3.0
HARD_BRAKE = -3.0        # m/s^2


# --------------------------------------------------------------------------
# scene geometry, discovered from motion
# --------------------------------------------------------------------------
def junction_centre(traj: pd.DataFrame) -> tuple[float, float]:
    return float(traj.x_m.median()), float(traj.y_m.median())


def _bearing(dx, dy):
    return np.degrees(np.arctan2(dx, dy)) % 360.0


def find_legs(traj: pd.DataFrame, n_bins: int = 72, min_frac: float = 0.018):
    """Discover approach legs from the bearings at which tracks enter/leave.

    Roads show up as peaks in the circular histogram of entry/exit bearings;
    everything between them is empty. No map, no manual zones.
    """
    cx, cy = junction_centre(traj)
    ends = traj.groupby("track_id").agg(
        x0=("x_m", "first"), y0=("y_m", "first"),
        x1=("x_m", "last"), y1=("y_m", "last"))

    # Only tracks that actually traversed the scene say anything about where the
    # roads are. Short fragments start and end mid-carriageway and would smear
    # the bearing histogram into noise.
    travelled = np.hypot(ends.x1 - ends.x0, ends.y1 - ends.y0) > 25.0
    if travelled.sum() >= 30:
        ends = ends[travelled]

    b = np.concatenate([_bearing(ends.x0 - cx, ends.y0 - cy),
                        _bearing(ends.x1 - cx, ends.y1 - cy)])
    hist, _ = np.histogram(b, bins=n_bins, range=(0, 360))

    k = np.array([1.0, 2.0, 3.0, 2.0, 1.0])
    k /= k.sum()
    sm = np.convolve(np.r_[hist[-2:], hist, hist[:2]], k, mode="valid")

    peaks = []
    for i in range(n_bins):
        if (sm[i] >= sm[(i - 1) % n_bins] and sm[i] >= sm[(i + 1) % n_bins]
                and sm[i] > sm.sum() * min_frac):
            peaks.append(i)

    step = 360.0 / n_bins
    merged = []
    for p in sorted(peaks, key=lambda q: -sm[q]):
        if any(min(abs(p - m), n_bins - abs(p - m)) * step < 28 for m in merged):
            continue
        merged.append(p)
    merged.sort()

    centres = [(p + 0.5) * step for p in merged]
    return {"centre": (cx, cy), "bearings": centres, "names": _name_legs(centres)}


def _name_legs(bearings):
    compass = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    out, seen = [], {}
    for b in bearings:
        nm = compass[int(((b + 22.5) % 360) // 45)]
        seen[nm] = seen.get(nm, 0) + 1
        out.append(nm if seen[nm] == 1 else f"{nm}{seen[nm]}")
    return out


def assign_movements(traj: pd.DataFrame, legs: dict) -> pd.DataFrame:
    """Per track: origin leg, destination leg, and turn type."""
    cx, cy = legs["centre"]
    bear = np.array(legs["bearings"])
    names = legs["names"]

    ends = traj.groupby("track_id").agg(
        x0=("x_m", "first"), y0=("y_m", "first"),
        x1=("x_m", "last"), y1=("y_m", "last"),
        cls=("cls", lambda s: s.mode().iloc[0]),
        t0=("t", "first"), t1=("t", "last"),
        vmax=("speed_kph", "max"), vmean=("speed_kph", "mean"),
        pcu=("pcu", "first"))

    def nearest(bs):
        d = np.abs((bs[:, None] - bear[None, :] + 180) % 360 - 180)
        return d.argmin(axis=1)

    oi = nearest(_bearing(ends.x0 - cx, ends.y0 - cy).to_numpy())
    di = nearest(_bearing(ends.x1 - cx, ends.y1 - cy).to_numpy())
    ends["origin"] = [names[i] for i in oi]
    ends["dest"] = [names[i] for i in di]

    # A leg's bearing points outward from the junction centre, so a vehicle
    # arriving from leg `oi` is travelling at bear[oi]+180. The turn is the
    # change in *travel* direction, not the angle between the two legs --
    # otherwise every through movement reads as a u-turn.
    turn = (bear[di] - bear[oi] - 180.0 + 180.0) % 360.0 - 180.0
    ends["turn"] = np.select(
        [np.abs(turn) < 35, np.abs(turn) > 150, turn > 0],
        ["through", "u-turn", "right"], default="left")
    ends["dwell_s"] = ends.t1 - ends.t0
    ends["movement"] = ends.origin + "->" + ends.dest
    # A movement is only meaningful if the track actually reached the junction
    # and left again. A vehicle tracked along one arm without getting there has
    # near-identical entry and exit bearings and would otherwise be counted as a
    # u-turn -- which is how a junction ends up appearing to be 50% u-turns.
    cen = np.hypot(traj.x_m - cx, traj.y_m - cy)
    dmin = cen.groupby(traj.track_id).min()
    d0 = np.hypot(ends.x0 - cx, ends.y0 - cy)
    d1 = np.hypot(ends.x1 - cx, ends.y1 - cy)
    ends["d_min"] = dmin.reindex(ends.index).to_numpy()
    approached = (ends.d_min < 0.6 * np.minimum(d0, d1)) | (ends.d_min < 15.0)
    ends["traversed"] = (np.hypot(ends.x1 - ends.x0, ends.y1 - ends.y0) > 25.0) & approached
    return ends.reset_index()


# --------------------------------------------------------------------------
# flow, speed, delay
# --------------------------------------------------------------------------
def counts(moves: pd.DataFrame) -> pd.DataFrame:
    c = moves.groupby("cls").agg(n=("track_id", "count"),
                                 pcu=("pcu", "sum"),
                                 mean_kph=("vmean", "mean")).reset_index()
    c["share_pct"] = (100 * c.n / c.n.sum()).round(1)
    c["mean_kph"] = c.mean_kph.round(1)
    return c.sort_values("n", ascending=False)


def od_matrix(moves: pd.DataFrame) -> pd.DataFrame:
    return pd.crosstab(moves.origin, moves.dest)


def flow_series(moves: pd.DataFrame, bin_s: float = 15.0) -> pd.DataFrame:
    b = (moves.t0 // bin_s) * bin_s
    g = moves.groupby(b).agg(vehicles=("track_id", "count"), pcu=("pcu", "sum"))
    g.index.name = "t"
    g["flow_pcu_hr"] = g.pcu * 3600.0 / bin_s
    return g.reset_index()


def speed_stats(traj: pd.DataFrame) -> pd.DataFrame:
    m = traj[traj.speed_kph > STOP_KPH]
    return (m.groupby("cls")["speed_kph"]
              .agg(n="count", mean="mean",
                   p15=lambda s: s.quantile(.15),
                   p50="median",
                   p85=lambda s: s.quantile(.85))
              .round(1).reset_index())


def delay(traj: pd.DataFrame, moves: pd.DataFrame) -> pd.DataFrame:
    """Stopped time per track -- the user-facing cost of the junction."""
    dt = float(np.median(np.diff(np.sort(traj.t.unique()))))
    st = (traj.speed_kph < STOP_KPH).groupby(traj.track_id).sum() * dt
    out = moves.set_index("track_id").join(st.rename("stopped_s"))
    out["stopped_s"] = out.stopped_s.fillna(0.0)
    return out.reset_index()


# --------------------------------------------------------------------------
# open-vocabulary anomalies
# --------------------------------------------------------------------------
def anomalies(traj: pd.DataFrame, cell: float = 6.0) -> pd.DataFrame:
    """Flag statistical outliers instead of matching a fixed event list.

    We learn what normal looks like *per location* -- the prevailing heading in
    each cell of the scene -- and score departures from it. Nothing here
    enumerates event types in advance, so a behaviour nobody thought to
    configure still surfaces.
    """
    d = traj[~traj.imputed].copy()
    d["cx"] = np.floor(d.x_m / cell).astype("Int64")
    d["cy"] = np.floor(d.y_m / cell).astype("Int64")

    moving = d[d.speed_kph > STOP_KPH]
    rad = np.radians(moving.heading)
    field = (moving.assign(sx=np.sin(rad), sy=np.cos(rad))
                   .groupby(["cx", "cy"])
                   .agg(sx=("sx", "mean"), sy=("sy", "mean"), n=("sx", "size")))
    field = field[field.n >= 12]
    field["mode_hd"] = np.degrees(np.arctan2(field.sx, field.sy)) % 360

    d = d.join(field[["mode_hd"]], on=["cx", "cy"])
    dev = np.abs((d.heading - d.mode_hd + 180) % 360 - 180)
    d["contraflow"] = (dev > 120) & (d.speed_kph > STOP_KPH) & d.mode_hd.notna()

    dt = float(np.median(np.diff(np.sort(traj.t.unique()))))
    ev = []
    for tid, g in d.groupby("track_id"):
        if g.contraflow.mean() > 0.5 and len(g) > 8:
            ev.append({"track_id": tid, "kind": "contraflow", "cls": g.cls.iloc[0],
                       "t": float(g.t.iloc[0]),
                       "x_m": float(g.x_m.mean()), "y_m": float(g.y_m.mean()),
                       "detail": f"{g.contraflow.mean():.0%} of path against prevailing flow"})
        stopped = float((g.speed_kph < 1.0).sum()) * dt
        if stopped > 8.0 and g.speed_kph.max() > 8.0:
            ev.append({"track_id": tid, "kind": "stopped_in_carriageway",
                       "cls": g.cls.iloc[0], "t": float(g.t.iloc[0]),
                       "x_m": float(g.x_m.mean()), "y_m": float(g.y_m.mean()),
                       "detail": f"stationary {stopped:.0f}s mid-scene"})
        if g.accel.min() < HARD_BRAKE:
            i = g.accel.idxmin()
            ev.append({"track_id": tid, "kind": "hard_braking", "cls": g.cls.iloc[0],
                       "t": float(g.t.loc[i]),
                       "x_m": float(g.x_m.loc[i]), "y_m": float(g.y_m.loc[i]),
                       "detail": f"{g.accel.min():.1f} m/s2 deceleration"})
    return pd.DataFrame(ev)


def quality(traj: pd.DataFrame, moves: pd.DataFrame, cal: dict) -> dict:
    """Physical-plausibility checks, reported rather than hidden.

    There are no annotations, so correctness is argued from physics: speeds and
    accelerations must sit in humanly possible bands, and the calibration must
    reproduce a size it was not fitted to.
    """
    return {
        "tracks": int(traj.track_id.nunique()),
        "rows": int(len(traj)),
        "imputed_frac": round(float(traj.imputed.mean()), 4),
        "speed_p50_kph": round(float(traj.speed_kph.median()), 1),
        "speed_p99_kph": round(float(traj.speed_kph.quantile(.99)), 1),
        "speed_over_100kph_frac": round(float((traj.speed_kph > 100).mean()), 5),
        "accel_within_4_frac": round(float((traj.accel.abs() < 4).mean()), 4),
        "median_track_dwell_s": round(float(moves.dwell_s.median()), 1),
        "calibration": cal,
    }

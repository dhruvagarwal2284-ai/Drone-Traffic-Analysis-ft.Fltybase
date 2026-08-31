"""Aggregate insight: summaries across many road users, time, and space.

Five things live here, in rising order of what they demand from the data:

* **Classified counts by interval and movement** -- the turning count a survey
  crew produces, but resolved in time.
* **Spatial speed field** -- where on the carriageway speed is actually lost,
  rather than a single average for the site.
* **Lane discovery** -- lanes are found from where vehicles put themselves
  laterally, not from painted markings (which are largely absent here anyway),
  so lane discipline becomes a *measured* quantity rather than an assumption.
* **Queue length in metres** -- the back of queue, traced outward from the stop
  line through contiguous stopped vehicles.
* **Fundamental diagram via Edie's definitions** -- generalised flow and
  density over space-time regions. This is the rigorous formulation, and it
  needs complete trajectories: loop detectors approximate it, drones can
  compute it directly.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

STOP_KPH = 3.0
QUEUE_GAP_M = 14.0     # a break longer than this ends the queue
CORRIDOR_HALF_W = 14.0  # metres either side of the arterial axis
LANE_WIDTH_M = 3.5      # for converting a measured width to equivalent lanes


# ---------------------------------------------------------------------------
# 1. classified counts by interval
# ---------------------------------------------------------------------------
def interval_counts(moves: pd.DataFrame, bin_s: float = 20.0) -> pd.DataFrame:
    """Counts per (interval, movement, class) -- a turning count resolved in time."""
    t = moves[moves.traversed].copy()
    if t.empty:
        return pd.DataFrame()
    t["bin"] = (t.t0 // bin_s) * bin_s
    g = (t.groupby(["bin", "movement", "cls"])
           .agg(n=("track_id", "size"), pcu=("pcu", "sum"))
           .reset_index())
    return g


def directional_volumes(moves: pd.DataFrame, bin_s: float = 20.0) -> pd.DataFrame:
    """Approach volumes over time, in vehicles and PCU per hour."""
    t = moves[moves.traversed].copy()
    if t.empty:
        return pd.DataFrame()
    t["bin"] = (t.t0 // bin_s) * bin_s
    g = (t.groupby(["bin", "origin"])
           .agg(veh=("track_id", "size"), pcu=("pcu", "sum")).reset_index())
    g["veh_hr"] = g.veh * 3600.0 / bin_s
    g["pcu_hr"] = g.pcu * 3600.0 / bin_s
    return g


# ---------------------------------------------------------------------------
# 2. spatial speed field
# ---------------------------------------------------------------------------
def speed_field(traj: pd.DataFrame, cell: float = 4.0, min_n: int = 15) -> dict:
    """Mean and 85th-percentile speed per ground cell, plus where the fastest
    traffic sits. A single site-wide average hides the fact that speed is lost
    in specific places."""
    d = traj[(~traj.imputed) & (traj.speed_kph > STOP_KPH)].copy()
    d["cx"] = np.floor(d.x_m / cell).astype(int)
    d["cy"] = np.floor(d.y_m / cell).astype(int)
    g = (d.groupby(["cx", "cy"])["speed_kph"]
           .agg(n="size", mean="mean", v85=lambda s: s.quantile(0.85))
           .reset_index())
    g = g[g.n >= min_n]
    if g.empty:
        return {}
    return {
        "cell_m": cell,
        "cells": [{"x": float(r.cx * cell + cell / 2),
                   "y": float(r.cy * cell + cell / 2),
                   "n": int(r.n), "mean": round(float(r.mean), 1),
                   "v85": round(float(r.v85), 1)} for r in g.itertuples()],
        "v85_max": round(float(g.v85.max()), 1),
        "v85_median_cell": round(float(g.v85.median()), 1),
        "frac_samples_over_40": round(float((d.speed_kph > 40).mean()), 4),
        "frac_samples_over_50": round(float((d.speed_kph > 50).mean()), 4),
    }


# ---------------------------------------------------------------------------
# 3. lane discovery from lateral position
# ---------------------------------------------------------------------------
def _leg_frame(legs: dict, bearing_deg: float):
    """Unit vectors along an approach and across it."""
    b = np.radians(bearing_deg)
    along = np.array([np.sin(b), np.cos(b)])       # outward from centre
    across = np.array([along[1], -along[0]])
    return along, across


def lanes_for_leg(traj: pd.DataFrame, legs: dict, leg_idx: int,
                  d_min: float = 12.0, d_max: float = 55.0,
                  bin_m: float = 0.5) -> dict:
    """Find lanes on one approach from the lateral positions vehicles adopt.

    Painted lanes are largely absent or ignored here, so a lane is defined as a
    mode in the lateral-offset distribution. If the distribution has no clear
    modes, that is itself the finding: traffic is using the carriageway as a
    continuum rather than as lanes.
    """
    cx, cy = legs["centre"]
    bearing = legs["bearings"][leg_idx]
    along, across = _leg_frame(legs, bearing)

    d = traj[(~traj.imputed) & (traj.speed_kph > STOP_KPH)]
    rel = np.column_stack([d.x_m - cx, d.y_m - cy])
    s = rel @ along
    q = rel @ across
    m = (s > d_min) & (s < d_max) & (np.abs(q) < 18.0)
    if m.sum() < 200:
        return {}

    q = q[m]
    cls = d.cls.to_numpy()[m]
    lo, hi = np.percentile(q, [1, 99])
    edges = np.arange(lo, hi + bin_m, bin_m)
    hist, _ = np.histogram(q, bins=edges)
    k = np.array([1.0, 2, 3, 4, 3, 2, 1])
    k /= k.sum()
    sm = np.convolve(hist, k, mode="same")

    peaks = [i for i in range(1, len(sm) - 1)
             if sm[i] >= sm[i - 1] and sm[i] >= sm[i + 1] and sm[i] > sm.max() * 0.35]
    merged = []
    for p in sorted(peaks, key=lambda i: -sm[i]):
        if any(abs(p - m2) * bin_m < 2.2 for m2 in merged):   # lanes >= 2.2 m apart
            continue
        merged.append(p)
    merged.sort()
    centres = [float(edges[i] + bin_m / 2) for i in merged]

    # how lane-like is this really? 1.0 = sharp lanes, 0 = a continuum
    disc = None      # undefined with fewer than two modes
    if len(merged) > 1:
        troughs = [sm[merged[i]:merged[i + 1]].min() for i in range(len(merged) - 1)]
        peaks_v = [sm[i] for i in merged]
        disc = float(1.0 - np.mean(troughs) / max(np.mean(peaks_v), 1e-9))

    out = {"leg": legs["names"][leg_idx], "bearing": round(bearing, 1),
           "lane_offsets_m": [round(c, 2) for c in centres],
           "n_lanes": len(centres),
           "discipline_index": (round(disc, 3) if disc is not None else None),
           "samples": int(m.sum()),
           "lateral_hist": {"edges": [round(float(e), 2) for e in edges[:-1]],
                            "counts": hist.astype(int).tolist()}}

    if centres:
        lane_of = np.argmin(np.abs(q[:, None] - np.array(centres)[None, :]), axis=1)
        rows = []
        for li in range(len(centres)):
            sel = lane_of == li
            if sel.sum() == 0:
                continue
            mix = pd.Series(cls[sel]).value_counts(normalize=True)
            rows.append({"lane": li + 1,
                         "offset_m": round(centres[li], 2),
                         "share_pct": round(100 * sel.mean(), 1),
                         "modal_split": {k2: round(100 * v, 1)
                                         for k2, v in mix.items()}})
        out["lanes"] = rows

    # lateral position by class -- does the two-wheeler filter to the edge?
    out["lateral_by_class"] = {
        c: {"p15": round(float(np.percentile(q[cls == c], 15)), 2),
            "p50": round(float(np.median(q[cls == c])), 2),
            "p85": round(float(np.percentile(q[cls == c], 85)), 2),
            "spread": round(float(np.percentile(q[cls == c], 85)
                                  - np.percentile(q[cls == c], 15)), 2),
            "n": int((cls == c).sum())}
        for c in pd.unique(cls) if (cls == c).sum() >= 60}
    return out


# ---------------------------------------------------------------------------
# 4. queue length in metres
# ---------------------------------------------------------------------------
def queue_length(traj: pd.DataFrame, legs: dict, leg_idx: int,
                 bin_s: float = 2.0) -> pd.DataFrame:
    """Back-of-queue distance over time for one approach.

    Walks outward from the stop line through stopped vehicles, ending the queue
    at the first gap wider than QUEUE_GAP_M. Reported in metres, which is what a
    signal timing decision actually needs -- a vehicle count does not say
    whether the queue blocks the upstream junction.
    """
    cx, cy = legs["centre"]
    along, across = _leg_frame(legs, legs["bearings"][leg_idx])

    d = traj[~traj.imputed].copy()
    rel = np.column_stack([d.x_m - cx, d.y_m - cy])
    d["s"] = rel @ along
    d["q"] = rel @ across
    d = d[(d.s > 0) & (d.s < 75) & (d.q.abs() < CORRIDOR_HALF_W)]
    if d.empty:
        return pd.DataFrame()

    stopped = d[d.speed_kph < STOP_KPH]
    if len(stopped) < 30:
        return pd.DataFrame()
    stop_line = float(np.percentile(stopped.s, 5))

    rows = []
    for tb, g in d.assign(tb=(d.t // bin_s) * bin_s).groupby("tb"):
        s = np.sort(g.loc[g.speed_kph < STOP_KPH, "s"].to_numpy())
        s = s[s >= stop_line - 3]
        if len(s) == 0:
            rows.append({"t": float(tb), "queue_m": 0.0, "n_stopped": 0})
            continue
        back = s[0]
        n = 1
        for a, b in zip(s[:-1], s[1:]):
            if b - a > QUEUE_GAP_M:
                break
            back = b
            n += 1
        rows.append({"t": float(tb), "queue_m": round(float(back - stop_line), 1),
                     "n_stopped": int(n)})
    out = pd.DataFrame(rows)
    out.attrs["stop_line_s"] = stop_line
    return out


# ---------------------------------------------------------------------------
# 5. fundamental diagram, Edie's generalised definitions
# ---------------------------------------------------------------------------
def edie(traj: pd.DataFrame, legs: dict, axis_leg: int = 1,
         s_range=(-55.0, 55.0), ds: float = 22.0, dt: float = 10.0,
         fps: float = 10.0) -> pd.DataFrame:
    """Flow, density and speed over space-time boxes on the main arterial.

    For a space-time region A of size (ds x dt):

        q = (total distance travelled in A) / |A|      [veh/s]
        k = (total time spent in A)         / |A|      [veh/m]
        v = q / k                                      [m/s]

    These are Edie's generalised definitions. They are exact for any region and
    need no assumption of stationarity -- unlike point measurements from a loop,
    which sample flow at one location and infer the rest. A drone gives the
    whole space-time plane, so the fundamental diagram can be measured rather
    than fitted.
    """
    cx, cy = legs["centre"]
    along, across = _leg_frame(legs, legs["bearings"][axis_leg])

    d = traj[~traj.imputed].sort_values(["track_id", "fi"]).copy()
    rel = np.column_stack([d.x_m - cx, d.y_m - cy])
    d["s"] = rel @ along
    d["q_off"] = rel @ across
    d = d[(d.s.between(*s_range)) & (d.q_off.abs() < CORRIDOR_HALF_W)]
    if d.empty:
        return pd.DataFrame()

    # per-sample distance travelled, within-track only
    step = np.hypot(d.x_m.diff(), d.y_m.diff())
    step[d.track_id != d.track_id.shift()] = np.nan
    d["step_m"] = step.fillna(0.0)
    d["sbin"] = np.floor((d.s - s_range[0]) / ds) * ds + s_range[0]
    d["tbin"] = np.floor(d.t / dt) * dt

    # Direction matters: the corridor carries opposing streams, and lumping them
    # together adds one direction's flow to the other's. Split on the sign of
    # velocity projected onto the axis.
    vel = np.column_stack([d.vx, d.vy])
    d["dir"] = np.where(vel @ along >= 0, "outbound", "inbound")
    d = d[np.hypot(d.vx, d.vy) > 0.3]          # a parked vehicle has no direction

    inv = 1.0 / fps
    area = ds * dt
    g = d.groupby(["dir", "sbin", "tbin"]).agg(
        dist_m=("step_m", "sum"), n=("step_m", "size")).reset_index()
    g["time_s"] = g.n * inv
    g["q_vehs"] = g.dist_m / area
    g["k_vehm"] = g.time_s / area
    g["speed_kph"] = np.where(g.k_vehm > 1e-9,
                              (g.q_vehs / g.k_vehm) * 3.6, np.nan)

    # Per-direction carriageway width, measured from where that stream actually
    # sits, converted to an equivalent lane count. Raw corridor totals are not
    # comparable to published capacities; per-lane values are.
    width = (d.groupby("dir")["q_off"]
               .agg(lambda x: np.percentile(x, 95) - np.percentile(x, 5)))
    lanes = (width / LANE_WIDTH_M).clip(lower=1.0)
    g["lanes_equiv"] = g["dir"].map(lanes).astype(float)
    g["width_m"] = g["dir"].map(width).astype(float)

    g["flow_vph"] = g.q_vehs * 3600.0 / g.lanes_equiv
    g["density_vpkm"] = g.k_vehm * 1000.0 / g.lanes_equiv

    # Occupancy: the share of time a point in the region is covered by a
    # vehicle, k * mean vehicle length -- the quantity a loop detector reports.
    mean_len = float(traj.get("extent_m", pd.Series([4.0])).median() or 4.0)
    g["occupancy_pct"] = (g.k_vehm / g.lanes_equiv * mean_len * 100.0).clip(0, 100)
    return g[g.n >= 8].reset_index(drop=True)


def summarise(fd: pd.DataFrame) -> dict:
    if fd.empty:
        return {}
    cap = fd.loc[fd.flow_vph.idxmax()]
    return {
        "boxes": int(len(fd)),
        "units": "per lane (measured width / 3.5 m), per direction",
        "lanes_equiv": {k: round(float(v), 2)
                        for k, v in fd.groupby("dir").lanes_equiv.first().items()},
        "max_flow_vph": round(float(fd.flow_vph.max()), 0),
        "density_at_max_flow_vpkm": round(float(cap.density_vpkm), 1),
        "speed_at_max_flow_kph": round(float(cap.speed_kph), 1),
        "max_density_vpkm": round(float(fd.density_vpkm.max()), 1),
        "min_speed_kph": round(float(fd.speed_kph.min()), 1),
        "median_density_vpkm": round(float(fd.density_vpkm.median()), 1),
        "max_occupancy_pct": round(float(fd.occupancy_pct.max()), 1),
    }

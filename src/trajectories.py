"""Image-space tracks -> smoothed metric trajectories.

This is the core data product. Everything downstream (conflicts, turning
movements, queues, anomalies) is derived from the table this writes, so it is
produced and persisted before any analysis runs.

Pipeline: back-project to the ground plane at each class's mid-height ->
self-calibrate global scale from vehicle footprints -> constant-velocity Kalman
filter with an RTS smoother per track -> derive kinematics.

The smoother is not cosmetic. Accelerations from raw finite differences of
noisy positions are pure noise, and the surrogate safety metrics depend on them.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from geometry import GroundPlane

# physical priors, metres
CLASS_HEIGHT = {  # full height; we back-project at half of this
    "person": 1.70, "bicycle": 1.20, "car": 1.50,
    "motorcycle": 1.55, "two_wheeler": 1.55, "bus": 3.20, "truck": 3.00,
}
CLASS_SIZE = {  # nominal length x width
    "car": (4.0, 1.8), "bus": (10.5, 2.5), "truck": (7.0, 2.5),
    "motorcycle": (1.9, 0.7), "two_wheeler": (1.9, 0.7),
    "bicycle": (1.7, 0.6), "person": (0.5, 0.5),
}
# passenger-car-unit equivalents (IRC 106 style, Indian mixed traffic)
PCU = {"car": 1.0, "motorcycle": 0.5, "bicycle": 0.5, "two_wheeler": 0.5,
       "bus": 3.0, "truck": 3.0, "person": 0.0}

MIN_TRACK_SEC = 1.5


# --------------------------------------------------------------------------
# metric projection
# --------------------------------------------------------------------------
def _box_ground(gp: GroundPlane, df: pd.DataFrame) -> np.ndarray:
    """Ground position of each detection, corrected for object height."""
    u = (df["x1"].to_numpy() + df["x2"].to_numpy()) / 2.0
    v = (df["y1"].to_numpy() + df["y2"].to_numpy()) / 2.0
    zmid = df["cls"].map(CLASS_HEIGHT).fillna(1.5).to_numpy() / 2.0
    return gp.to_ground(np.column_stack([u, v]), plane_z=zmid)


def _footprint_extent(gp: GroundPlane, df: pd.DataFrame) -> np.ndarray:
    """Longer ground-plane side of each detection's box, in metres."""
    zmid = df["cls"].map(CLASS_HEIGHT).fillna(1.5).to_numpy() / 2.0
    x1, y1 = df["x1"].to_numpy(), df["y1"].to_numpy()
    x2, y2 = df["x2"].to_numpy(), df["y2"].to_numpy()
    tl = gp.to_ground(np.column_stack([x1, y1]), plane_z=zmid)
    tr = gp.to_ground(np.column_stack([x2, y1]), plane_z=zmid)
    bl = gp.to_ground(np.column_stack([x1, y2]), plane_z=zmid)
    w = np.linalg.norm(tr - tl, axis=1)
    h = np.linalg.norm(bl - tl, axis=1)
    return np.maximum(w, h)


def _expected_max_extent(length: float, width: float) -> float:
    """Median longer-AABB-side for a L x W rectangle at uniform random heading.

    Vehicles are not axis-aligned in the image, so an axis-aligned box
    overstates size by an amount that depends on heading. Comparing the
    observed median against this expectation makes the scale calibration
    unbiased instead of systematically small.
    """
    th = np.linspace(0, np.pi / 2, 2001)
    a = length * np.abs(np.cos(th)) + width * np.abs(np.sin(th))
    b = length * np.abs(np.sin(th)) + width * np.abs(np.cos(th))
    return float(np.median(np.maximum(a, b)))


def calibrate(gp: GroundPlane, df: pd.DataFrame) -> tuple[float, dict]:
    """Recover the global scale factor from observed car footprints.

    rel_alt is height above takeoff, not above the road, so it carries an
    unknown bias. Only a single global scale is affected, and cars give us a
    ruler. Motorcycle length is then an independent check, since the
    calibration was not fitted to it.
    """
    cars = df[(df["cls"] == "car") & (df["conf"] > 0.5)]
    if len(cars) < 50:
        return 1.0, {"note": "too few cars; scale left at 1.0"}

    obs = np.nanmedian(_footprint_extent(gp, cars))
    exp = _expected_max_extent(*CLASS_SIZE["car"])
    scale = exp / obs

    # rel_alt is measured from the takeoff point. A rooftop launch puts the road
    # tens of metres below that datum, which shows up here as a large factor --
    # legitimate, not an error. Only refuse a frankly impossible one.
    if not (0.4 < scale < 2.5):
        return 1.0, {"note": f"rejected implausible scale {scale:.3f}",
                     "observed_car_m": float(obs), "expected_car_m": exp}

    gp.scale = scale
    motos = df[(df["cls"] == "motorcycle") & (df["conf"] > 0.5)]
    check = {}
    if len(motos) > 50:
        check["motorcycle_len_m"] = float(np.nanmedian(_footprint_extent(gp, motos)))
        check["motorcycle_expected_m"] = _expected_max_extent(*CLASS_SIZE["motorcycle"])
    return scale, {"observed_car_m": float(obs), "expected_car_m": exp,
                   "scale": scale, "effective_height_m": gp.height * scale, **check}


# --------------------------------------------------------------------------
# smoothing
# --------------------------------------------------------------------------
def _rts_smooth(z: np.ndarray, mask: np.ndarray, dt: float,
                q: float = 1.5, r: float = 0.35):
    """Constant-velocity Kalman + Rauch-Tung-Striebel smoother.

    z    : (T,2) observed positions (NaN where missing)
    mask : (T,) True where an observation exists
    q    : process noise as an acceleration std, m/s^2
    r    : measurement noise std, m
    Returns smoothed (T,2) position and (T,2) velocity.
    """
    T = len(z)
    F = np.array([[1, dt, 0, 0], [0, 1, 0, 0],
                  [0, 0, 1, dt], [0, 0, 0, 1]], dtype=float)
    H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]], dtype=float)
    g = np.array([dt ** 2 / 2, dt])
    Qb = np.outer(g, g) * q ** 2
    Q = np.zeros((4, 4))
    Q[:2, :2] = Qb
    Q[2:, 2:] = Qb
    R = np.eye(2) * r ** 2

    i0 = int(np.argmax(mask))
    x = np.array([z[i0, 0], 0.0, z[i0, 1], 0.0])
    P = np.diag([r ** 2, 25.0, r ** 2, 25.0])

    xp, Pp = np.zeros((T, 4)), np.zeros((T, 4, 4))   # predicted (for smoother)
    xf, Pf = np.zeros((T, 4)), np.zeros((T, 4, 4))   # filtered

    for t in range(T):
        if t > 0:
            x = F @ x
            P = F @ P @ F.T + Q
        xp[t], Pp[t] = x, P
        if mask[t]:
            y = z[t] - H @ x
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x = x + K @ y
            P = (np.eye(4) - K @ H) @ P
        xf[t], Pf[t] = x, P

    xs = xf.copy()
    for t in range(T - 2, -1, -1):
        C = Pf[t] @ F.T @ np.linalg.inv(Pp[t + 1])
        xs[t] = xf[t] + C @ (xs[t + 1] - xp[t + 1])

    return xs[:, [0, 2]], xs[:, [1, 3]]


# --------------------------------------------------------------------------
def merge_two_wheelers(df: pd.DataFrame) -> pd.DataFrame:
    """Fold small-footprint vehicles into a single two-wheeler class.

    An earlier version of this tried to *split out* auto-rickshaws by metric
    size, on the theory that a calibrated ground frame recovers a class COCO
    does not have. The data refuted it: tracks labelled auto-rickshaw came out
    at a median 2.13 m footprint against 2.07 m for tracks labelled motorcycle
    -- the same distribution, not two. A real auto-rickshaw is 2.6-2.9 m, so
    that band was relabelling two-wheelers, not finding autos.

    Overhead, COCO also frequently calls a motorcycle a car: nothing in its
    training data looks like a two-wheeler seen from directly above. So the
    honest operation is a merge, not a split. Separating autos from motorcycles
    needs a detector trained on aerial imagery with both classes (VisDrone has
    `motor` and `tricycle`); it is not recoverable from footprint size here.
    """
    med = df.groupby("track_id")["extent_m"].median()
    cls = df.groupby("track_id")["cls"].agg(lambda s: s.mode().iloc[0])
    small = ((cls.isin(["car", "truck"])) & (med < 2.6)) | cls.isin(["motorcycle", "bicycle"])
    ids = set(med.index[small])
    df["cls"] = np.where(df["track_id"].isin(ids), "two_wheeler", df["cls"])
    return df


def deduplicate(traj: pd.DataFrame, radius: float = 2.2,
                min_overlap: int = 6, frac: float = 0.55) -> pd.DataFrame:
    """Remove tracks that are really a second copy of another track.

    The tracker occasionally splits one vehicle into two ids, or fires twice on
    a large vehicle. Left in, those pairs sit on top of each other and produce
    PET values of ~0 s -- they would dominate the conflict ranking with pure
    artefact. Two ids that stay within `radius` for most of the frames they
    share are treated as one road user and the shorter is dropped.
    """
    co, tot = {}, {}
    for _, g in traj.groupby("fi", sort=False):
        if len(g) < 2:
            continue
        p = g[["x_m", "y_m"]].to_numpy()
        ids = g["track_id"].to_numpy()
        i, j = np.triu_indices(len(g), k=1)
        d = np.hypot(p[j, 0] - p[i, 0], p[j, 1] - p[i, 1])
        for k in range(len(i)):
            key = (min(ids[i[k]], ids[j[k]]), max(ids[i[k]], ids[j[k]]))
            tot[key] = tot.get(key, 0) + 1
            if d[k] < radius:
                co[key] = co.get(key, 0) + 1

    length = traj.groupby("track_id").size()
    drop = set()
    for key, n in co.items():
        if tot[key] >= min_overlap and n / tot[key] >= frac:
            a, b = key
            drop.add(b if length.get(a, 0) >= length.get(b, 0) else a)
    return traj[~traj.track_id.isin(drop)].copy(), len(drop)


def build(detections: pd.DataFrame, gp: GroundPlane, fps: float) -> tuple[pd.DataFrame, dict]:
    dt = 1.0 / fps

    scale, cal = calibrate(gp, detections)   # sets gp.scale
    xy = _box_ground(gp, detections)
    detections = detections.assign(
        x_m=xy[:, 0], y_m=xy[:, 1],
        extent_m=_footprint_extent(gp, detections),
    )
    detections = merge_two_wheelers(detections)

    out = []
    for tid, g in detections.groupby("track_id", sort=False):
        g = g.sort_values("fi")
        if len(g) < 2:
            continue
        f0, f1 = int(g["fi"].iloc[0]), int(g["fi"].iloc[-1])
        if (f1 - f0 + 1) * dt < MIN_TRACK_SEC:
            continue

        idx = np.arange(f0, f1 + 1)
        gi = g.set_index("fi").reindex(idx)
        z = gi[["x_m", "y_m"]].to_numpy()
        mask = np.isfinite(z).all(axis=1)
        if mask.sum() < 2:
            continue

        pos, vel = _rts_smooth(np.nan_to_num(z), mask, dt)
        speed = np.linalg.norm(vel, axis=1)
        acc = np.gradient(speed, dt)

        out.append(pd.DataFrame({
            "fi": idx,
            "t": gi["t"].to_numpy(dtype=float),
            "track_id": tid,
            "cls": gi["cls"].ffill().bfill().to_numpy(),
            "conf": gi["conf"].to_numpy(),
            "x_m": pos[:, 0], "y_m": pos[:, 1],
            "vx": vel[:, 0], "vy": vel[:, 1],
            "speed_kph": speed * 3.6,
            "heading": np.degrees(np.arctan2(vel[:, 0], vel[:, 1])) % 360.0,
            "accel": acc,
            "extent_m": gi["extent_m"].to_numpy(),
            "u_px": (gi["x1"].to_numpy() + gi["x2"].to_numpy()) / 2.0,
            "v_px": (gi["y1"].to_numpy() + gi["y2"].to_numpy()) / 2.0,
            "imputed": ~mask,
        }))

    traj = pd.concat(out, ignore_index=True)
    traj["t"] = traj.groupby("track_id")["t"].transform(
        lambda s: s.interpolate().ffill().bfill())
    traj["pcu"] = traj["cls"].map(PCU).fillna(1.0)

    traj, n_dup = deduplicate(traj)
    cal["duplicate_tracks_removed"] = int(n_dup)

    cal["n_tracks"] = int(traj.track_id.nunique())
    cal["n_rows"] = int(len(traj))
    cal["imputed_frac"] = float(traj.imputed.mean())
    return traj, cal


if __name__ == "__main__":
    import argparse
    import json
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import telemetry as tel

    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="intersection")
    ap.add_argument("--video", default="Intersection_Merged-002")
    ap.add_argument("--t0", type=float, default=240.0)
    ap.add_argument("--fps", type=float, default=10.0)
    a = ap.parse_args()

    det = pd.read_parquet(root / "out" / f"detections_{a.tag}.parquet")
    tm = tel.load(root / f"{a.video}.MP4", root / "out" / "cache")
    row = tm.iloc[int(a.t0 * 30000 / 1001)]

    gp = GroundPlane.from_telemetry(3840, 2160, row.focal_len, row.rel_alt,
                                    row.gb_yaw, row.gb_pitch, row.gb_roll,
                                    zoom=row.dzoom)
    gp.set_origin(np.array([[1920.0, 1080.0]]))

    traj, cal = build(det, gp, a.fps)
    dest = root / "out" / f"trajectories_{a.tag}.parquet"
    traj.to_parquet(dest, index=False)
    (root / "out" / f"calibration_{a.tag}.json").write_text(json.dumps(cal, indent=2))

    print(json.dumps(cal, indent=2))
    print(f"\n-> {dest}")
    print(traj.groupby("cls")["track_id"].nunique().to_string())
    print(f"\nspeed p50/p95 kph: {traj.speed_kph.median():.1f} / {traj.speed_kph.quantile(.95):.1f}")

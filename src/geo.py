"""Georeferencing and map-matching: local metres -> the real road network.

Everything upstream works in a local ENU frame whose origin sits under a chosen
pixel. That is enough for speeds and conflicts, but it is not *map-native*: the
results cannot be laid over real road geometry, joined to an asset register, or
handed to anyone who works in lat/lon.

Two steps close that gap.

**Georeferencing.** The ENU frame is already metric and north-aligned (the
rotation is built from a compass gimbal yaw), and the drone's own GPS fix gives
the origin. So the conversion to WGS84 is a local tangent-plane offset -- no
control points, no rubber-sheeting.

**Map-matching.** OpenStreetMap supplies the link geometry. Each trajectory
sample is bound to a link, a direction along that link, and a lane, moment to
moment -- so a trajectory stops being a floating polyline and becomes a
statement about a specific piece of road.

The alignment is checked the hard way: OSM centrelines are projected *into* the
drone frame. Independent geometry landing on the visible carriageway is
external validation that a self-consistent local calibration cannot give.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

R_EARTH = 6378137.0


# ---------------------------------------------------------------------------
# local tangent plane <-> WGS84
# ---------------------------------------------------------------------------
def enu_to_wgs84(e, n, lat0: float, lon0: float):
    """Local ENU metres (origin at lat0/lon0) -> lat, lon."""
    e = np.asarray(e, dtype=float)
    n = np.asarray(n, dtype=float)
    dlat = np.degrees(n / R_EARTH)
    dlon = np.degrees(e / (R_EARTH * math.cos(math.radians(lat0))))
    return lat0 + dlat, lon0 + dlon


def wgs84_to_enu(lat, lon, lat0: float, lon0: float):
    lat = np.asarray(lat, dtype=float)
    lon = np.asarray(lon, dtype=float)
    n = np.radians(lat - lat0) * R_EARTH
    e = np.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    return e, n


class Frame:
    """Ties the analysis frame to the globe.

    `origin` is the ENU offset the GroundPlane subtracts (the chosen pixel), and
    lat0/lon0 is the drone's nadir. Analysis coordinates therefore sit at
    `origin` in the nadir-centred ENU frame.
    """

    def __init__(self, lat0: float, lon0: float, origin: np.ndarray):
        self.lat0, self.lon0 = float(lat0), float(lon0)
        self.origin = np.asarray(origin, dtype=float)

    def to_wgs84(self, x_m, y_m):
        return enu_to_wgs84(np.asarray(x_m) + self.origin[0],
                            np.asarray(y_m) + self.origin[1],
                            self.lat0, self.lon0)

    def from_wgs84(self, lat, lon):
        e, n = wgs84_to_enu(lat, lon, self.lat0, self.lon0)
        return e - self.origin[0], n - self.origin[1]


# ---------------------------------------------------------------------------
# road network
# ---------------------------------------------------------------------------
KEEP_HIGHWAY = {"motorway", "trunk", "primary", "secondary", "tertiary",
                "unclassified", "residential", "living_street",
                "motorway_link", "trunk_link", "primary_link",
                "secondary_link", "tertiary_link", "service"}


def load_network(path: Path, frame: Frame, extent: float = 130.0) -> list[dict]:
    """OSM ways -> links with geometry in analysis coordinates.

    Only carriageways are kept: paths, steps and footways are not links a
    vehicle trajectory can be matched to.
    """
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    links = []
    for el in raw.get("elements", []):
        if el.get("type") != "way" or "geometry" not in el:
            continue
        tags = el.get("tags", {})
        hw = tags.get("highway")
        if hw not in KEEP_HIGHWAY:
            continue
        lat = np.array([g["lat"] for g in el["geometry"]])
        lon = np.array([g["lon"] for g in el["geometry"]])
        x, y = frame.from_wgs84(lat, lon)
        keep = (np.abs(x) < extent) & (np.abs(y) < extent)
        if keep.sum() < 2:
            continue
        links.append({
            "id": el["id"],
            "name": tags.get("name"),
            "highway": hw,
            "oneway": tags.get("oneway") in ("yes", "true", "1"),
            "lanes": tags.get("lanes"),
            "x": x, "y": y,
            "lat": lat, "lon": lon,
        })
    return links


def _link_cloud(links, spacing: float = 1.0):
    """Dense point cloud along all link centrelines, for nearest-link queries."""
    pts = []
    for L in links:
        xy = np.column_stack([L["x"], L["y"]])
        d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
        if d[-1] < spacing:
            continue
        u = np.arange(0.0, d[-1], spacing)
        pts.append(np.column_stack([np.interp(u, d, xy[:, 0]),
                                    np.interp(u, d, xy[:, 1])]))
    return np.vstack(pts) if pts else np.zeros((0, 2))


def refine_alignment(traj: pd.DataFrame, links: list[dict],
                     search_m: float = 40.0, coarse: float = 2.0,
                     fine: float = 0.25) -> tuple[np.ndarray, dict]:
    """Register the OSM network to the observed trajectories.

    The drone's GPS fix is good to a few metres and OSM geometry carries its own
    error, so the two disagree by a translation even when orientation and scale
    are correct. Vehicles, however, are on the carriageway by definition -- the
    trajectories are better control points than the GPS fix is.

    Solves for the translation minimising the median distance from moving
    vehicle samples to the nearest link centreline, coarse-to-fine. Only the
    offset is fitted: rotation comes from the gimbal compass and scale from the
    footprint calibration, both already independently validated.
    """
    from scipy.spatial import cKDTree

    cloud = _link_cloud(links)
    if len(cloud) < 50:
        return np.zeros(2), {"note": "network too sparse to register"}

    veh = traj[(~traj.imputed) & (traj.speed_kph > 5) & (traj.cls != "person")]
    pts = veh[["x_m", "y_m"]].to_numpy()
    if len(pts) > 20000:
        pts = pts[np.random.default_rng(0).choice(len(pts), 20000, replace=False)]

    tree = cKDTree(cloud)
    before = float(np.median(tree.query(pts)[0]))

    def score(dx, dy):
        return float(np.median(tree.query(pts + np.array([dx, dy]))[0]))

    best, bxy = np.inf, np.zeros(2)
    for step, half in ((coarse, search_m), (fine, coarse * 2)):
        gx = np.arange(bxy[0] - half, bxy[0] + half + 1e-9, step)
        gy = np.arange(bxy[1] - half, bxy[1] + half + 1e-9, step)
        for dx in gx:
            for dy in gy:
                v = score(dx, dy)
                if v < best:
                    best, bxy = v, np.array([dx, dy])

    return bxy, {"median_dist_before_m": round(before, 2),
                 "median_dist_after_m": round(best, 2),
                 "offset_east_m": round(float(-bxy[0]), 2),
                 "offset_north_m": round(float(-bxy[1]), 2),
                 "shift_magnitude_m": round(float(np.hypot(*bxy)), 2),
                 "samples": int(len(pts))}


def apply_offset(links: list[dict], shift: np.ndarray) -> list[dict]:
    """Move link geometry into the analysis frame by the fitted translation."""
    out = []
    for L in links:
        M = dict(L)
        M["x"] = L["x"] - shift[0]
        M["y"] = L["y"] - shift[1]
        out.append(M)
    return out


def _seg_project(px, py, ax, ay, bx, by):
    """Perpendicular distance from P to segment AB, plus the along-fraction."""
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = np.where(L2 > 1e-9, ((px - ax) * vx + (py - ay) * vy) / np.maximum(L2, 1e-9), 0.0)
    t = np.clip(t, 0.0, 1.0)
    cx, cy = ax + t * vx, ay + t * vy
    return np.hypot(px - cx, py - cy), t, cx, cy


def match(traj: pd.DataFrame, links: list[dict], max_dist: float = 18.0,
          sample_every: int = 1, heading_weight_m: float = 6.0) -> pd.DataFrame:
    """Bind every trajectory sample to a link, a direction and a lateral offset.

    Matching is by perpendicular distance plus a trajectory-heading penalty,
    gated by `max_dist`. The heading term prevents a vehicle on an arterial
    being snapped to a nearby crossing street simply because its centreline is
    a little closer. Direction is the sign of the vehicle's heading against the
    link's own digitised direction, so "which way along this road" is recovered
    without needing the one-way tag to be correct.

    Signed lateral offset is retained: it is what a lane assignment is built
    from, and it distinguishes the two carriageways of a divided road.
    """
    if not links:
        return pd.DataFrame()

    # `sample_every=1` is deliberate: the map-native product binds the
    # trajectory at the analysis cadence (10 fps here), not only at a sparse
    # reporting cadence. Callers can still opt down for exploratory work.
    d = traj.iloc[::sample_every].copy()
    px = d.x_m.to_numpy()
    py = d.y_m.to_numpy()
    vx = d.vx.to_numpy()
    vy = d.vy.to_numpy()

    best = np.full(len(d), np.inf)       # actual perpendicular distance
    best_score = np.full(len(d), np.inf) # distance + heading compatibility
    best_i = np.full(len(d), -1, dtype=int)
    best_off = np.zeros(len(d))
    best_dir = np.zeros(len(d))
    best_s = np.zeros(len(d))
    best_x = np.zeros(len(d))
    best_y = np.zeros(len(d))
    best_ux = np.zeros(len(d))
    best_uy = np.zeros(len(d))
    best_len = np.zeros(len(d))
    speed = np.hypot(vx, vy)

    for li, L in enumerate(links):
        xs, ys = L["x"], L["y"]
        seglen = np.hypot(np.diff(xs), np.diff(ys))
        cum = np.concatenate([[0.0], np.cumsum(seglen)])
        for k in range(len(xs) - 1):
            dist, t, cx, cy = _seg_project(px, py, xs[k], ys[k], xs[k + 1], ys[k + 1])
            ux, uy = xs[k + 1] - xs[k], ys[k + 1] - ys[k]
            nrm = math.hypot(ux, uy) or 1e-9
            ux, uy = ux / nrm, uy / nrm
            # signed offset: positive to the left of the link direction
            off = -(px - cx) * uy + (py - cy) * ux
            along = vx * ux + vy * uy
            # A road can be digitised in either direction, hence abs(along).
            # Stopped vehicles have no reliable heading and fall back to
            # distance-only selection.
            align = np.abs(along) / np.maximum(speed, 1e-6)
            angle_penalty = np.where(speed > 0.5, 1.0 - np.clip(align, 0, 1), 0.0)
            score = dist + heading_weight_m * angle_penalty
            upd = score < best_score
            best = np.where(upd, dist, best)
            best_score = np.where(upd, score, best_score)
            best_i = np.where(upd, li, best_i)
            best_off = np.where(upd, off, best_off)
            best_dir = np.where(upd, np.sign(along), best_dir)
            best_s = np.where(upd, cum[k] + t * seglen[k], best_s)
            best_x = np.where(upd, cx, best_x)
            best_y = np.where(upd, cy, best_y)
            best_ux = np.where(upd, ux, best_ux)
            best_uy = np.where(upd, uy, best_uy)
            best_len = np.where(upd, cum[-1], best_len)

    ok = best <= max_dist
    out = d.assign(
        link_idx=np.where(ok, best_i, -1),
        link_dist_m=np.where(ok, best, np.nan),
        link_offset_m=np.where(ok, best_off, np.nan),
        link_dir=np.where(ok, best_dir, 0.0),
        link_s_m=np.where(ok, best_s, np.nan),
        # Projection point and road tangent are retained so an overlay can
        # draw a map-matched lane band on the actual carriageway geometry.
        link_x_m=np.where(ok, best_x, np.nan),
        link_y_m=np.where(ok, best_y, np.nan),
        link_ux=np.where(ok, best_ux, np.nan),
        link_uy=np.where(ok, best_uy, np.nan),
        link_length_m=np.where(ok, best_len, np.nan),
    )
    out["link_id"] = [links[i]["id"] if i >= 0 else None for i in out.link_idx]
    out["link_name"] = [links[i]["name"] if i >= 0 else None for i in out.link_idx]
    out["link_class"] = [links[i]["highway"] if i >= 0 else None for i in out.link_idx]
    return out


def assign_lanes(matched: pd.DataFrame, lane_width: float = 3.3) -> pd.DataFrame:
    """Lane index from the signed offset, per link and direction.

    Lane 1 is the lane nearest the link centreline on that side; the sign of the
    offset already separates the two carriageways of a divided road.
    """
    m = matched[matched.link_idx >= 0].copy()
    if m.empty:
        return matched
    side = np.sign(m.link_offset_m).replace(0, 1)
    m["carriageway"] = np.where(side > 0, "left", "right")
    m["lane"] = np.floor(np.abs(m.link_offset_m) / lane_width).astype(int) + 1
    m["lane"] = m["lane"].clip(1, 6)
    return matched.join(m[["carriageway", "lane"]])


# ---------------------------------------------------------------------------
# GeoJSON export -- the map-native product
# ---------------------------------------------------------------------------
def _fc(features):
    return {"type": "FeatureCollection", "features": features}


def trajectories_geojson(traj: pd.DataFrame, frame: Frame, objs: pd.DataFrame | None,
                         max_tracks: int = 400, step: int = 3) -> dict:
    attrs = {}
    if objs is not None and not objs.empty:
        for r in objs.itertuples():
            attrs[r.track_id] = {
                "cls": getattr(r, "cls", None),
                "body_type": getattr(r, "body_type", None),
                "colour": getattr(r, "colour", None),
                "length_m": (None if not np.isfinite(getattr(r, "L_m", np.nan))
                             else round(float(r.L_m), 2)),
                "v_mean_kph": round(float(getattr(r, "v_mean_kph", 0)), 1),
                "movement": getattr(r, "movement", None),
            }
    feats = []
    ids = traj.track_id.drop_duplicates()
    if len(ids) > max_tracks:
        ids = ids.sample(max_tracks, random_state=0)
    for tid, g in traj[traj.track_id.isin(ids)].groupby("track_id"):
        g = g.iloc[::step]
        if len(g) < 3:
            continue
        lat, lon = frame.to_wgs84(g.x_m.to_numpy(), g.y_m.to_numpy())
        coords = [[round(float(a), 7), round(float(b), 7)] for a, b in zip(lon, lat)]
        feats.append({"type": "Feature",
                      "geometry": {"type": "LineString", "coordinates": coords},
                      "properties": {"track_id": int(tid), **attrs.get(tid, {})}})
    return _fc(feats)


def network_geojson(links: list[dict]) -> dict:
    return _fc([{
        "type": "Feature",
        "geometry": {"type": "LineString",
                     "coordinates": [[round(float(o), 7), round(float(a), 7)]
                                     for a, o in zip(L["lat"], L["lon"])]},
        "properties": {"link_id": L["id"], "name": L["name"],
                       "highway": L["highway"], "oneway": L["oneway"]},
    } for L in links])


def queues_geojson(queues: dict, legs: dict, frame: Frame) -> dict:
    """Queue extents drawn along the carriageway, not as a point marker."""
    feats = []
    for i, nm in enumerate(legs["names"]):
        q = queues.get(nm)
        if not q:
            continue
        b = math.radians(legs["bearings"][i])
        along = np.array([math.sin(b), math.cos(b)])
        for stat, val in (("max_m", q["max_m"]), ("p85_m", q["p85_m"])):
            pts = np.array([along * s for s in np.linspace(0.0, float(val), 12)])
            lat, lon = frame.to_wgs84(pts[:, 0], pts[:, 1])
            feats.append({"type": "Feature",
                          "geometry": {"type": "LineString",
                                       "coordinates": [[round(float(o), 7), round(float(a), 7)]
                                                       for a, o in zip(lat, lon)]},
                          "properties": {"approach": nm, "stat": stat,
                                         "length_m": round(float(val), 1)}})
    return _fc(feats)


def desire_lines_geojson(moves: pd.DataFrame, traj: pd.DataFrame,
                         frame: Frame, legs: dict) -> dict:
    """One line per origin-destination movement, weighted by volume.

    Drawn through the *median observed path* for that movement rather than as a
    straight chord, so the line follows the road layout instead of cutting
    across it.
    """
    feats = []
    trav = moves[moves.traversed]
    for mv, g in trav.groupby("movement"):
        ids = set(g.track_id)
        sub = traj[traj.track_id.isin(ids)]
        if sub.empty or len(g) < 2:
            continue
        # resample every path to a common arc-length grid, then take the median
        paths = []
        for _, p in sub.groupby("track_id"):
            xy = p[["x_m", "y_m"]].to_numpy()
            if len(xy) < 4:
                continue
            d = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
            if d[-1] < 15:
                continue
            u = np.linspace(0, d[-1], 24)
            paths.append(np.column_stack([np.interp(u, d, xy[:, 0]),
                                          np.interp(u, d, xy[:, 1])]))
        if len(paths) < 2:
            continue
        med = np.median(np.stack(paths), axis=0)
        lat, lon = frame.to_wgs84(med[:, 0], med[:, 1])
        feats.append({"type": "Feature",
                      "geometry": {"type": "LineString",
                                   "coordinates": [[round(float(o), 7), round(float(a), 7)]
                                                   for a, o in zip(lat, lon)]},
                      "properties": {"movement": mv, "volume": int(len(g)),
                                     "turn": g.turn.mode().iloc[0],
                                     "pcu": round(float(g.pcu.sum()), 1)}})
    return _fc(feats)

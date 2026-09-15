"""Annotated overlay video: per-object records and aggregate results, in frame.

Three layers, matching the three layers of the analysis:

* **Trails** -- the evidence that a complete path exists in a common metric
  frame. A box with a label is what every fielded system already draws; the
  trail is what the rest of the analysis is built on.
* **Object records** -- body type, measured length, speed, and a chip of the
  measured paint colour, so classification and colour extraction can be checked
  against the vehicle they were taken from.
* **Aggregate results projected back onto the road** -- discovered lane centres,
  live density ribbons on every discovered lane path, and queue readouts. The
  ribbons all sample the same road length, so width means traffic density rather
  than a road's physical width.
  Drawing them back in image space is the visual proof that the ground frame is
  correct, because a wrong homography puts the lane lines off the carriageway.
"""
from __future__ import annotations

import subprocess
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

# Keyed on body type, which is what the object layer actually produces. The
# previous version keyed on detector class and had no entry for `two_wheeler`
# -- 30.6% of all detections, the largest non-car class, silently rendered as
# anonymous grey.
COLOUR = {  # BGR
    "two-wheeler": (60, 170, 255), "auto-rickshaw": (60, 240, 240),
    "hatchback": (80, 200, 90), "sedan": (120, 220, 120),
    "SUV / MUV": (200, 170, 60), "van / LCV": (200, 90, 200),
    "bus": (230, 120, 60), "truck": (170, 80, 170),
    "pedestrian": (90, 90, 250),
    # fallbacks for detector classes when no body type was solved
    "car": (80, 200, 90), "two_wheeler": (60, 170, 255),
    "person": (90, 90, 250), "motorcycle": (60, 170, 255),
    "autorickshaw": (60, 240, 240),
}
ABBREV = {"two-wheeler": "2W", "auto-rickshaw": "auto", "hatchback": "hatch",
          "sedan": "sedan", "SUV / MUV": "SUV", "van / LCV": "van",
          "bus": "bus", "truck": "truck", "pedestrian": "ped",
          "two_wheeler": "2W", "car": "car", "person": "ped",
          "autorickshaw": "auto"}
TRAIL_S = 4.0


def _hex_to_bgr(h):
    if not isinstance(h, str) or len(h) != 7:
        return None
    return (int(h[5:7], 16), int(h[3:5], 16), int(h[1:3], 16))


def _leg_frame(bearing_deg):
    b = np.radians(bearing_deg)
    along = np.array([np.sin(b), np.cos(b)])
    return along, np.array([along[1], -along[0]])


def _poly_px(gp, pts, scale):
    """Ground metres -> image pixels at render scale, dropping anything off-frame."""
    uv = gp.to_image(np.asarray(pts, dtype=float)) * scale
    uv = uv[np.isfinite(uv).all(axis=1)]
    return uv.astype(np.int32) if len(uv) > 1 else None


def _lane_path_density(traj: pd.DataFrame, legs: dict, lane_offsets: dict,
                       fps: float, d_min: float = 12.0, d_max: float = 60.0):
    """Live density for *each discovered lane path*, in vehicles/km/lane.

    The previous approach-level measure divided the total across lanes then
    rendered it on the median lane centre. That left most visible traffic paths
    unrepresented. Here each tracked sample is assigned to its nearest
    discovered lateral mode before density is calculated, and every mode gets
    its own ribbon.
    """
    max_fi = int(traj.fi.max()) if len(traj) else 0
    frames = pd.RangeIndex(max_fi + 1)
    d = traj[~traj.imputed]
    out = {}
    for i, nm in enumerate(legs["names"]):
        offsets = np.asarray(lane_offsets.get(nm, [0.0]), dtype=float)
        along, across = _leg_frame(legs["bearings"][i])
        rel = np.column_stack([d.x_m.to_numpy() - legs["centre"][0],
                               d.y_m.to_numpy() - legs["centre"][1]])
        s, q = rel @ along, rel @ across
        use = (s >= d_min) & (s <= d_max) & (np.abs(q) <= 14.0)
        path = d.loc[use, ["fi", "track_id"]].copy()
        path["lane_path"] = np.argmin(
            np.abs(q[use, None] - offsets[None, :]), axis=1)
        for lane_i in range(len(offsets)):
            n = (path.loc[path.lane_path == lane_i].groupby("fi")["track_id"]
                     .nunique().reindex(frames, fill_value=0))
            # Smooth over 1.5 seconds so the visual shows traffic state, not a
            # tracker blink. Each series is already one lane path: do not
            # divide by the number of lanes again.
            n = n.rolling(max(3, int(round(1.5 * fps))), center=True, min_periods=1).mean()
            out[(nm, lane_i)] = n * (1000.0 / (d_max - d_min))
    return out


def _density_style(density_vpkm: float):
    """Colour and ribbon width for a leg's live traffic density."""
    if density_vpkm < 60:
        level, colour = "LIGHT", (70, 190, 85)       # green, BGR
    elif density_vpkm < 140:
        level, colour = "BUSY", (20, 190, 255)       # amber
    else:
        level, colour = "DENSE", (45, 80, 235)       # red
    # At half-resolution output this spans 7--29 px: clearly different from
    # both vehicle boxes (1 px) and lane-centre dashes (2 px).
    width = int(np.clip(4 + 0.12 * density_vpkm, 7, 29))
    return level, colour, width


def _mapped_path_states(mapmatch: pd.DataFrame | None, fps: float,
                        sample_m: float = 48.0, max_visual_residual_m: float = 4.0) -> dict[int, list[dict]]:
    """Live states for map-matched (link, approach, direction, lane) paths.

    These are deliberately based on the road-network binding, not the
    motion-inferred leg geometry. Each state retains the nearest point and
    tangent on the registered OSM link, which lets the overlay place a density
    ribbon exactly along the road segment that its trajectories were assigned to.
    """
    if mapmatch is None or mapmatch.empty:
        return {}
    required = {"fi", "track_id", "link_id", "link_x_m", "link_y_m",
                "link_ux", "link_uy", "link_offset_m", "link_dir", "lane",
                "link_s_m", "link_length_m", "link_dist_m"}
    if not required.issubset(mapmatch.columns):
        return {}

    d = mapmatch[(mapmatch.link_id.notna()) & (mapmatch.lane.notna()) &
                 (mapmatch.link_ux.notna()) & (mapmatch.link_uy.notna()) &
                 (mapmatch.link_dist_m <= max_visual_residual_m)].copy()
    if d.empty:
        return {}
    d["link_id_int"] = d.link_id.astype(int)
    d["lane_int"] = d.lane.astype(int)
    d["dir_name"] = np.where(d.link_dir >= 0, "fwd", "rev")
    d["path_key"] = (d.link_id_int.astype(str) + "|" +
                     d.carriageway.fillna("?").astype(str) + "|" +
                     d.lane_int.astype(str) + "|" + d.dir_name)

    # One density series per actual map path. Re-index each path through the
    # full frame range so the 1.5 s smoother represents traffic state, rather
    # than tracker blink, at the video cadence.
    frames = pd.RangeIndex(int(d.fi.max()) + 1)
    counts = d.groupby(["path_key", "fi"])["track_id"].nunique()
    density = {}
    for key, g in counts.groupby(level=0):
        s = (g.droplevel(0).reindex(frames, fill_value=0)
               .rolling(max(3, int(round(1.5 * fps))), center=True, min_periods=1)
               .mean() * (1000.0 / sample_m))
        density.update({(key, int(fi)): float(v) for fi, v in s.items()})

    def mode_or_blank(s):
        s = s.dropna()
        return str(s.mode().iloc[0]) if len(s) else "?"

    live = (d.groupby(["fi", "path_key"])
              .agg(link_id=("link_id_int", "first"),
                   approach=("origin", mode_or_blank),
                   lane=("lane_int", "first"),
                   direction=("dir_name", "first"),
                   n_tracks=("track_id", "nunique"),
                   x=("link_x_m", "median"), y=("link_y_m", "median"),
                   ux=("link_ux", "median"), uy=("link_uy", "median"),
                   offset=("link_offset_m", "median"),
                   s=("link_s_m", "median"), length=("link_length_m", "median"))
              .reset_index())
    out = {}
    for r in live.itertuples():
        out.setdefault(int(r.fi), []).append({
            "key": r.path_key, "link_id": int(r.link_id),
            "approach": r.approach, "lane": int(r.lane),
            "direction": r.direction, "n_tracks": int(r.n_tracks),
            "x": float(r.x), "y": float(r.y), "ux": float(r.ux), "uy": float(r.uy),
            "offset": float(r.offset), "s": float(r.s), "length": float(r.length),
            "density": density.get((r.path_key, int(r.fi)), 0.0),
        })
    return out


def render(window: Path, traj: pd.DataFrame, det: pd.DataFrame, dest: Path,
           fps: float = 10.0, max_frames: int | None = None,
           out_fps: float = 10.0, scale: float = 0.5,
           objs: pd.DataFrame | None = None,
           gp=None, agg: dict | None = None, legs: dict | None = None,
           mapmatch: pd.DataFrame | None = None, mapmeta: dict | None = None):
    cap = cv2.VideoCapture(str(window))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) * scale)
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) * scale)

    tmp = dest.with_suffix(".raw.mp4")
    vw = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*"mp4v"), out_fps, (W, H))

    boxes = {k: v for k, v in det.groupby("fi")}
    tinfo = traj.set_index(["fi", "track_id"])[["speed_kph", "cls"]]

    # per-object attributes: what it is, how long it is, what colour it is
    ATTR = {}
    if objs is not None and not objs.empty:
        for r in objs.itertuples():
            ATTR[r.track_id] = (
                getattr(r, "body_type", None),
                getattr(r, "L_m", float("nan")),
                _hex_to_bgr(getattr(r, "hex", None)),
            )
    trails: dict[int, deque] = defaultdict(lambda: deque(maxlen=int(TRAIL_S * fps)))
    seen_ids: set[int] = set()

    # --- aggregate geometry, precomputed once in image space ---
    lane_polys, road_bands = [], []
    if gp is not None and agg and legs:
        by_leg = {L["leg"]: L for L in agg.get("lanes", [])}
        for i, nm in enumerate(legs["names"]):
            along, across = _leg_frame(legs["bearings"][i])
            L = by_leg.get(nm)
            if L:
                for off in L.get("lane_offsets_m", []):
                    pts = [along * d + across * off for d in np.linspace(12, 55, 24)]
                    poly = _poly_px(gp, pts, scale)
                    if poly is not None:
                        lane_polys.append(poly)
            q = (agg.get("queues") or {}).get(nm)
            offs = (L or {}).get("lane_offsets_m") or [0.0]
            road_bands.append({"name": nm, "along": along, "across": across,
                               "queue": q, "offsets": [float(off) for off in offs]})

    density_by_path = _lane_path_density(
        traj, legs, {r["name"]: r["offsets"] for r in road_bands}, fps
    ) if road_bands and legs else {}
    mapped_states = _mapped_path_states(mapmatch, fps)
    use_map_binding = bool(mapped_states)

    fd_by_t: dict[float, list] = {}
    if agg:
        for r in (agg.get("fundamental", {}) or {}).get("points", []):
            fd_by_t.setdefault(float(r["t"]), []).append(r)

    fi = 0
    while True:
        ok, frame = cap.read()
        if not ok or (max_frames and fi >= max_frames):
            break
        frame = cv2.resize(frame, (W, H))

        # --- live lane-path density ribbons --------------------------------
        # A common 48 m road sample makes the visual comparison meaningful.
        # Every discovered lane path receives its own band, so thickness means
        # only that path's density (veh/km/lane), never road geometry.
        t = fi / fps
        tabs = float(traj.t.min()) + t if len(traj) else t
        qtxt, density_labels, map_summary = [], [], []
        density_layer = frame.copy()
        for road in road_bands:
            q = road["queue"]
            qm = 0.0
            if q:
                arr = np.asarray(q["t"], dtype=float)
                if len(arr):
                    j = int(np.argmin(np.abs(arr - tabs)))
                    qm = float(q["queue_m"][j])
            qtxt.append(f"{road['name']} Q{qm:.0f}m")

        frame_states = mapped_states.get(fi, [])
        if use_map_binding:
            # Place the capsule on the registered OSM segment at the map-match
            # projection point, then offset it by the assigned lane position.
            # This is the visual proof that density belongs to a road link, not
            # merely to a straight line inferred from a movement bearing.
            for state in sorted(frame_states, key=lambda s: s["density"]):
                level, colour, width = _density_style(state["density"])
                u = np.array([state["ux"], state["uy"]], dtype=float)
                u /= max(np.linalg.norm(u), 1e-9)
                left = np.array([-u[1], u[0]])
                centre = (np.array([state["x"], state["y"]]) +
                          left * state["offset"])
                # Never extrapolate a short OSM segment beyond its geometry:
                # that was the source of capsules appearing in buildings.
                s0 = max(0.0, state["s"] - 24.0)
                s1 = min(state["length"], state["s"] + 24.0)
                if s1 - s0 < 3.0:
                    continue
                pts = [centre + u * (s0 - state["s"]),
                       centre + u * (s1 - state["s"])]
                poly = _poly_px(gp, pts, scale)
                if poly is None:
                    continue
                cv2.polylines(density_layer, [poly], False, (20, 20, 20), width + 6, cv2.LINE_AA)
                cv2.polylines(density_layer, [poly], False, colour, width, cv2.LINE_AA)
                arrow = ">" if state["direction"] == "fwd" else "<"
                lid = str(state["link_id"])[-5:]
                lx, ly = int(poly[-1][0]) + 6, int(poly[-1][1]) - 5
                # All active map paths receive a ribbon. Labels are reserved
                # for paths carrying at least two live road users (or a dense
                # one-user path), otherwise a busy junction becomes illegible.
                if state["n_tracks"] >= 2 or state["density"] >= 60:
                    density_labels.append((
                        f"#{lid} {state['approach']} {arrow} L{state['lane']} {state['density']:.0f}",
                        (lx, ly), colour))
                map_summary.append(f"#{lid}:{state['approach']}{arrow}L{state['lane']}")
        else:
            # Compatibility fallback for analyses where the map-match sidecar
            # has not yet been generated.
            for road in road_bands:
                for lane_i, offset in enumerate(road["offsets"]):
                    series = density_by_path.get((road["name"], lane_i))
                    density = (float(series.iloc[min(fi, len(series) - 1)])
                               if series is not None and len(series) else 0.0)
                    level, colour, width = _density_style(density)
                    pts = [road["along"] * d + road["across"] * offset
                           for d in np.linspace(12.0, 60.0, 28)]
                    poly = _poly_px(gp, pts, scale)
                    if poly is not None:
                        cv2.polylines(density_layer, [poly], False, (20, 20, 20), width + 6, cv2.LINE_AA)
                        cv2.polylines(density_layer, [poly], False, colour, width, cv2.LINE_AA)
                        lx, ly = int(poly[-1][0]) + 6, int(poly[-1][1]) - 5
                        density_labels.append((f"{road['name']} P{lane_i + 1} {density:.0f}",
                                               (lx, ly), colour))
        if frame_states or road_bands:
            cv2.addWeighted(density_layer, 0.52, frame, 0.48, 0, frame)

        cur = boxes.get(fi)
        live = set()
        if cur is not None:
            for r in cur.itertuples():
                tid = r.track_id
                live.add(tid)
                seen_ids.add(tid)
                cx = (r.x1 + r.x2) / 2 * scale
                cy = (r.y1 + r.y2) / 2 * scale
                trails[tid].append((int(cx), int(cy)))

                try:
                    info = tinfo.loc[(fi, tid)]
                    cls, spd = info.cls, float(info.speed_kph)
                except KeyError:
                    cls, spd = r.cls, float("nan")

                body, L, paint = ATTR.get(tid, (None, float("nan"), None))
                key = body if isinstance(body, str) and body != "unknown" else cls
                col = COLOUR.get(key, (200, 200, 200))

                p1 = (int(r.x1 * scale), int(r.y1 * scale))
                p2 = (int(r.x2 * scale), int(r.y2 * scale))
                cv2.rectangle(frame, p1, p2, col, 1)

                bits = [ABBREV.get(key, str(key)[:5])]
                if np.isfinite(L):
                    bits.append(f"{L:.1f}m")
                if np.isfinite(spd):
                    bits.append(f"{spd:.0f}")
                lbl = " ".join(bits)

                ty = max(11, p1[1] - 4)
                tx = p1[0]
                # a chip of the measured paint colour, so the extraction can be
                # checked against the vehicle it was taken from
                if paint is not None:
                    cv2.rectangle(frame, (tx, ty - 7), (tx + 6, ty - 1), paint, -1)
                    cv2.rectangle(frame, (tx, ty - 7), (tx + 6, ty - 1), (40, 40, 40), 1)
                    tx += 9
                cv2.putText(frame, lbl, (tx, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.34, col, 1, cv2.LINE_AA)

        for tid, pts in list(trails.items()):
            if tid not in live:
                pts.popleft() if pts else trails.pop(tid, None)
                continue
            if len(pts) > 1:
                b2, _, _ = ATTR.get(tid, (None, None, None))
                c2 = tinfo.loc[(fi, tid)].cls if (fi, tid) in tinfo.index else "car"
                col = COLOUR.get(b2 if isinstance(b2, str) and b2 != "unknown" else c2,
                                 (200, 200, 200))
                dim = tuple(int(c * 0.55) for c in col)
                cv2.polylines(frame, [np.array(pts, np.int32)], False, dim, 1, cv2.LINE_AA)

        # In map-native mode the road-aligned ribbons are the lane visual. Do
        # not overlay the old bearing-derived dashed lines, which can contradict
        # the registered road geometry near the junction.
        if not use_map_binding:
            for poly in lane_polys:
                for k in range(0, len(poly) - 1, 2):
                    cv2.line(frame, tuple(poly[k]), tuple(poly[k + 1]),
                             (30, 30, 30), 4, cv2.LINE_AA)
                    cv2.line(frame, tuple(poly[k]), tuple(poly[k + 1]),
                             (255, 255, 255), 2, cv2.LINE_AA)

        # Labels land above vehicle boxes, while the ribbons themselves sit
        # behind the tracked objects so both traffic state and evidence remain readable.
        for txt, pos, colour in density_labels:
            cv2.putText(frame, txt, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, txt, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        colour, 1, cv2.LINE_AA)

        # live flow / density / speed for this instant, from the Edie boxes
        if fd_by_t:
            key = min(fd_by_t, key=lambda k: abs(k - tabs))
            rowsv = fd_by_t[key]
            q_v = float(np.mean([r["q"] for r in rowsv]))
            k_v = float(np.mean([r["k"] for r in rowsv]))
            v_v = float(np.mean([r["v"] for r in rowsv]))
            o_v = float(np.mean([r["occ"] for r in rowsv]))
            pw, ph = 292, 132
            x0, y0 = W - pw - 12, 44
            panel = frame.copy()
            cv2.rectangle(panel, (x0, y0), (x0 + pw, y0 + ph), (18, 18, 18), -1)
            cv2.addWeighted(panel, 0.78, frame, 0.22, 0, frame)
            cv2.rectangle(frame, (x0, y0), (x0 + pw, y0 + ph), (110, 110, 110), 1)
            cv2.putText(frame, "AGGREGATE  (Edie, per lane)", (x0 + 12, y0 + 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (150, 220, 255), 1, cv2.LINE_AA)
            for i, (lab, val) in enumerate([
                    ("flow", f"{q_v:7.0f} veh/h/lane"),
                    ("density", f"{k_v:7.1f} veh/km/lane"),
                    ("space-mean speed", f"{v_v:7.1f} km/h"),
                    ("occupancy", f"{o_v:7.1f} %")]):
                yy = y0 + 44 + i * 21
                cv2.putText(frame, lab, (x0 + 12, yy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (185, 185, 185), 1, cv2.LINE_AA)
                cv2.putText(frame, val, (x0 + 150, yy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1, cv2.LINE_AA)

        # Make the geospatial claim inspectable in the video itself. The
        # density labels on the road use this same binding: #link, approach,
        # digitised-road direction and lane, refreshed at each 10-fps sample.
        if use_map_binding:
            align = (mapmeta or {}).get("alignment", {})
            match_rate = (mapmeta or {}).get("match_rate", "?")
            residual = align.get("median_dist_after_m", "?")
            active_bound = sum(s["n_tracks"] for s in frame_states)
            pw, ph = 340, 112
            x0, y0 = W - pw - 12, 184
            panel = frame.copy()
            cv2.rectangle(panel, (x0, y0), (x0 + pw, y0 + ph), (18, 18, 18), -1)
            cv2.addWeighted(panel, 0.78, frame, 0.22, 0, frame)
            cv2.rectangle(frame, (x0, y0), (x0 + pw, y0 + ph), (110, 110, 110), 1)
            cv2.putText(frame, "MAP BINDING  GPS + pose -> ground -> OSM", (x0 + 12, y0 + 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 220, 255), 1, cv2.LINE_AA)
            for i, txt in enumerate([
                    f"10 fps  |  {match_rate}% trajectory samples matched",
                    f"registration residual: {residual} m median",
                    f"active binding: {active_bound}/{len(live)}  link + approach + dir + lane",
                    "ribbon label: #link  approach  >/<  L(lane)"]):
                cv2.putText(frame, txt, (x0 + 12, y0 + 43 + i * 17),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.39, (235, 235, 235), 1, cv2.LINE_AA)

        cv2.rectangle(frame, (0, 0), (W, 34), (0, 0, 0), -1)
        if qtxt:
            cv2.rectangle(frame, (0, H - 26), (W, H), (0, 0, 0), -1)
            footer = ("MAP-MATCHED RIBBONS  each lies on its registered OSM road segment"
                      if use_map_binding else
                      "PATH DENSITY RIBBONS  every discovered traffic path is shown")
            cv2.putText(frame, footer + "   |  width = live veh/km/lane over 48 m"
                        "   |  L/B/D = light/busy/dense   |  " + "  ".join(qtxt),
                        (10, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                        (150, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"t={t:6.1f}s   tracked now: {len(live):3d}   "
                            f"cumulative ids: {len(seen_ids):4d}   "
                            f"label: body type | measured length | km/h   "
                            f"chip = measured paint colour"
                            + ("   |  map label = #link approach dir lane" if use_map_binding else ""),
                    (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        vw.write(frame)
        fi += 1

    cap.release()
    vw.release()

    subprocess.run(["ffmpeg", "-v", "error", "-i", str(tmp),
                    "-c:v", "libx264", "-crf", "24", "-preset", "veryfast",
                    "-pix_fmt", "yuv420p", str(dest), "-y"], check=True)
    tmp.unlink(missing_ok=True)
    return dest


if __name__ == "__main__":
    import argparse

    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="intersection")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--video", default="Intersection_Merged-002")
    ap.add_argument("--t0", type=float, default=240.0)
    a = ap.parse_args()

    traj = pd.read_parquet(root / "out" / f"trajectories_{a.tag}.parquet")
    det = pd.read_parquet(root / "out" / f"detections_{a.tag}.parquet")
    op = root / "out" / f"objects_{a.tag}.parquet"
    objs = pd.read_parquet(op) if op.exists() else None

    # ground frame + aggregate results, so lane centres and queues can be drawn
    import json
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import insights as ins
    import telemetry as tel
    from geometry import GroundPlane

    rep = json.loads((root / "out" / f"report_{a.tag}.json").read_text())
    mp = root / "out" / f"mapmatch_{a.tag}.parquet"
    mapmatch = pd.read_parquet(mp) if mp.exists() else None
    mb = root / "out" / f"mapnative_{a.tag}.json"
    mapmeta = json.loads(mb.read_text()) if mb.exists() else None
    tm = tel.load(root / f"{a.video}.MP4", root / "out" / "cache")
    row = tm.iloc[int(a.t0 * 30000 / 1001)]
    gp = GroundPlane.from_telemetry(3840, 2160, row.focal_len, row.rel_alt,
                                    row.gb_yaw, row.gb_pitch, row.gb_roll,
                                    zoom=row.dzoom)
    gp.scale = rep["quality"]["calibration"].get("scale", 1.0)
    gp.set_origin(np.array([[1920.0, 1080.0]]))
    legs = ins.find_legs(traj)

    dest = render(root / "out" / f"window_{a.tag}.mp4", traj, det,
                  root / "out" / f"overlay_{a.tag}.mp4",
                  fps=a.fps, max_frames=int(a.seconds * a.fps), objs=objs,
                  gp=gp, agg=rep.get("aggregate"), legs=legs,
                  mapmatch=mapmatch, mapmeta=mapmeta)
    print(f"-> {dest}  {dest.stat().st_size/1e6:.1f} MB")

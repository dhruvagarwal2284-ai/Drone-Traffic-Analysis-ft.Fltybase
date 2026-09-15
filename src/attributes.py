"""Object-level attributes: true metric dimensions, colour, size class, body type.

Three ideas do the work here.

**Oriented dimensions from an axis-aligned box.** The detector gives an
image-axis-aligned box, so its ground projection is a quadrilateral aligned to
the *image* axes, not the vehicle. Naively the longer side overstates length by
an amount that depends on heading. But the heading is known from the smoothed
trajectory, so the projection is invertible: measuring the ground extent along
both image axes gives two equations in length and width, and solving them
recovers the vehicle's true L and W without an oriented-box detector.

**Colour under a grey-world reference.** The footage is overcast evening light
on a wet road, which casts everything blue. Rather than classify raw pixels,
the road surface itself is used as a neutral reference to white-balance each
frame -- the road is grey by construction, so it makes a free colour chart.

**Roof sampling, not box sampling.** A bounding box contains road, shadow and
often part of a neighbour. Colour is taken from the central patch only, and
aggregated as a median across every frame of the track, so one bad sample
cannot move the answer.

Licence plates are deliberately absent: at 3.27 cm/px an Indian car plate is
15.3 x 3.7 px with 2.0 px character height against the ~16 px OCR needs, and at
-63 deg gimbal pitch the plate surfaces face away from the camera entirely. See
`plate_requirements()` for what tasking would actually be needed.
"""
from __future__ import annotations

import cv2
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# size / body-type bands, metres. Deliberately coarse: these are inferred from
# dimensions, not read off a badge, and pretending otherwise would be false
# precision.
BODY_BANDS = [
    # (name,        L_min, L_max, W_max)
    ("two-wheeler",   0.0,  2.60, 1.10),
    ("auto-rickshaw", 2.30,  3.10, 1.70),
    ("hatchback",     3.10,  4.05, 2.00),
    ("sedan",         4.05,  4.75, 2.05),
    ("SUV / MUV",     4.30,  5.30, 2.30),
    ("van / LCV",     4.60,  6.50, 2.60),
    ("truck",         6.00, 12.00, 3.20),
    ("bus",           8.00, 15.00, 3.20),
]

SIZE_CLASS = [("XS", 0.0, 2.6), ("S", 2.6, 4.05), ("M", 4.05, 5.3),
              ("L", 5.3, 8.0), ("XL", 8.0, 99.0)]

# colour vocabulary in HSV terms after white balance
CHROMATIC = [("red", 0, 12), ("orange", 12, 24), ("yellow", 24, 38),
             ("green", 38, 85), ("cyan", 85, 100), ("blue", 100, 135),
             ("violet", 135, 160), ("red", 160, 180)]


def plate_requirements(char_mm: float = 65.0, need_px: float = 16.0,
                       gsd_cm_px: float = 3.27, height_m: float = 106.6,
                       focal_35mm: float = 24.0) -> dict:
    """What it would take to actually read a plate from the air.

    Reported so the infeasibility is actionable rather than a dead end.
    """
    have_px = char_mm / (gsd_cm_px * 10.0)
    factor = need_px / have_px
    return {
        "character_height_px_now": round(have_px, 2),
        "character_height_px_needed": need_px,
        "shortfall_factor": round(factor, 1),
        "required_gsd_mm_px": round(char_mm / need_px, 1),
        "option_lower_altitude_m": round(height_m / factor, 1),
        "option_longer_lens_mm_equiv": round(focal_35mm * factor),
        "note": ("Resolution is only half the problem: at -63 deg pitch the camera sees "
                 "roofs, and plates sit on vertical fore/aft surfaces. Reading plates "
                 "needs a shallow-angle or ground-level camera, not just more pixels."),
    }


# ---------------------------------------------------------------------------
# true metric dimensions
# ---------------------------------------------------------------------------
def oriented_dims(gp, u, v, w_px, h_px, heading_deg):
    """Recover true (length, width) in metres from an axis-aligned image box.

    The ground projection of the box has extent `a` along the ground direction
    of the image x-axis and `b` along that of the image y-axis. For a rectangle
    of size L x W at angle phi to those directions:

        a = L|cos phi_x| + W|sin phi_x|
        b = L|cos phi_y| + W|sin phi_y|

    Two equations, two unknowns. Ill-conditioned when the vehicle sits at ~45
    deg to both axes, so those samples are rejected and the estimate is taken as
    a median over the whole track.
    """
    uv = np.column_stack([u, v])
    p0 = gp.to_ground(uv)
    px = gp.to_ground(uv + np.array([1.0, 0.0]))
    py = gp.to_ground(uv + np.array([0.0, 1.0]))

    dx, dy = px - p0, py - p0
    mx = np.linalg.norm(dx, axis=1)
    my = np.linalg.norm(dy, axis=1)
    a = w_px * mx                     # ground extent along image-x
    b = h_px * my                     # ground extent along image-y

    ex = dx / np.maximum(mx, 1e-9)[:, None]
    ey = dy / np.maximum(my, 1e-9)[:, None]

    th = np.radians(heading_deg)
    hd = np.column_stack([np.sin(th), np.cos(th)])   # heading unit vector (E,N)

    c1 = np.abs(np.einsum("ij,ij->i", hd, ex))
    s1 = np.sqrt(np.clip(1 - c1 ** 2, 0, 1))
    c2 = np.abs(np.einsum("ij,ij->i", hd, ey))
    s2 = np.sqrt(np.clip(1 - c2 ** 2, 0, 1))

    det = c1 * s2 - s1 * c2
    ok = np.abs(det) > 0.20           # reject the ill-conditioned ~45 deg case
    L = np.full(len(u), np.nan)
    W = np.full(len(u), np.nan)
    L[ok] = (a[ok] * s2[ok] - b[ok] * s1[ok]) / det[ok]
    W[ok] = (b[ok] * c1[ok] - a[ok] * c2[ok]) / det[ok]

    bad = ~np.isfinite(L) | ~np.isfinite(W) | (L <= 0) | (W <= 0) | (L > 20) | (W > 4)
    L[bad] = np.nan
    W[bad] = np.nan
    return L, W


# ---------------------------------------------------------------------------
# colour
# ---------------------------------------------------------------------------
def grey_world_gains(frame: np.ndarray, boxes: np.ndarray | None = None) -> np.ndarray:
    """Per-channel gains that make the road surface neutral.

    Samples the frame away from any detection, which is overwhelmingly road and
    pavement, and drives its mean to grey. Removes the blue cast of overcast
    evening light on wet asphalt so that colour names mean something.
    """
    h, w = frame.shape[:2]
    rng = np.random.default_rng(0)  # fixed seed: makes colour/hex reproducible across reruns
    ys = rng.integers(0, h, 4000)
    xs = rng.integers(0, w, 4000)
    keep = np.ones(len(ys), bool)
    if boxes is not None and len(boxes):
        for x1, y1, x2, y2 in boxes:
            keep &= ~((xs >= x1) & (xs <= x2) & (ys >= y1) & (ys <= y2))
    px = frame[ys[keep], xs[keep]].astype(np.float32)
    if len(px) < 200:
        return np.ones(3, np.float32)

    # mid-tones only: ignore deep shadow and blown highlights
    lum = px.mean(axis=1)
    px = px[(lum > 40) & (lum < 210)]
    if len(px) < 150:
        return np.ones(3, np.float32)

    # Keep only *achromatic* samples as the reference. This scene is full of
    # tree canopy and planted medians; averaging those in makes the reference
    # green, which then pushes every corrected pixel toward magenta. Road and
    # pavement are grey by construction, and selecting low-chroma pixels is what
    # isolates them without needing a road mask.
    mx = px.max(axis=1)
    mn = px.min(axis=1)
    chroma = (mx - mn) / np.maximum(mx, 1e-6)
    px = px[chroma < 0.18]
    if len(px) < 100:
        return np.ones(3, np.float32)

    ref = np.median(px, axis=0)
    gains = float(ref.mean()) / np.maximum(ref, 1e-6)
    return np.clip(gains, 0.6, 1.7).astype(np.float32)


def roof_patch(frame: np.ndarray, x1, y1, x2, y2, frac: float = 0.45):
    """Central patch of the box -- the roof, without road, shadow or neighbours."""
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    hw, hh = (x2 - x1) * frac / 2, (y2 - y1) * frac / 2
    a, b = int(max(0, cy - hh)), int(min(frame.shape[0], cy + hh))
    c, d = int(max(0, cx - hw)), int(min(frame.shape[1], cx + hw))
    if b - a < 2 or d - c < 2:
        return None
    return frame[a:b, c:d]


def patch_colour(patch: np.ndarray, gains: np.ndarray):
    """Median BGR of a patch after white balance, rejecting specular pixels."""
    p = patch.reshape(-1, 3).astype(np.float32) * gains
    p = np.clip(p, 0, 255)
    lum = p.mean(axis=1)
    keep = (lum > 25) & (lum < 245)
    if keep.sum() < 8:
        keep = np.ones(len(p), bool)
    return np.median(p[keep], axis=0)


def fleet_neutral_gains(bgr: np.ndarray) -> np.ndarray:
    """Second-stage correction using the vehicle fleet as its own colour chart.

    Road-based white balance leaves a residual cast here, because wet asphalt
    reflecting an overcast sky genuinely *is* blue -- the "road is neutral"
    assumption is only approximately true. The fleet is a better reference: in
    this market roughly two-thirds of vehicles are white, silver, grey or black,
    so the *median* vehicle is achromatic by composition. Driving that median to
    neutral removes what the road reference could not.

    bgr: (N,3) median colour per track.
    """
    ref = np.median(np.asarray(bgr, dtype=float), axis=0)
    gains = float(ref.mean()) / np.maximum(ref, 1e-6)
    return np.clip(gains, 0.75, 1.35)


def name_colour(bgr: np.ndarray) -> str:
    """Map a white-balanced BGR triple to a colour name."""
    px = np.uint8([[bgr]])
    h, s, v = cv2.cvtColor(px, cv2.COLOR_BGR2HSV)[0, 0].astype(float)
    # NB: h stays in OpenCV's 0-179 units -- CHROMATIC is expressed in the same
    # units. Rescaling it to degrees here pushed half of all vehicles past the
    # last band and into "other".
    if v < 55:
        return "black"
    if s < 62:
        return "white" if v > 172 else ("silver / grey" if v > 104 else "dark grey")
    if s < 88 and v > 165:
        return "white"
    for nm, lo, hi in CHROMATIC:
        if lo <= h < hi:
            return nm
    return "other"


# ---------------------------------------------------------------------------
def classify_body(L: float, W: float, cls: str) -> str:
    if not np.isfinite(L):
        return "unknown"
    if cls == "person":
        return "pedestrian"
    if cls in ("bus", "truck"):
        return cls
    best, score = "unknown", -1.0
    for nm, lo, hi, wmax in BODY_BANDS:
        if lo <= L <= hi:
            fit = 1.0 - abs(L - (lo + hi) / 2) / max((hi - lo) / 2, 1e-6)
            # Width is the ill-conditioned half of the L/W solve and measures
            # ~30% wide against known vehicles, so it only breaks ties.
            if np.isfinite(W) and W > wmax * 1.35:
                fit -= 0.15
            if fit > score:
                best, score = nm, fit
    return best


def size_class(L: float) -> str:
    if not np.isfinite(L):
        return "?"
    for nm, lo, hi in SIZE_CLASS:
        if lo <= L < hi:
            return nm
    return "?"


# ---------------------------------------------------------------------------
def build(video, det: pd.DataFrame, traj: pd.DataFrame, gp,
          max_samples: int = 18) -> pd.DataFrame:
    """One pass over the video, producing a per-track attribute record."""
    hd = traj.set_index(["fi", "track_id"])["heading"]
    det = det.join(hd, on=["fi", "track_id"], rsuffix="_h")
    det = det[det["heading"].notna()]

    # cap work per track: a handful of good frames beats every frame
    det = (det.sample(frac=1.0, random_state=0)
              .groupby("track_id", group_keys=False)
              .head(max_samples)
              .sort_values("fi"))

    L, W = oriented_dims(gp,
                         (det.x1.to_numpy() + det.x2.to_numpy()) / 2,
                         (det.y1.to_numpy() + det.y2.to_numpy()) / 2,
                         det.x2.to_numpy() - det.x1.to_numpy(),
                         det.y2.to_numpy() - det.y1.to_numpy(),
                         det["heading"].to_numpy())
    det = det.assign(L_m=L, W_m=W)

    # Read sequentially. Random seeking in 4K H.264 re-decodes from the nearest
    # keyframe every time, which costs far more than simply walking the file.
    cap = cv2.VideoCapture(str(video))
    by_frame = {int(k): v for k, v in det.groupby("fi")}
    wanted = set(by_frame)
    rows = []
    fi = -1
    while wanted:
        ok, frame = cap.read()
        if not ok:
            break
        fi += 1
        if fi not in wanted:
            continue
        wanted.discard(fi)
        g = by_frame[fi]
        gains = grey_world_gains(frame, g[["x1", "y1", "x2", "y2"]].to_numpy())
        for r in g.itertuples():
            p = roof_patch(frame, r.x1, r.y1, r.x2, r.y2)
            if p is None:
                continue
            bgr = patch_colour(p, gains)
            rows.append({"track_id": r.track_id, "cls": r.cls,
                         "b": bgr[0], "g": bgr[1], "r": bgr[2],
                         "L_m": r.L_m, "W_m": r.W_m})
    cap.release()

    s = pd.DataFrame(rows)
    if s.empty:
        return s
    agg = s.groupby("track_id").agg(
        cls=("cls", lambda x: x.mode().iloc[0]),
        b=("b", "median"), g=("g", "median"), r=("r", "median"),
        L_m=("L_m", "median"), W_m=("W_m", "median"),
        n_samples=("b", "size")).reset_index()

    fleet = agg[["b", "g", "r"]].to_numpy()
    gains = fleet_neutral_gains(fleet)
    fleet = np.clip(fleet * gains, 0, 255)
    agg[["b", "g", "r"]] = fleet

    agg["colour"] = [name_colour(np.array([b, gg, rr]))
                     for b, gg, rr in zip(agg.b, agg.g, agg.r)]
    agg["hex"] = ["#%02x%02x%02x" % (int(rr), int(gg), int(b))
                  for b, gg, rr in zip(agg.b, agg.g, agg.r)]
    agg["body_type"] = [classify_body(l, w, c)
                        for l, w, c in zip(agg.L_m, agg.W_m, agg.cls)]
    agg["size_class"] = [size_class(l) for l in agg.L_m]
    return agg

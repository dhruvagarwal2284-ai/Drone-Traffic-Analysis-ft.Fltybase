"""Synthetic regression test: association must bridge an occlusion gap.

Three simulated objects cross the scene on straight constant-velocity paths
(two of them, a motorcycle and a bicycle, literally cross paths mid-gap).
Detections are removed for a 3 s window in the middle -- the tree occlusion.
With mtrack's default `max_lost` (6 s), each object must come back as the
SAME track, not a new one: 3 tracks total, not 6.
"""
import sys
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from mtrack import DEFAULT_CFG, run  # noqa: E402


def _sim_track(x0, y0, vx, vy, cls, n_frames, dt, gap):
    rows = []
    for fi in range(n_frames):
        if gap[0] <= fi < gap[1]:
            continue
        t = fi * dt
        x_m, y_m = x0 + vx * t, y0 + vy * t
        rows.append({
            "fi": fi, "t": t, "cls": cls, "conf": 0.9,
            "x1": x_m, "y1": y_m, "x2": x_m + 1.0, "y2": y_m + 1.0,
            "u_px": x_m, "v_px": y_m,
            "x_m": x_m, "y_m": y_m, "w_m": 1.8, "h_m": 1.8,
        })
    return rows


def test_gap_bridging_keeps_three_tracks():
    fps = 10.0
    dt = 1.0 / fps
    n_frames = 90
    gap = (30, 60)  # frames 30..59 missing -> exactly 3 s at 10 fps

    rows = []
    rows += _sim_track(0.0, 0.0, 1.0, 0.3, "motorcycle", n_frames, dt, gap)
    rows += _sim_track(0.0, 10.0, 1.0, -0.3, "car", n_frames, dt, gap)
    rows += _sim_track(9.0, 0.0, -1.0, 0.3, "bicycle", n_frames, dt, gap)
    det = pd.DataFrame(rows)

    out = run(det, fps, DEFAULT_CFG)

    n_tracks = out.track_id.nunique()
    assert n_tracks == 3, f"expected 3 tracks (gap bridged), got {n_tracks}"
    # every track must have rows both before and after the gap (proves the
    # SAME id survived the occlusion, not that 3 tracks happened by luck)
    for tid, g in out.groupby("track_id"):
        assert g.fi.min() < gap[0] and g.fi.max() >= gap[1], (
            f"track {tid} does not span the gap: fi range {g.fi.min()}-{g.fi.max()}")


if __name__ == "__main__":
    test_gap_bridging_keeps_three_tracks()
    print("ok: 3 tracks, all span the occlusion gap")

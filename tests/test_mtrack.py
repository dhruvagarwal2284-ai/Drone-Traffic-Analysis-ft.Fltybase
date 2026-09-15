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


def test_class_hard_gate_blocks_person_into_car():
    """EXP4-FIX: a lone person detection must never join a car track, even
    when it sits exactly where the car's Kalman filter predicts its next
    position (the case EXP4-QA.md found 155/678 raw tracks doing under the
    old soft-cost-only gate: a stray pedestrian detection is spatially the
    solver's cheapest legal match onto a nearby vehicle track)."""
    fps = 10.0
    dt = 1.0 / fps
    n_frames = 20

    rows = []
    for fi in range(n_frames):
        if fi == 10:
            continue  # car misses this one frame
        t = fi * dt
        x_m = 1.0 * t
        rows.append({
            "fi": fi, "t": t, "cls": "car", "conf": 0.9,
            "x1": x_m, "y1": 0.0, "x2": x_m + 1.0, "y2": 1.0,
            "u_px": x_m, "v_px": 0.0,
            "x_m": x_m, "y_m": 0.0, "w_m": 4.0, "h_m": 1.8,
        })
    # a person detection at exactly the car's predicted position for fi=10
    t10 = 10 * dt
    x10 = 1.0 * t10
    rows.append({
        "fi": 10, "t": t10, "cls": "person", "conf": 0.9,
        "x1": x10, "y1": 0.0, "x2": x10 + 0.5, "y2": 0.5,
        "u_px": x10, "v_px": 0.0,
        "x_m": x10, "y_m": 0.0, "w_m": 0.5, "h_m": 0.5,
    })
    det = pd.DataFrame(rows)

    # gate ON (default): the person detection is blocked, never joins the car
    out = run(det, fps, DEFAULT_CFG)
    assert 10 not in set(out.fi), "person detection at fi=10 leaked into output with the hard gate on"
    assert set(out.cls) == {"car"}

    # sanity: gate OFF reproduces the leak this fix exists to stop, proving
    # the scenario is a real test of the gate and not a vacuous pass
    cfg_off = dict(DEFAULT_CFG, class_hard_gate=False)
    out_off = run(det, fps, cfg_off)
    assert 10 in set(out_off.fi), "expected the pre-fix soft-cost-only gate to leak the person detection"


if __name__ == "__main__":
    test_gap_bridging_keeps_three_tracks()
    test_class_hard_gate_blocks_person_into_car()
    print("ok: 3 tracks span the occlusion gap; class hard gate blocks person->car leak")

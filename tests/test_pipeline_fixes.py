"""Regression tests for two data-quality bugs fixed in the interview-polish pass.

1. Per-sample class leak: trajectories.build() used to forward-fill each
   frame's detector label, so 4 stray 'motorcycle' frames inside a 450-frame
   auto-rickshaw track created a phantom 'motorcycle' class in every
   per-sample aggregate. A road user must carry exactly one class.
2. Anomaly timestamps: insights.anomalies() stamped every event with the
   track's first sample, so everything already in view when the window opened
   was reported at t0. An event must be stamped when it happens.

Run with `python tests/test_pipeline_fixes.py` or pytest.
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import insights as ins  # noqa: E402
import trajectories as tj  # noqa: E402


class _FlatGround:
    """Stand-in GroundPlane: pixels are already metres. Only what build() uses."""

    scale = 1.0

    def image_to_ground(self, pts):
        return np.asarray(pts, dtype=float)


def _det(n=60, fps=10.0):
    rows = []
    for fi in range(n):
        x = 1.0 * fi / fps * 5          # 5 m/s along x
        cls = "motorcycle" if fi in (7, 21, 33) else "autorickshaw"
        rows.append({"fi": fi, "t": fi / fps, "track_id": 1, "cls": cls, "conf": 0.9,
                     "x1": x - 1.3, "y1": -0.7, "x2": x + 1.3, "y2": 0.7})
    return pd.DataFrame(rows)


def test_track_carries_one_majority_class(monkeypatch=None):
    det = _det()
    # isolate build() from projection/calibration: positions come straight
    # from the box centres, calibration is a no-op
    orig = (tj.calibrate, tj._box_ground, tj._footprint_extent)
    tj.calibrate = lambda gp, d: (1.0, {})
    tj._box_ground = lambda gp, d: np.c_[(d.x1 + d.x2) / 2, (d.y1 + d.y2) / 2]
    tj._footprint_extent = lambda gp, d: (d.x2 - d.x1).to_numpy()
    try:
        traj, _ = tj.build(det, _FlatGround(), 10.0)
    finally:
        tj.calibrate, tj._box_ground, tj._footprint_extent = orig
    assert traj.cls.nunique() == 1, f"track split across classes: {traj.cls.unique()}"
    # 2.6 m footprint is above merge_two_wheelers' 2.6 m cut only for car/truck,
    # so an autorickshaw keeps its own class
    assert traj.cls.iloc[0] == "autorickshaw", traj.cls.iloc[0]


def test_anomaly_stamped_when_it_happens():
    fps, dt = 10.0, 0.1
    rows = []
    # 20 background tracks establish the prevailing heading (east, 90 deg) in
    # every cell; track 99 is present from t=0 but only stops at t=5 s.
    for tid in range(20):
        for k in range(100):
            rows.append({"track_id": tid, "t": k * dt, "x_m": k * 0.5, "y_m": 0.0,
                         "speed_kph": 18.0, "heading": 90.0, "accel": 0.0,
                         "cls": "car", "imputed": False})
    for k in range(200):
        moving = k < 50
        rows.append({"track_id": 99, "t": k * dt, "x_m": min(k, 50) * 0.5, "y_m": 0.0,
                     "speed_kph": 18.0 if moving else 0.0, "heading": 90.0,
                     "accel": 0.0, "cls": "two_wheeler", "imputed": False})
    ev = ins.anomalies(pd.DataFrame(rows))
    stop = ev[(ev.track_id == 99) & (ev.kind == "stopped_in_carriageway")]
    assert len(stop) == 1, ev
    assert abs(stop.t.iloc[0] - 5.0) < 0.15, f"stop stamped at {stop.t.iloc[0]} s, expected ~5.0 s"


if __name__ == "__main__":
    test_track_carries_one_majority_class()
    test_anomaly_stamped_when_it_happens()
    print("ok: one class per track; anomalies stamped at event time")

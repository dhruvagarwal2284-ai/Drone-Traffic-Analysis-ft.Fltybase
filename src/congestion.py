"""Detector-free congestion signal over the *entire* recording.

The tracked window is 120 s, but the recording is 399 s. This module covers all
of it for almost no cost, using nothing but frame statistics, and it separates
the two states a single motion signal confuses:

    motion    = mean |frame_t - frame_{t-1}|   -> how much is moving
    occupancy = mean |frame_t - background|    -> how much is present

    empty road      low motion,  low occupancy
    free flow       high motion, high occupancy
    STANDING QUEUE  low motion,  HIGH occupancy   <- the state that matters

Statistics are accumulated on a coarse block grid rather than a hand-drawn
mask, so the active carriageway is *discovered* as the high-variance blocks
instead of being annotated. That keeps the method portable to any new site.
"""
from __future__ import annotations

import subprocess
from pathlib import Path

import numpy as np
import pandas as pd

W, H = 480, 270          # analysis resolution
GX, GY = 12, 9           # block grid (480x270 divides evenly)
SAMPLE_FPS = 2.0


def _frames(video: Path, fps: float = SAMPLE_FPS):
    cmd = ["ffmpeg", "-v", "error", "-hwaccel", "cuda", "-i", str(video),
           "-vf", f"fps={fps},scale={W}:{H}", "-pix_fmt", "gray",
           "-f", "rawvideo", "-"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=W * H * 16)
    n = W * H
    while True:
        buf = p.stdout.read(n)
        if len(buf) < n:
            break
        yield np.frombuffer(buf, np.uint8).reshape(H, W)
    p.stdout.close()
    p.wait()


def _blocks(img: np.ndarray) -> np.ndarray:
    """Mean over a GY x GX block grid."""
    return img.reshape(GY, H // GY, GX, W // GX).mean(axis=(1, 3))


def analyse(video: Path, fps: float = SAMPLE_FPS) -> tuple[pd.DataFrame, np.ndarray]:
    stack = [f.astype(np.float32) for f in _frames(video, fps)]
    arr = np.stack(stack)                      # (T,H,W)
    background = np.median(arr, axis=0)        # static scene

    motion, occ = [], []
    prev = None
    for f in arr:
        occ.append(_blocks(np.abs(f - background)))
        motion.append(_blocks(np.abs(f - prev)) if prev is not None
                      else np.zeros((GY, GX), np.float32))
        prev = f

    motion = np.stack(motion)
    occ = np.stack(occ)
    t = np.arange(len(arr)) / fps

    # active blocks = those whose occupancy actually varies (i.e. carriageway)
    var = occ.std(axis=0)
    active = var > np.percentile(var, 70)

    df = pd.DataFrame({
        "t": t,
        "motion": motion[:, active].mean(axis=1),
        "occupancy": occ[:, active].mean(axis=1),
    })
    # standing queue: present but not moving
    m = df["motion"] / df["motion"].max()
    o = df["occupancy"] / df["occupancy"].max()
    df["queue_index"] = (o * (1.0 - m)).rolling(5, center=True, min_periods=1).mean()
    return df, active


def cycle_length(sig: pd.Series, fps: float = SAMPLE_FPS,
                 lo: float = 30.0, hi: float = 200.0) -> dict:
    """Estimate a repeating cycle by autocorrelation of the congestion signal.

    If the junction is signal-gated, queue build-up and discharge repeat, and
    the peak of the autocorrelation inside a plausible band is the cycle time.
    """
    x = np.asarray(sig, dtype=float)
    x = x - x.mean()
    if x.std() < 1e-9:
        return {"cycle_s": None}
    ac = np.correlate(x, x, mode="full")[len(x) - 1:]
    ac /= ac[0]
    lags = np.arange(len(ac)) / fps
    band = (lags >= lo) & (lags <= hi)
    if not band.any():
        return {"cycle_s": None}
    k = np.flatnonzero(band)[np.argmax(ac[band])]
    return {"cycle_s": float(lags[k]), "peak_autocorr": float(ac[k]),
            "lags": lags.tolist(), "acf": ac.tolist()}


if __name__ == "__main__":
    import argparse
    import json

    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default="Intersection_Merged-002")
    ap.add_argument("--tag", default="intersection")
    a = ap.parse_args()

    df, active = analyse(root / f"{a.video}.MP4")
    df.to_parquet(root / "out" / f"congestion_{a.tag}.parquet", index=False)

    cyc = cycle_length(df["queue_index"])
    small = {k: v for k, v in cyc.items() if k not in ("lags", "acf")}
    (root / "out" / f"cycle_{a.tag}.json").write_text(json.dumps(cyc))

    print(f"{a.tag}: {len(df)} samples over {df.t.max():.0f}s, "
          f"{int(active.sum())}/{active.size} active blocks")
    print("cycle estimate:", json.dumps(small, indent=2))
    print(df.describe().round(3).to_string())

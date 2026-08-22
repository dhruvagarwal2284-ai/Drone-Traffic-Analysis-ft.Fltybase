"""Parse DJI per-frame telemetry embedded as a mov_text subtitle track.

The SRT carries one cue per video frame:

    FrameCnt: 0 2026-08-21 17:38:52.463
    [iso: 130] ... [latitude: 18.566227] [longitude: 73.771846]
    [rel_alt: 70.472 abs_alt: 607.273] [gb_yaw: -125.5 gb_pitch: -63.1 gb_roll: 0.0]

A "Merged" file is several DJI clips concatenated, so FrameCnt resets at each
boundary; we split on that.
"""
from __future__ import annotations

import re
import subprocess
from datetime import datetime
from pathlib import Path

import pandas as pd

_RE_HEAD = re.compile(r"FrameCnt:\s*(\d+)\s+([\d\-]+ [\d:.]+)")
_RE_KV = re.compile(r"\[?(\w+):\s*([-\d./]+)")

# fields we want as floats, with the SRT key they arrive under
_FLOAT_KEYS = {
    "latitude": "lat",
    "longitude": "lon",
    "rel_alt": "rel_alt",
    "abs_alt": "abs_alt",
    "gb_yaw": "gb_yaw",
    "gb_pitch": "gb_pitch",
    "gb_roll": "gb_roll",
    "focal_len": "focal_len",
    "dzoom_ratio": "dzoom",
    "iso": "iso",
}


def extract_srt(video: Path, dest: Path) -> Path:
    """Demux the subtitle track to .srt (idempotent)."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not dest.exists():
        subprocess.run(
            ["ffmpeg", "-v", "error", "-i", str(video),
             "-map", "0:s:0", "-c:s", "srt", str(dest), "-y"],
            check=True,
        )
    return dest


def parse_srt(srt: Path) -> pd.DataFrame:
    """One row per video frame, indexed by absolute frame number in the file."""
    text = srt.read_text(encoding="utf-8", errors="replace")
    rows = []
    for block in text.split("\n\n"):
        lines = block.strip().split("\n")
        if len(lines) < 3:
            continue
        head = _RE_HEAD.search(lines[2])
        if not head:
            continue
        kv = dict(_RE_KV.findall(block))
        row = {
            "frame_cnt": int(head.group(1)),
            "ts": datetime.strptime(head.group(2), "%Y-%m-%d %H:%M:%S.%f"),
        }
        for src, dst in _FLOAT_KEYS.items():
            try:
                row[dst] = float(kv[src])
            except (KeyError, ValueError):
                row[dst] = float("nan")
        rows.append(row)

    df = pd.DataFrame(rows)
    df["frame"] = range(len(df))
    # a clip boundary is where FrameCnt stops increasing
    resets = df.index[df["frame_cnt"].diff().fillna(1) <= 0].tolist()
    df["clip"] = 0
    for i, r in enumerate(resets, start=1):
        df.loc[r:, "clip"] = i
    return df


def load(video: Path, cache_dir: Path) -> pd.DataFrame:
    srt = extract_srt(video, cache_dir / (video.stem + ".srt"))
    return parse_srt(srt)


def clip_summary(df: pd.DataFrame) -> pd.DataFrame:
    """Per-clip pose envelope — used to justify the fixed-camera assumption."""
    g = df.groupby("clip")
    out = pd.DataFrame({
        "frames": g.size(),
        "start_s": g["frame"].min(),
        "dur_s": (g["ts"].max() - g["ts"].min()).dt.total_seconds(),
        "yaw_range": g["gb_yaw"].max() - g["gb_yaw"].min(),
        "pitch_range": g["gb_pitch"].max() - g["gb_pitch"].min(),
        "alt_range": g["rel_alt"].max() - g["rel_alt"].min(),
        "lat_drift_m": (g["lat"].max() - g["lat"].min()) * 111_320,
        "lon_drift_m": (g["lon"].max() - g["lon"].min()) * 111_320 * 0.9487,
    })
    return out.round(3)


if __name__ == "__main__":
    import sys

    root = Path(__file__).resolve().parent.parent
    cache = root / "out" / "cache"
    for name in ("Intersection_Merged-002", "Multi_Road_Merged-001"):
        df = load(root / f"{name}.MP4", cache)
        print(f"\n=== {name}: {len(df)} frames")
        print(clip_summary(df).to_string())

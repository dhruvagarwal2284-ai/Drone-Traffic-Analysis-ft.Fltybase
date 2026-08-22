"""Annotated overlay video: boxes, ids, class, speed, and trajectory trails.

The trails are the point. A box with a label is what every fielded system
already draws; the trail is the evidence that a complete path exists in a
common metric frame, which is what the rest of the analysis is built on.
"""
from __future__ import annotations

import subprocess
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

COLOUR = {  # BGR
    "car": (80, 200, 90), "motorcycle": (60, 170, 255),
    "auto_rickshaw": (60, 240, 240), "bus": (230, 120, 60),
    "truck": (200, 90, 200), "person": (90, 90, 250),
    "bicycle": (220, 220, 120),
}
TRAIL_S = 4.0


def render(window: Path, traj: pd.DataFrame, det: pd.DataFrame, dest: Path,
           fps: float = 10.0, max_frames: int | None = None,
           out_fps: float = 15.0, scale: float = 0.5):
    cap = cv2.VideoCapture(str(window))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) * scale)
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) * scale)

    tmp = dest.with_suffix(".raw.mp4")
    vw = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*"mp4v"), out_fps, (W, H))

    boxes = {k: v for k, v in det.groupby("fi")}
    tinfo = traj.set_index(["fi", "track_id"])[["speed_kph", "cls"]]
    trails: dict[int, deque] = defaultdict(lambda: deque(maxlen=int(TRAIL_S * fps)))
    seen_ids: set[int] = set()

    fi = 0
    while True:
        ok, frame = cap.read()
        if not ok or (max_frames and fi >= max_frames):
            break
        frame = cv2.resize(frame, (W, H))

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

                col = COLOUR.get(cls, (200, 200, 200))
                p1 = (int(r.x1 * scale), int(r.y1 * scale))
                p2 = (int(r.x2 * scale), int(r.y2 * scale))
                cv2.rectangle(frame, p1, p2, col, 1)
                lbl = f"{cls[:4]} {spd:.0f}" if np.isfinite(spd) else cls[:4]
                cv2.putText(frame, lbl, (p1[0], max(10, p1[1] - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1, cv2.LINE_AA)

        for tid, pts in list(trails.items()):
            if tid not in live:
                pts.popleft() if pts else trails.pop(tid, None)
                continue
            if len(pts) > 1:
                cls = tinfo.loc[(fi, tid)].cls if (fi, tid) in tinfo.index else "car"
                col = COLOUR.get(cls, (200, 200, 200))
                cv2.polylines(frame, [np.array(pts, np.int32)], False, col, 1, cv2.LINE_AA)

        t = fi / fps
        cv2.rectangle(frame, (0, 0), (W, 34), (0, 0, 0), -1)
        cv2.putText(frame, f"t={t:6.1f}s   tracked now: {len(live):3d}   "
                            f"cumulative ids: {len(seen_ids):4d}",
                    (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
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
    a = ap.parse_args()

    traj = pd.read_parquet(root / "out" / f"trajectories_{a.tag}.parquet")
    det = pd.read_parquet(root / "out" / f"detections_{a.tag}.parquet")
    dest = render(root / "out" / f"window_{a.tag}.mp4", traj, det,
                  root / "out" / f"overlay_{a.tag}.mp4",
                  fps=a.fps, max_frames=int(a.seconds * a.fps))
    print(f"-> {dest}  {dest.stat().st_size/1e6:.1f} MB")

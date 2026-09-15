"""Detection + tracking over a time window of the drone video.

Two stages, deliberately kept simple:

1. ffmpeg cuts the analysis window and resamples to the working frame rate,
   writing a small intermediate MP4 (also reused for the overlay video).
2. Ultralytics YOLO + ByteTrack runs over that file.

Association happens in image space. That is sound here because the camera is
static and the intersection view is steep (2.6 cm/px), so apparent scale barely
varies across the frame.
"""
from __future__ import annotations

import subprocess
from pathlib import Path

import pandas as pd

# COCO ids we care about. There is no auto-rickshaw class -- see reclassify_autos()
# in trajectories.py, which recovers it from metric footprint size.
COCO_KEEP = [0, 1, 2, 3, 5, 7]  # person, bicycle, car, motorcycle, bus, truck

# VisDrone (dronefreak/visdrone-yolov11s, EXP1.md): 11 classes, id 10 'others'
# dropped. Selected automatically when 'visdrone' is in the weights filename --
# mapped once here to this repo's class vocabulary; tricycle/awning-tricycle
# become a new 'autorickshaw' class (one-line addition in trajectories.py's
# CLASS_HEIGHT/PCU and conflicts.py's RADIUS; attributes.py's body-type bands
# already had an 'auto-rickshaw' size band, unchanged).
VISDRONE_KEEP = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
VISDRONE_MAP = {
    "pedestrian": "person", "people": "person", "bicycle": "bicycle",
    "car": "car", "van": "car", "truck": "truck", "bus": "bus",
    "motor": "motorcycle",
    "tricycle": "autorickshaw", "awning-tricycle": "autorickshaw",
}


def build_window(video: Path, t0: float, dur: float, fps: float, dest: Path) -> Path:
    """Cut [t0, t0+dur) at `fps`, keeping native 4K resolution."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists():
        return dest
    cmd = [
        "ffmpeg", "-v", "error", "-hwaccel", "cuda",
        "-ss", str(t0), "-i", str(video), "-t", str(dur),
        "-vf", f"fps={fps}",
        "-c:v", "h264_nvenc", "-cq", "20", "-preset", "p5",
        "-pix_fmt", "yuv420p", str(dest), "-y",
    ]
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:  # no NVENC -> fall back to x264
        cmd[cmd.index("h264_nvenc")] = "libx264"
        cmd[cmd.index("-cq")] = "-crf"
        cmd[cmd.index("p5")] = "veryfast"
        subprocess.run(cmd, check=True)
    return dest


def run(window_mp4: Path, weights: str = "yolo11x.pt", imgsz: int = 1920,
        conf: float = 0.20, t0: float = 0.0, fps: float = 10.0,
        device: str = "0", tracker: str = "bytetrack.yaml") -> pd.DataFrame:
    """Track through the window. Returns one row per detection per frame."""
    from ultralytics import YOLO

    model = YOLO(weights)
    names = model.names
    is_visdrone = "visdrone" in str(weights).lower()
    keep = VISDRONE_KEEP if is_visdrone else COCO_KEEP

    rows = []
    stream = model.track(
        source=str(window_mp4),
        stream=True,
        persist=True,
        tracker=tracker,
        classes=keep,
        imgsz=imgsz,
        conf=conf,
        iou=0.5,
        max_det=1000,
        half=True,
        device=device,
        verbose=False,
    )

    for fi, res in enumerate(stream):
        b = res.boxes
        if b is None or b.id is None:
            continue
        xyxy = b.xyxy.cpu().numpy()
        ids = b.id.cpu().numpy().astype(int)
        cls = b.cls.cpu().numpy().astype(int)
        cf = b.conf.cpu().numpy()
        for k in range(len(ids)):
            raw = names[int(cls[k])]
            rows.append({
                "fi": fi,
                "t": t0 + fi / fps,
                "track_id": int(ids[k]),
                "cls_id": int(cls[k]),
                "cls": VISDRONE_MAP.get(raw, raw) if is_visdrone else raw,
                "conf": float(cf[k]),
                "x1": float(xyxy[k, 0]), "y1": float(xyxy[k, 1]),
                "x2": float(xyxy[k, 2]), "y2": float(xyxy[k, 3]),
            })
        if fi % 100 == 0:
            print(f"  frame {fi:5d}  dets={len(rows)}", flush=True)

    return pd.DataFrame(rows)


if __name__ == "__main__":
    import argparse

    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=str(root / "Intersection_Merged-002.MP4"))
    ap.add_argument("--t0", type=float, default=240.0)
    ap.add_argument("--dur", type=float, default=120.0)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--weights", default="yolo11x.pt")
    ap.add_argument("--imgsz", type=int, default=1920)
    ap.add_argument("--tag", default="intersection")
    ap.add_argument("--window", default=None,
                    help="reuse an existing cut window mp4 instead of re-cutting")
    ap.add_argument("--tracker", default="bytetrack.yaml",
                    help="tracker yaml (ultralytics built-in name or path to a custom one)")
    a = ap.parse_args()

    if a.window:
        win = Path(a.window)
        print(f"window -> {win} (reused)", flush=True)
    else:
        win = build_window(Path(a.video), a.t0, a.dur, a.fps,
                           root / "out" / f"window_{a.tag}.mp4")
        print(f"window -> {win}", flush=True)

    df = run(win, a.weights, a.imgsz, t0=a.t0, fps=a.fps, tracker=a.tracker)
    dest = root / "out" / f"detections_{a.tag}.parquet"
    df.to_parquet(dest, index=False)
    print(f"\n{len(df)} detections, {df.track_id.nunique()} tracks -> {dest}")
    print(df.cls.value_counts().to_string())

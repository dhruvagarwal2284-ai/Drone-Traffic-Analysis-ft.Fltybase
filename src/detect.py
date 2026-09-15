"""Detector-only stage: per-frame YOLO detections on the ground plane, no tracking.

Step A of the tracker re-architecture (README next-step #1): today track.py fuses
detection and image-space ByteTrack in one call. This stage does only the
detection half, so a future metric-frame tracker (Step B) can associate boxes
itself instead of inheriting ByteTrack's image-space ids.

Reuses, rather than re-derives:
- track.py's VisDrone/COCO class list and name map (same detector, same classes).
- geometry.py's GroundPlane projection and trajectories.py's calibrate()/
  _box_ground() -- the exact scale rule (median car footprint -> 4.0 m,
  heading-unbiased) and mid-height back-projection the rest of the pipeline
  already trusts, so a future tracker's numbers stay comparable to today's.

Never re-cuts the window video; --tiles 2 is an optional 2x2 overlapping-tile
pass (SAHI-style) merged with per-class NMS, off by default.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from track import COCO_KEEP, VISDRONE_KEEP, VISDRONE_MAP
from geometry import GroundPlane
from trajectories import CLASS_HEIGHT, _box_ground, calibrate

EMBED_DIM = 20  # 16-bin hue histogram + mean S + mean V + normalised w,h


def _footprint_wh(gp: GroundPlane, df: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """Ground-plane box width/height, metres -- same projection as trajectories._footprint_extent,
    kept separate (w, h) instead of collapsed to max(w, h)."""
    zmid = df["cls"].map(CLASS_HEIGHT).fillna(1.5).to_numpy() / 2.0
    x1, y1 = df["x1"].to_numpy(), df["y1"].to_numpy()
    x2, y2 = df["x2"].to_numpy(), df["y2"].to_numpy()
    tl = gp.to_ground(np.column_stack([x1, y1]), plane_z=zmid)
    tr = gp.to_ground(np.column_stack([x2, y1]), plane_z=zmid)
    bl = gp.to_ground(np.column_stack([x1, y2]), plane_z=zmid)
    return np.linalg.norm(tr - tl, axis=1), np.linalg.norm(bl - tl, axis=1)


def _hsv_embed(frame_bgr: np.ndarray, x1, y1, x2, y2, img_w: int, img_h: int) -> list[float]:
    """Compact appearance vector: no ReID network, just enough colour/size signal
    for Step B's tracker to break ties ByteTrack-style IoU alone can't."""
    x1i, y1i = max(int(x1), 0), max(int(y1), 0)
    crop = frame_bgr[y1i:int(y2), x1i:int(x2)]
    if crop.size == 0:
        return [0.0] * EMBED_DIM
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0], None, [16], [0, 180]).flatten()
    hist = (hist / max(hist.sum(), 1.0)).tolist()
    mean_s, mean_v = float(hsv[..., 1].mean()) / 255.0, float(hsv[..., 2].mean()) / 255.0
    return hist + [mean_s, mean_v, (x2 - x1) / img_w, (y2 - y1) / img_h]


def _tile_origins(w: int, h: int, overlap: float = 0.2) -> list[tuple[int, int, int, int]]:
    """2x2 overlapping tile (x0, y0, x1, y1) origins covering a w x h frame."""
    hw, hh = w // 2, h // 2
    ow, oh = int(hw * overlap), int(hh * overlap)
    xs = [(0, hw + ow), (hw - ow, w)]
    ys = [(0, hh + oh), (hh - oh, h)]
    return [(x0, y0, x1, y1) for x0, x1 in xs for y0, y1 in ys]


def _detect_tiled(model, frame, keep, imgsz, conf, device):
    """4 overlapping-tile passes, boxes offset back to full-frame pixels, merged
    with per-class NMS (torchvision, already an installed dependency -- no new one)."""
    import torch
    from torchvision.ops import batched_nms

    h, w = frame.shape[:2]
    origins = _tile_origins(w, h)
    crops = [frame[y0:y1, x0:x1] for x0, y0, x1, y1 in origins]
    results = model.predict(crops, imgsz=imgsz, conf=conf, classes=keep, iou=0.5,
                            max_det=1000, half=True, device=device, verbose=False)
    boxes, cls_id, confs = [], [], []
    for (x0, y0, _, _), r in zip(origins, results):
        b = r.boxes
        if b is None or len(b) == 0:
            continue
        xy = b.xyxy.cpu().numpy()
        xy[:, [0, 2]] += x0
        xy[:, [1, 3]] += y0
        boxes.append(xy)
        cls_id.append(b.cls.cpu().numpy().astype(int))
        confs.append(b.conf.cpu().numpy())
    if not boxes:
        return np.zeros((0, 4)), np.zeros(0, int), np.zeros(0)
    boxes, cls_id, confs = np.concatenate(boxes), np.concatenate(cls_id), np.concatenate(confs)
    idx = batched_nms(torch.from_numpy(boxes).float(), torch.from_numpy(confs).float(),
                      torch.from_numpy(cls_id), iou_threshold=0.5).numpy()
    return boxes[idx], cls_id[idx], confs[idx]


def _detect_single(model, frame, keep, imgsz, conf, device):
    r = model.predict(frame, imgsz=imgsz, conf=conf, classes=keep, iou=0.5,
                      max_det=1000, half=True, device=device, verbose=False)[0]
    b = r.boxes
    if b is None or len(b) == 0:
        return np.zeros((0, 4)), np.zeros(0, int), np.zeros(0)
    return b.xyxy.cpu().numpy(), b.cls.cpu().numpy().astype(int), b.conf.cpu().numpy()


def run(window_mp4: Path, weights: str, imgsz: int, conf: float, t0: float, fps: float,
        tiles: int, embed: bool, device: str = "0") -> pd.DataFrame:
    """Detect through the window. Returns one row per detection per frame -- no track_id."""
    from ultralytics import YOLO

    model = YOLO(weights)
    names = model.names
    is_visdrone = "visdrone" in str(weights).lower()
    keep = VISDRONE_KEEP if is_visdrone else COCO_KEEP
    detect_frame = _detect_tiled if tiles == 2 else _detect_single

    cap = cv2.VideoCapture(str(window_mp4))
    rows = []
    fi = -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        fi += 1
        img_h, img_w = frame.shape[:2]
        xyxy, cls_id, confs = detect_frame(model, frame, keep, imgsz, conf, device)
        for k in range(len(cls_id)):
            raw = names[int(cls_id[k])]
            cls = VISDRONE_MAP.get(raw, raw) if is_visdrone else raw
            x1, y1, x2, y2 = (float(v) for v in xyxy[k])
            row = {
                "fi": fi, "t": t0 + fi / fps, "cls": cls, "conf": float(confs[k]),
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "u_px": (x1 + x2) / 2.0, "v_px": (y1 + y2) / 2.0,
            }
            if embed:
                row["embed"] = _hsv_embed(frame, x1, y1, x2, y2, img_w, img_h)
            rows.append(row)
        if fi % 100 == 0:
            print(f"  frame {fi:5d}  dets={len(rows)}", flush=True)
    cap.release()
    return pd.DataFrame(rows)


if __name__ == "__main__":
    import argparse
    import json

    import pyarrow as pa
    import pyarrow.parquet as pq

    import telemetry as tel

    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="intersection")
    ap.add_argument("--video", default="Intersection_Merged-002")
    ap.add_argument("--t0", type=float, default=240.0)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--weights", default="models/visdrone-yolov11s.pt")
    ap.add_argument("--imgsz", type=int, default=1920)
    ap.add_argument("--conf", type=float, default=0.20)
    ap.add_argument("--window", default=None,
                    help="reuse an existing cut window mp4 (this stage never re-cuts video)")
    ap.add_argument("--tiles", type=int, default=1, choices=[1, 2])
    ap.add_argument("--embed", action="store_true")
    a = ap.parse_args()

    win = Path(a.window) if a.window else root / "out" / f"window_{a.tag}.mp4"
    if not win.exists():
        raise SystemExit(f"{win} missing -- cut it with track.py first, or pass --window")
    print(f"window -> {win} (reused)", flush=True)

    det = run(win, a.weights, a.imgsz, a.conf, a.t0, a.fps, a.tiles, a.embed)

    tm = tel.load(root / f"{a.video}.MP4", root / "out" / "cache")
    row = tm.iloc[int(a.t0 * 30000 / 1001)]
    gp = GroundPlane.from_telemetry(3840, 2160, row.focal_len, row.rel_alt,
                                    row.gb_yaw, row.gb_pitch, row.gb_roll, zoom=row.dzoom)
    gp.set_origin(np.array([[1920.0, 1080.0]]))

    scale, cal = calibrate(gp, det)          # same scale rule as trajectories.py
    xy = _box_ground(gp, det)
    w_m, h_m = _footprint_wh(gp, det)
    det = det.assign(x_m=xy[:, 0], y_m=xy[:, 1], w_m=w_m, h_m=h_m)

    dest = root / "out" / f"detraw_{a.tag}.parquet"
    table = pa.Table.from_pandas(det, preserve_index=False)
    meta = dict(table.schema.metadata or {})
    meta[b"calibration"] = json.dumps(cal).encode()
    table = table.replace_schema_metadata(meta)
    pq.write_table(table, dest)

    print(json.dumps(cal, indent=2))
    print(f"\n{len(det)} detections, {det.fi.nunique()} frames -> {dest}")
    print(det.cls.value_counts().to_string())

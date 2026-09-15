# EXP4A — decouple detection from tracking (Step A)

Splits track.py's fused `model.track()` (detector + image-space ByteTrack) into a
detector-only stage, `src/detect.py`, so a future metric-frame tracker (Step B) can
associate boxes itself instead of inheriting ByteTrack's image-space ids. Branch
`mtrack`. Reused rather than re-derived: track.py's VisDrone class list/map,
trajectories.py's `calibrate()`/`_box_ground()` ground-plane projection and scale
rule. Detector/settings unchanged from config.yaml (`models/visdrone-yolov11s.pt`,
imgsz 1920, conf 0.20). Never re-cut video — both runs reuse `out/window_intersection.mp4`.

## Schema — `out/detraw_<tag>.parquet`

One row per detection per frame, **no `track_id`** (that's Step B's job):

| column | meaning |
|---|---|
| `fi`, `t` | frame index in window; absolute time in source video (s) |
| `cls`, `conf` | class (VisDrone→repo name map, same as track.py), detector confidence |
| `x1,y1,x2,y2` | box corners, px |
| `u_px,v_px` | box-centre, px — **note**: track.py itself computes no u_px/v_px; the
  only existing convention is trajectories.py's own `u_px=(x1+x2)/2` (box centre, not
  a bottom-centre foot point). Matched that exactly instead of the dispatch's
  "bottom-centre" description, which doesn't correspond to any code in this repo —
  verified by grep, not assumed. |
| `x_m,y_m` | ground-plane metres, via `geometry.GroundPlane.to_ground` at each class's
  mid-height (`trajectories.CLASS_HEIGHT`/2) — identical projection to `_box_ground` |
| `w_m,h_m` | box footprint width/height, metres, same mid-height projection as
  `trajectories._footprint_extent` but kept as two components instead of `max(w,h)` |
| `embed` (only with `--embed`) | 20 floats: 16-bin hue histogram (normalised) + mean
  S + mean V + normalised box w,h. No ReID network — deliberately simple |

Scale/calibration is written into the **parquet's own schema metadata** (key
`calibration`, JSON — `pyarrow.Table.replace_schema_metadata`), not a sidecar file,
so the two required outputs stay exactly `out/detraw_intersection.parquet` and
`out/detraw_inter_t2.parquet`.

## Calibration result

Both runs self-calibrate scale from their own raw detections (`trajectories.calibrate()`,
median car footprint → 4.0 m, heading-unbiased), independent of the tracked run:

| run | scale | effective height | motorcycle check |
|---|---|---|---|
| tracked run (existing, for reference) | 1.3847 | — | — |
| detect.py tiles=1 | **1.3827** | 97.44 m | 1.852 m vs 1.986 m expected (−6.7%) |
| detect.py tiles=2 | **1.3955** | 98.34 m | 1.873 m vs 1.986 m expected (−5.7%) |

Within 0.9% of the tracked run's scale on tiles=1 — the calibration only needs raw
per-detection boxes, so decoupling tracking changes nothing about it, as expected.

## Per-frame counts, tiles=1 vs tiles=2 (full 1200-frame `intersection` window)

| | tiles=1 | tiles=2 | ratio |
|---|---|---|---|
| total detections | 269,645 | 326,823 | **1.21×** |
| per frame (mean ± std) | 224.7 ± 13.8 | 272.4 ± 15.2 | |
| motorcycle | 142,406 (52.8%) | 148,008 (45.3%) | 1.04× |
| car | 64,357 (23.9%) | 74,542 (22.8%) | 1.16× |
| person | 48,067 (17.8%) | 81,441 (24.9%) | **1.69×** |
| autorickshaw | 9,208 (3.4%) | 10,795 (3.3%) | 1.17× |
| truck | 4,429 (1.6%) | 3,845 (1.2%) | **0.87×** |
| bus | 912 (0.3%) | 6,070 (1.9%) | **6.66×** |
| bicycle | 266 (0.1%) | 2,122 (0.6%) | **7.98×** |

Tiling does find more small road users — pedestrians +69%, bicycles +698% — matching
the README's expectation that tiling helps small classes, but the **1.21× overall
ratio is well under the 1.7–1.9× the write-up measured on COCO**. That's consistent
with the README's own read: VisDrone alone already recovers much of what tiling was
for (denser native detections than COCO), so tiling on top of VisDrone earns less
than tiling on top of COCO did.

Two results need a caveat, not a bare number: **bus 6.66× and truck 0.87×.** Both are
large vehicles that can straddle a tile seam. A plausible mechanism (not confirmed
further — flagging for Step B, not chasing it here): a large vehicle split across two
tiles can produce two partial, non-overlapping boxes that per-class NMS doesn't merge
(inflating bus), while a large vehicle's single best full-frame view can beat every
partial tile crop's confidence and get suppressed as a near-duplicate against a worse
partial box (deflating truck). Small/medium classes (motorcycle, car, autorickshaw)
don't show this instability — their ratios are all close to the frame-count ratio.

## Timings (RTX 5050, this machine)

| run | wall time | notes |
|---|---|---|
| tiles=1, 1200 frames | 1m 26s | ~70ms/frame steady-state (model load ~8s) |
| tiles=2, 1200 frames | 3m 27s | ~2.4× tiles=1, not 4×: tile crops batched per frame |

Both well under the 15 GPU-minute cap; combined under 5 minutes.

## Guidance for Step B (the tracker)

1. **Typical per-frame count**: ~225 detections/frame (tiles=1) or ~272 (tiles=2) over
   a scene with ~250 unique road users total — expect heavy multi-detection-per-track
   association load per frame, not a handful of objects.
2. **No `merge_two_wheelers()` has run** — `cls` is the raw per-detection VisDrone-mapped
   class (motorcycle/car/bicycle stay separate). That merge in trajectories.py runs at
   the *track* level using per-track median footprint; Step B should apply the same
   merge once it has track-level footprint medians, not per-detection, or it will
   misclassify individual noisy detections.
3. **Footpoint/position noise**: ground sampling distance is ~3.4–4.5 cm/px across the
   frame (computed via `GroundPlane.metres_per_pixel` at frame centre/edges). A few
   pixels of box-edge jitter on a small class (motorcycle, bicycle, person) is enough
   to move `x_m,y_m` by 10–20 cm frame-to-frame — size the association gate distance
   accordingly, not tighter.
4. **Large-class tiling instability** (bus/truck, see above): if Step B consumes the
   `--tiles 2` output, treat large-vehicle detections near tile seams as lower-trust —
   either widen the NMS IoU threshold for large boxes specifically, or prefer the
   `--tiles 1` detection for any track whose class is bus/truck/van.
5. **`embed` is unvalidated at scale** — smoke-tested only (20-dim HSV+size vector,
   no ReID network). It exists as a hook for Step B's association cost function, not a
   proven feature; treat it as a candidate signal to ablate, not a given.

## What's unchanged

`track.py`, `trajectories.py`, `config.yaml` untouched — every existing tag and
downstream stage runs exactly as before. `detect.py` is additive only.

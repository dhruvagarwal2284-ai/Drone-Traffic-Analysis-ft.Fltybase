# EXP1 — does an aerial-pretrained detector fix the "two-wheeler called car" problem?

Run 2026-09-15 by temp `worker-fh-exp1`. Experiment design taken as-is from `PREP.md`
§"Smallest experiment to prove a win" — not redesigned. No tracking, no metric
projection, single-pass detection only on 20 sampled frames. Env: torch 2.11+cu128,
RTX 5050, ultralytics 8.4.126.

## (1) Weights downloaded

| pick | HF repo | file | size | licence | classes confirmed via `ultralytics.YOLO()` |
|---|---|---|---|---|---|
| #1 | `dronefreak/visdrone-yolov11s` | `best.pt` → `models/visdrone-yolov11s.pt` | 19,184,538 B (18.3 MB) | AGPL-3.0 | pedestrian, people, bicycle, car, van, truck, tricycle, awning-tricycle, bus, motor, others (11) |
| #2 | `dronefreak/visdrone-yolov8x` | `best.pt` → `models/visdrone-yolov8x.pt` | 136,737,274 B (130.4 MB) | AGPL-3.0 | same 11-class list |

URLs: `https://huggingface.co/dronefreak/visdrone-yolov11s/resolve/main/best.pt`,
`https://huggingface.co/dronefreak/visdrone-yolov8x/resolve/main/best.pt`. Both loaded
directly with `ultralytics.YOLO(path)`, no custom loader, no code change. Both `curl -L`
downloads returned HTTP 200 (no GitHub-release failure — HF mirror worked as expected).

## (2) Sampling and inference

20 frames sampled evenly (`np.linspace(0, 1199, 20)`) from `out/window_intersection.mp4`
(1200 frames @ 10 fps, 3840×2160, the existing 120 s intersection window — no re-cut).
Same 20 frames run through all three detectors at `imgsz=1920, conf=0.20, device=0`.
Per-frame per-class counts: `out/exp1/counts.csv` (60 rows). Annotated side-by-side
images (COCO left, VisDrone-yolo11s right) for frames 0, 631, 1199:
`out/exp1/sidebyside_f00_0.jpg`, `_f10_631.jpg`, `_f19_1199.jpg`.

Total GPU work: 60 single-frame inferences, wall time ~19 s of actual inference (first
call per model pays ~250–830 ms one-time CUDA-context/cuDNN-autotune warmup, excluded
from the means below) — well inside the 10-minute budget.

## (3) Results

Class shares are per-frame detection-count shares (not tracked/deduped objects),
mapped to this repo's classes: COCO `person/bicycle/car/motorcycle/bus/truck` used
as-is; VisDrone `pedestrian+people→person`, `bicycle→bicycle`, `car+van→car`,
`motor→motorcycle`, `tricycle+awning-tricycle→auto-rickshaw`, `bus→bus`, `truck→truck`
(VisDrone's `others` class dropped, as is COCO's `traffic light`/`potted plant`
background noise).

| detector | person | bicycle | motorcycle | auto-rickshaw | car | bus | truck | mean detections/frame | mean ms/frame (no warmup) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| COCO yolo11x (current) | 10.0% | 0.0% | **8.3%** | 0.0% (no class) | **75.9%** | 1.9% | 3.8% | 117.7 | 267.0 |
| VisDrone-yolo11s (pick #1) | 16.9% | 0.1% | **55.9%** | 3.3% | **22.0%** | 0.4% | 1.5% | 245.8 | 94.7 |
| VisDrone-yolov8x (pick #2) | 26.6% | 0.6% | **44.7%** | 2.8% | **23.0%** | 0.2% | 2.2% | 255.2 | 242.6 |

## Verdict

**Yes — both VisDrone weights fix the confusion, decisively.** Motorcycle share rises
from 8.3% (COCO) to 55.9% (yolo11s) / 44.7% (yolov8x) — a +47.6 / +36.4 point swing —
while car share falls from 75.9% to 22.0% / 23.0%, a −53.9 / −52.9 point swing. This is
the exact predicted direction and it is not a small effect.

Looked at all three side-by-side images (`out/exp1/sidebyside_*.jpg`). In every frame,
the same tight rows of small vehicles along the roadside that COCO tiles wall-to-wall
with `car 0.7–0.95` boxes are, in the VisDrone panel, the *same boxes in the same
positions* relabeled `motor 0.4–0.9` — this is a reclassification of real objects, not
new phantom detections in empty space. The box positions and sizes line up 1:1 across
panels; VisDrone isn't inventing motorcycles in the trees or on rooftops. Two-wheelers
parked/moving in a row from directly overhead genuinely look boxy and car-sized at this
altitude — that's the exact failure mode WRITEUP.md flags, and it's visibly gone in the
VisDrone panel. VisDrone also picks up real awning-tricycle/auto-rickshaw shapes that
COCO has no class for at all and folds into `car` today. Total detections/frame is also
~2.1× higher for VisDrone (245.8/255.2 vs 117.7) — VisDrone is simply better-suited to
this dense small-object aerial scene, consistent with it being trained for it.

One caveat: these are *raw per-frame detection shares*, not the tracked/deduped
`report_intersection.json` modal split (45.1% two-wheeler / 31.4% car) — the two
numbers aren't directly comparable (no tracking, no size/motion filtering here, and a
single vehicle is counted once per frame it appears in vs. once per track in the
report). The experiment answers the qualitative question it was designed to answer
(does the classifier stop calling two-wheelers cars) — it is not a prediction of what
the new modal split would be after a full re-track.

**Recommendation: adopt pick #1 (`dronefreak/visdrone-yolov11s`).** It gives the same
class fix as pick #2, is 2.8× faster per frame (94.7 ms vs 242.6 ms — even faster than
current COCO yolo11x's 267.0 ms, despite being the newer architecture generation), and
is 7.5× smaller on disk (18.3 MB vs 130.4 MB). Pick #2 is the fallback if yolo11s's
lower-mAP/smaller-model tradeoff shows problems in a fuller test — no mAP figure is
published for pick #1, so that's a real unknown pick #2 doesn't have (yolov8x has a
published mAP50 36.81%).

**Full re-track cost estimate:** extrapolating pick #1's 94.7 ms/frame × 1200 frames =
113.6 s (**~1.9 GPU-minutes**) for the detection pass alone (pick #2: 242.6 ms × 1200 =
291.1 s, **~4.9 GPU-minutes**). Add the already-measured ~106 s (1.8 min) for the
deterministic downstream stages (`run_analysis.py`+`run_attrs.py`+`run_geo.py`+
`build_dashboard.py`, per `worker-fh-prep3`'s repro run) plus ByteTrack association
overhead (CPU-side, small). **Total estimated cost for a full re-track + rebuild with
pick #1: well under 5 GPU-minutes** — cheap enough that the next step should be running
it for real, not further scoping.

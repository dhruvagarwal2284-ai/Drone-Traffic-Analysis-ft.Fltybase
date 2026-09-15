# Drone Traffic Analysis — Baner Junction, Pune

Extracts road-user **trajectories** from drone footage and derives traffic insight from
them. No annotations, no ground-control points, no map, no fixed infrastructure.

**Dashboard:** https://claude.ai/code/artifact/aa34f729-3f04-4df9-8532-842e13016429

---

## Headline results

Intersection video, 120 s window (t = 240–360 s) at 10 fps, **731 road users**,
**204,632 trajectory samples**, detected with an aerial-pretrained (VisDrone) YOLO and a
tuned ByteTrack — see "Revision" below; the COCO-detector numbers this superseded are
archived in `out/exp4/report_intersection_COCO.json` and `NUMBERS-CHANGELOG.md`.

| | |
|---|---|
| Modal split | two-wheeler 43.8%, pedestrian 27.1%, car 25.3%, auto-rickshaw 1.8%, truck 1.2%, bus 0.5% |
| Turning movements | 169 through, 28 left, 24 right, 22 u-turn (243 traversing tracks) |
| Conflicts | 37 critical, 61 serious, 404 conflict, 1,270 minor |
| Dominant conflict pairs | two-wheeler↔two-wheeler (175), car↔two-wheeler (173), car↔car (51) |
| Worst delay | `W→SE` at 5.4 s mean stopped time (best-sampled, n=67); the raw worst movement, `NE→SE` at 51.3 s, rests on only 2 tracks and should not be quoted alone |
| Inferred signal cycle | 116 s (autocorrelation r = 0.18) |
| Anomalies | 84 (61 contraflow, 21 stopped in carriageway, 2 hard-braking) |
| Object attributes | 731 with colour, 675 with measured dimensions (92%), 88.2% achromatic fleet |
| Per-object kinematics | 731/731 in SI; 99.9% physically plausible, 1 flagged |
| Map-native | 81.8% of samples matched to OSM links; registration residual 1.79 m |
| Aggregate | max flow 1,503 veh/h/lane at 78.3 veh/km/lane; max queue 70 m; 0.95% of samples over 40 km/h |

### The three findings worth defending

1. **The telemetry is wrong and the trajectories caught it.** `rel_alt` reports 70.5 m.
   Self-calibration against car footprints demands ×1.385 → **97.6 m effective height**.
   The drone launched from a rooftop ~27 m above street level, so its height datum was
   never the road. Independent check: motorcycle length then lands at **1.85 m against
   1.99 m expected** — a quantity the calibration was never fitted to. Without this
   correction every distance, speed and time-to-collision would be substantially wrong.

2. **The junction is signal-gated; the corridor is not.** The junction's queue index
   autocorrelates at **116 s**, recovered with no controller feed and no signal head in
   view. The corridor's best peak is 36.5 s at r = 0.15 — noise. Same method, opposite
   answer: retiming helps one site and would do nothing for the other, where congestion
   comes from side friction and a construction narrowing. (Detector-free — unaffected by
   the detector change below.)

3. **Two-wheelers dominate the safety picture**, appearing in 86% of conflict-grade-or-worse
   interactions (348/404) at 43.8% of traffic. This only becomes visible once road users
   share a metric frame — and it is *more* pronounced with the class-fix detector than it
   first appeared with COCO (was ~85% at 45% of traffic).

4. **COCO's own scale self-calibration was contaminated by the exact class bug it also
   caused downstream.** `calibrate()` runs on raw pre-merge detector labels, so COCO's
   `car` bucket — and therefore its ×1.513 scale correction — was quietly full of
   two-wheelers wrongly called cars (observed footprint 2.84 m). VisDrone's clean `car`
   class observes 3.11 m directly, needing only ×1.385. Both pass the independent
   motorcycle check within ~7%, so neither run was "wrong" — but this is a second-order
   fix nobody asked for. Full detail in `EXP1.md`–`EXP3.md` and the "Revision" section
   below.

---

## Revision — aerial-pretrained detector

The numbers above are from `models/visdrone-yolov11s.pt` (VisDrone-pretrained,
AGPL-3.0) + a tuned ByteTrack (`track_buffer=90`, `match_thresh=0.9`), adopted
in place of the original COCO YOLO11x + plain ByteTrack. Three experiments,
in order:

- **`EXP1.md`** — proved COCO calls an overhead two-wheeler a car (motorcycle
  share of detections 8.3%→55.9% swapping detectors on the same 20 frames).
- **`EXP2.md`** — full before/after re-track: the class fix is real and
  decisive, but exposed the tracker as the new weak link (OSM match rate
  96.6%→82.7%, duplicate tracks 80→541).
- **`EXP3.md`** — tuned ByteTrack's buffer/match-threshold to claw back most of
  that fragmentation (duplicate tracks 541→402) without touching the class fix.

Full before/after figures for every headline number: `NUMBERS-CHANGELOG.md`.
The COCO option remains available (`config.yaml`'s `detector_coco` block,
`models/README.md`) for anyone who wants to reproduce the original run.

---

## Pipeline

```
telemetry.py     demux per-frame DJI telemetry from the mov_text subtitle track
geometry.py      analytic image -> ground-plane homography from gimbal attitude
track.py         ffmpeg window cut -> YOLO11x + ByteTrack
trajectories.py  metric projection, scale self-calibration, Kalman/RTS smoothing
conflicts.py     PET + TTC surrogate safety measures
attributes.py    measured L/W (oriented-dims solve), colour, size class, body type
objects.py       per-object record: identity + kinematics + movement + conflicts
aggregate.py     interval counts, speed field, lane discovery, queues, Edie flow/density
geo.py           georeferencing, OSM registration, map-matching, GeoJSON export
insights.py      leg discovery, turning movements, speeds, delay, anomalies
congestion.py    detector-free congestion signal over the full recording
run_analysis.py  end-to-end -> out/report_<tag>.json
overlay.py       annotated video with trajectory trails
build_dashboard.py  inject the bundle into the dashboard template
detect.py        (Step A of next-step #1) detector-only, no tracking
```

`detect.py` is the first step of next-step #1 (tracker re-architecture): it splits
track.py's fused detect+ByteTrack into a detector-only stage, so a future
metric-frame tracker can associate boxes itself. `python src/detect.py --tag
<tag> [--tiles 1|2] [--embed]` runs the configured detector per frame over an
already-cut `out/window_<tag>.mp4` (never re-cuts video) and writes
`out/detraw_<tag>.parquet` — one row per detection, no `track_id`: `fi`, `t`,
`cls`, `conf`, `x1,y1,x2,y2` (px), `u_px,v_px` (box-centre px, matching
trajectories.py's own convention), `x_m,y_m,w_m,h_m` (ground-plane metres, via
the same `calibrate()`/mid-height projection trajectories.py uses), plus an
optional 20-float HSV-histogram+size `embed` column with `--embed`. `--tiles 2`
runs an overlapping 2x2 SAHI-style tile pass merged with per-class NMS
(`torchvision.ops.batched_nms`, no new dependency). Full results: `EXP4A.md`.

Run:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics opencv-python numpy pandas pyarrow scipy

# fetch the default detector weight first (see models/README.md for the COCO
# alternative and licence details)
curl -L -o models/visdrone-yolov11s.pt "https://huggingface.co/dronefreak/visdrone-yolov11s/resolve/main/best.pt"

python src/track.py        --tag intersection --t0 240 --dur 120 --fps 10
python src/congestion.py   --video Intersection_Merged-002 --tag intersection
python src/run_analysis.py --tag intersection --t0 240 --fps 10
# run order above no longer matters: run_analysis.py computes attributes itself
# (one extra video pass) if out/attributes_<tag>.parquet doesn't exist yet.
python run_attrs.py                      # object attributes (one video pass)
python run_geo.py                        # map-native: georeference + map-match
python src/build_dashboard.py intersection
python src/overlay.py      --tag intersection --seconds 60
```

`track.py`'s defaults now point at the VisDrone weight + tuned tracker
(`trackers/bytetrack_buf90.yaml`) above. To reproduce the original COCO run
instead, pass `--weights yolo11x.pt --tracker bytetrack.yaml` explicitly (see
`config.yaml`'s `detector_coco` block).

The RTX 5050 is Blackwell (`sm_120`) and needs CUDA 12.8 wheels — the default cu121
build will not run on it. YOLO weights failed to download from GitHub (curl error 35);
the HuggingFace mirror `Ultralytics/YOLO11` works.

---

## Trajectory schema — `out/trajectories_intersection.parquet`

One row per (track, frame). This is the reusable data product; every figure derives from it.

| column | meaning |
|---|---|
| `fi`, `t` | frame index in window; absolute time in the source video (s) |
| `track_id` | persistent id |
| `cls`, `conf` | class, detector confidence |
| `x_m`, `y_m` | ground-plane position, metres, origin at the optical centre projected to road |
| `vx`, `vy`, `speed_kph`, `heading` | smoothed kinematics |
| `accel` | along-track acceleration, m/s² |
| `extent_m` | longer ground-plane footprint side |
| `u_px`, `v_px` | source pixel location (for clip export / overlay) |
| `imputed` | true where the position was interpolated through occlusion |
| `pcu` | passenger-car-unit weight |

## Object schema — `out/objects_intersection.parquet`

One row per road user (731 × 31): identity, kinematics, movement and interactions joined.

| group | columns |
|---|---|
| identity | `cls`, `body_type`, `size_class`, `colour`, `hex`, `L_m`, `W_m`, `descriptor` |
| kinematics (SI) | `v_mean_ms`, `v_max_ms`, `v85_ms`, `a_max_ms2`, `a_min_ms2`, `a_p95_ms2`, `a_p05_ms2`, `jerk_p95_ms3`, `distance_m`, `duration_s`, `stopped_s`, `n_brake_events`, `n_accel_events` |
| movement | `origin`, `dest`, `turn`, `movement`, `traversed` |
| interaction | `n_conflicts`, `worst_value_s` |
| quality | `imputed_frac`, `n_samples`, `kinematics_plausible` |

---

## Demo outputs (`demo/`)

| file | what it is |
|---|---|
| `example_output.mp4` | 60 s overlay: box by body type, label = body type + measured length + km/h, plus a chip of the vehicle's measured paint colour |
| `conflict_*.mp4` | six evidence clips cropped to the worst interactions |
| `dashboard.html` | the full dashboard, self-contained |
| `trajectories.parquet` | per-(track, frame) trajectory table |
| `attributes.parquet` | per-track colour, measured L/W, body type, size class |
| `objects.parquet` | per-object record (identity + kinematics + movement + conflicts) |
| `report.json` | every figure in the write-up, machine-readable |

## Validation

No annotations exist, so accuracy cannot be quoted against ground truth. What is checked
is physics — each of these could have failed:

- `|accel|` < 4 m/s² for **100%** of samples; **no** speeds above 100 km/h
- median car footprint = 4.0 m by construction; **motorcycle then measures 1.85 m vs
  1.99 m expected** — an independent check
- 10.7% of positions interpolated through occlusion, flagged in the data
- 402 duplicate tracks detected and removed

## Known limits

- **Conflict counts are an upper bound.** Residual ID fragmentation inflates them; the
  spatial pattern and class composition are the robust reads, not the absolute rate.
- **Only 243 of 731 tracks traversed the scene** — 67% are fragments, caused by the tree
  occluding the junction core, by image-space association, and now also by VisDrone's
  denser per-frame detection field giving the tracker more boxes to confuse (EXP3 cut
  this from 80% to 67% by tuning ByteTrack's buffer/match threshold, but did not
  eliminate it — see next steps #1).
- **OSM map-match rate dropped from 96.6% to 81.8%** when the class fix was adopted.
  This is a detection-coverage problem, not an association one (per-sample registration
  residual is *better*, 1.79 m vs 1.97 m): VisDrone's denser field puts more boxes in
  places (verges, parked two-wheelers, pavement) outside the road network's 20 m buffer.
  Tuning the tracker did not move this number (EXP3: 81.8–82.6% across four tracker
  configs) — flow/density/queue figures downstream of map-matching should be read with
  that in mind.
- **Auto-rickshaws** are now separable via the oriented-dimension solve (2.58 m vs 1.45 m),
  reversing an earlier limitation — but body type is inferred from measured size, not read
  from the vehicle, so individual calls are less certain than the aggregate.
- **Licence plates and make/model are not achievable.** An Indian plate is 15.3 × 3.7 px
  here, giving 2.0 px character height against ~16 px for OCR — an 8× shortfall — and at
  −63° pitch the plate surfaces face away from the camera entirely. Would need ~13 m
  altitude or a 193 mm-equivalent lens *and* a shallow angle.
- **Vehicle width is ~30% over-measured** (ill-conditioned half of the L/W solve).
- **Peak acceleration needs the percentile columns.** Smoother edge transients inflate the
  raw per-object maximum; use `a_p95_ms2` / `a_p05_ms2`, and the `kinematics_plausible`
  flag (1 of 731 objects fails it).
- **Pedestrian counts were a floor under COCO, and still likely are.** Adopting the
  VisDrone class fix alone (no tiling) already lifted the pedestrian count from 134 to
  198 (19.5%→27.1% share) — most of the undercounting EXP2 attributed to class confusion.
  Tiled inference was previously measured at 1.7–1.9× more road users and 5–6× more
  pedestrians than full-frame COCO; whether tiling still adds meaningfully on top of
  VisDrone hasn't been re-tested.
- **The cycle estimate is indicative**, not a measured signal plan (r = 0.18).
- **Multi_Road is deliberately not tracked.** At 18.5° gimbal pitch a motorcycle is 8 px
  across; trajectories there would be fabricated precision. It gets the detector-free
  congestion pass only.

## Next steps, in priority order

1. **Re-architect the tracker** — decouple detection from tracking (`boxmot`) so tiled
   inference is possible at all, and associate in the metric frame with appearance re-ID
   across occlusion. This is what fixes the remaining 67% fragmentation and the OSM
   match-rate drop (81.8% vs 96.6%); tuning the tracker's built-in knobs (EXP3) already
   captured the cheap gains available without doing this. *Promoted from #2 to #1*: it
   is now a prerequisite for the flow/queue/match-rate numbers, not just a fragmentation
   fix — see "Revision" below.
2. **SAHI tiling** — measured 1.7–1.9× more road users on COCO, but **6× slower**, not
   faster. VisDrone alone already recovers much of the pedestrian undercount tiling was
   aimed at (known limits, above); worth re-measuring whether tiling still earns its cost
   on top of it.

~~Aerial-pretrained weights (VisDrone/DOTA)~~ — **done**, see "Revision" below and
`EXP1.md`–`EXP3.md`.

# Drone Traffic Analysis — Baner Junction, Pune

Extracts road-user **trajectories** from drone footage and derives traffic insight from
them. No annotations, no ground-control points, no map, no fixed infrastructure.

**Dashboard:** https://claude.ai/code/artifact/aa34f729-3f04-4df9-8532-842e13016429

---

## Headline results

Intersection video, 120 s window (t = 240–360 s) at 10 fps, **602 road users**,
**253,972 trajectory samples**, detected with an aerial-pretrained (VisDrone) YOLO and
tracked with a metric-frame Kalman tracker (`detect.py` + `mtrack.py`) — see "Revision 2"
below. The ByteTrack numbers this superseded are archived in
`out/exp5/report_intersection_BYTETRACK.json` and `NUMBERS-CHANGELOG.md`; the older
COCO-detector numbers before that are in `out/exp4/report_intersection_COCO.json`.

| | |
|---|---|
| Modal split (by track) | two-wheeler 46.2%, car 29.1%, pedestrian 19.1%, auto-rickshaw 3.7%, truck 1.5%, bus 0.5% |
| Modal split (presence share, fragmentation-independent — see "Revision 2") | two-wheeler 52.9%, car 23.9%, pedestrian 17.8%, auto-rickshaw 3.4%, truck 1.6%, bus 0.3% |
| Turning movements | 186 through, 51 left, 39 right, 29 u-turn (305 traversing tracks, 50.7%) |
| Conflicts (post re-emergence filter — see "Revision 2") | 27 critical, 40 serious, 295 conflict, 1,001 minor |
| Dominant conflict pairs | two-wheeler↔two-wheeler (162), car↔two-wheeler (128), car↔car (25) |
| Worst delay | `W→W` (u-turn) at 13.3 s mean stopped (n=12); the two best-sampled through movements clear fast — `W→SE` 5.6 s (n=77), `SE→W` 2.1 s (n=109) |
| Inferred signal cycle | 116 s (autocorrelation r = 0.18) |
| Anomalies | 75 (33 contraflow, 42 stopped in carriageway) |
| Object attributes | 602 with colour, 579 with measured dimensions (96%), 89.7% achromatic fleet |
| Per-object kinematics | 602/602 in SI; 100% physically plausible, 0 flagged |
| Map-native | 82.6% of samples matched to OSM links |
| Aggregate | max flow 1,556 veh/h/lane at 75.0 veh/km/lane; max queue 68 m (leg W) |

### The three findings worth defending

1. **The telemetry is wrong and the trajectories caught it.** `rel_alt` reports 70.5 m.
   Self-calibration against car footprints demands ×1.385 → **97.6 m effective height**.
   The drone launched from a rooftop ~27 m above street level, so its height datum was
   never the road. Independent check: motorcycle length then lands at **1.85 m against
   1.99 m expected** — a quantity the calibration was never fitted to. Without this
   correction every distance, speed and time-to-collision would be substantially wrong.
   (Unchanged by the tracker re-architecture below: both trackers calibrate from the same
   shared `detraw_intersection.parquet`, ×1.3822–1.3847 either way.)

2. **The junction is signal-gated; the corridor is not.** The junction's queue index
   autocorrelates at **116 s**, recovered with no controller feed and no signal head in
   view. The corridor's best peak is 36.5 s at r = 0.15 — noise. Same method, opposite
   answer: retiming helps one site and would do nothing for the other, where congestion
   comes from side friction and a construction narrowing. (Detector-free — unaffected by
   the detector change below.)

3. **Two-wheelers dominate the safety picture**, appearing in ≈86% of conflict-grade-or-worse
   interactions (310/362 under the metric-frame tracker; was 348/404-ish under ByteTrack) at
   43.8–46.2% of traffic depending on tracker. The finding survives two independent tracker
   swaps (COCO ByteTrack → VisDrone ByteTrack → metric-frame Kalman) and a stricter conflict
   rule that strips a 4.8x re-emergence artefact (see "Revision 2") without moving this
   share — it is a property of the traffic mix, not of a specific detector, tracker, or
   conflict-counting rule.

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

## Revision 2 — metric-frame tracker

The numbers above are now from a two-stage pipeline — `src/detect.py`
(detector-only, unchanged VisDrone weights) → `src/mtrack.py` (per-track
constant-velocity Kalman filter, association in **ground-plane metres**
instead of image pixels) — replacing image-space ByteTrack as the default
tracker. `track.py` (fused detect+ByteTrack) remains available as a
documented alternative (see Pipeline below). Four experiments, in order:

- **`EXP4A.md`** — Step A: split `track.py`'s fused detect+track into
  `detect.py` (detector-only) so a future tracker can associate boxes
  itself. Also tried 2×2 SAHI-style tiling: 1.21x more detections for
  ~2.4x the compute, with bus/truck tile-seam artefacts — **closed as a
  negative result**, not adopted.
- **`EXP4.md`** — Step B: `mtrack.py`, a from-scratch metric-frame tracker
  (Kalman filter + Hungarian assignment, gated on Mahalanobis distance,
  costed on class-group distance + footprint-size). A 4-combo knob sweep
  picked `max_lost_s=6.0`. Verdict: **ADOPT** — duplicate tracks 402→110,
  traversing tracks 33.2%→52.4%, no physics regression — but flagged two
  numbers (critical conflicts ×4.8, pedestrian share 27.1%→18.2%) as
  needing independent QA before publication.
- **`EXP4-QA.md`** — independent QA of both flags. Found the conflict spike
  is 81.5% a re-emergence artefact (the RTS-smoothed velocity right where a
  track exits an occlusion-bridged gap is its least trustworthy sample, and
  that is exactly the sample PET/TTC scores) and the pedestrian drop is
  partly the same fragmentation-reduction mechanism already seen for
  two-wheelers, partly a real class-vote leak (a lone pedestrian detection
  absorbed into a nearby vehicle track under a purely-spatial gate).
  Verdict: **ADOPT WITH A CONFLICT POST-FILTER**.
- **`EXP4-FIX.md`** — implemented both of EXP4-QA's prescriptions: a
  class-group hard gate in `mtrack.py` (a detection may only match a track
  in the same group — person / bicycle / {motorcycle, autorickshaw} /
  {car, bus, truck}) and a `reemerged` trajectory column that
  `conflicts.py` excludes from PET/TTC candidacy for 2.0s after a track
  re-emerges from an imputed gap, applied identically to both trackers.

The re-emergence exclusion rule is folded into every conflict number in
this README, `WRITEUP.md`, and both trackers' outputs — it is a shared
trajectory-processing rule, not a per-tracker tweak. Full before/after
figures for every headline number: `NUMBERS-CHANGELOG.md`'s "Revision 2"
section. The ByteTrack option remains available (`track.py`, see Pipeline)
for anyone who wants to reproduce the prior run.

---

## Pipeline

```
telemetry.py     demux per-frame DJI telemetry from the mov_text subtitle track
geometry.py      analytic image -> ground-plane homography from gimbal attitude
detect.py        ffmpeg window cut -> YOLO detector only, no tracking (default, Step A)
mtrack.py        metric-frame Kalman tracker: associates detect.py's boxes (default, Step B)
track.py         fused YOLO11x + ByteTrack, image-space association (documented alternative)
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
```

**Default pipeline (since Revision 2): `detect.py` → `mtrack.py`.** `detect.py`
runs the configured VisDrone detector per frame over an already-cut
`out/window_<tag>.mp4` (never re-cuts video) and writes `out/detraw_<tag>.parquet`
— one row per detection, no `track_id`: `fi`, `t`, `cls`, `conf`, `x1,y1,x2,y2` (px),
`u_px,v_px` (box-centre px), `x_m,y_m,w_m,h_m` (ground-plane metres via the same
`calibrate()`/mid-height projection `trajectories.py` uses), plus an optional
20-float HSV-histogram+size `embed` column with `--embed`. `mtrack.py` then reads
that parquet and associates boxes into tracks itself, entirely in ground-plane
metres: a constant-velocity Kalman filter per track, Mahalanobis-gated Hungarian
assignment costed on class-group distance + footprint-size distance, with a hard
class-group gate (person / bicycle / {motorcycle, autorickshaw} / {car, bus,
truck} — a detection may only match a track in the same group) and a growing
acceptance radius that lets the gate bridge an occlusion by itself, no separate
buffer-growth schedule needed. Writes `out/detections_<tag>.parquet` in
`track.py`'s own schema, so every downstream stage runs unchanged. `--tiles 2`
on `detect.py` runs an overlapping 2×2 SAHI-style tile pass merged with per-class
NMS — tried and **not adopted** (1.21x more detections for ~2.4x the compute,
bus/truck tile-seam artefacts; `EXP4A.md`). Full tracker design and knob sweep:
`EXP4.md`, `EXP4-QA.md`, `EXP4-FIX.md`.

Run:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics opencv-python numpy pandas pyarrow scipy

# fetch the default detector weight first (see models/README.md for the COCO
# alternative and licence details)
curl -L -o models/visdrone-yolov11s.pt "https://huggingface.co/dronefreak/visdrone-yolov11s/resolve/main/best.pt"

python src/detect.py       --tag intersection --t0 240 --dur 120 --fps 10
python src/mtrack.py       --tag intersection
python src/congestion.py   --video Intersection_Merged-002 --tag intersection
python src/run_analysis.py --tag intersection --t0 240 --fps 10
# run order above no longer matters: run_analysis.py computes attributes itself
# (one extra video pass) if out/attributes_<tag>.parquet doesn't exist yet.
python run_attrs.py intersection         # object attributes (one video pass)
python run_geo.py intersection           # map-native: georeference + map-match
python src/build_dashboard.py intersection
python src/overlay.py      --tag intersection --seconds 60
```

**To reproduce the prior ByteTrack default** instead, run `track.py` in place of
`detect.py` + `mtrack.py` — it writes the same `out/detections_<tag>.parquet`
schema, so every step from `congestion.py` onward is identical either way:

```bash
python src/track.py        --tag intersection --t0 240 --dur 120 --fps 10
```

`track.py`'s own defaults still point at the VisDrone weight + tuned tracker
(`trackers/bytetrack_buf90.yaml`). To reproduce the original COCO run instead,
pass `--weights yolo11x.pt --tracker bytetrack.yaml` explicitly (see
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
| `reemerged` | true for samples within `reemerge_s` (default 2.0 s) after an imputed run ends -- the RTS-smoothed velocity is least trustworthy right at a gap edge, so `conflicts.py` excludes these the same way it excludes `imputed` rows (EXP4-FIX.md) |
| `pcu` | passenger-car-unit weight |

## Object schema — `out/objects_intersection.parquet`

One row per road user (602 × 31): identity, kinematics, movement and interactions joined.

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
- 22.3% of positions interpolated through occlusion, flagged in the data (up from 10.7%
  under ByteTrack — the metric-frame tracker bridges occlusion continuously instead of
  dying and restarting, so more rows fall inside a bridged gap; see Revision 2)
- 180 duplicate tracks detected and removed (down from 402)
- `src/mtrack.py --tag intersection` run twice independently: 0/242,679 detection rows
  differ — deterministic, no seeding needed

## Known limits

- **Conflict counts are still an approximation, now with a stated correction rule.**
  A re-emergence exclusion (Revision 2) removes the single biggest artefact (a track's
  least-trustworthy sample, right where it exits an occlusion-bridged gap), but residual
  ID fragmentation and PET/TTC's own exposure sensitivity remain — the spatial pattern
  and class composition are the robust reads, not the absolute rate.
- **305 of 602 tracks traversed the scene (50.7%)** — improved from 33.2% (previously
  from 67% fragments down to 49.3%) by the tracker re-architecture (Revision 2): metric-
  frame association with a growing Kalman gate bridges the tree's occlusion instead of
  losing the track and restarting it. Not eliminated — the class-group hard gate itself
  costs ~1.7 points back by freeing previously-absorbed detections into short-lived
  fragments rather than vehicle tracks (`EXP4-FIX.md`).
- **OSM map-match rate sits at 82.6%**, essentially unchanged from ByteTrack's 81.8%
  (was 96.6% under COCO). Confirmed across two independent trackers now that this is a
  **detection-coverage** problem, not an association one: VisDrone's denser field puts
  more boxes in places (verges, parked two-wheelers, pavement) outside the road
  network's 20 m buffer, and no tracker choice moves it by more than a point (EXP3: four
  ByteTrack configs, 81.8–82.6%; EXP4: metric-frame tracker, 81.3%; this run: 82.6%).
  Fixing this needs a road-buffer or detection-filtering change, not a tracking one —
  see next steps.
- **Pedestrian modal split is now a three-way, not two-way, comparison — and it doesn't
  resolve cleanly.** By-track share moved 27.1%→19.1% across the tracker swap; a third,
  fragmentation-independent cut (mean per-frame detection-class share, "presence share")
  gives 17.8% — *lower* than either track-based number, because a few fast-transiting
  two-wheelers generate more total detection-frames than fewer, longer-lingering
  pedestrians. Track share and presence share answer different questions (see
  `NUMBERS-CHANGELOG.md` Revision 2 for which to quote when). Separately, `EXP4-FIX.md`
  found a real, structural leak: the tracker's class-group gate stops a stray pedestrian
  detection being absorbed into a vehicle track (155/678 raw tracks affected pre-fix),
  but the freed detections are often too short-lived to confirm as their own track —
  fixing this needs better pedestrian detection recall, not a tracking-side change.
- **Auto-rickshaws** are separable via the oriented-dimension solve (2.05–2.58 m vs
  1.45 m two-wheeler) and VisDrone's own `autorickshaw` class — but body type is still
  inferred from measured size for ambiguous cases, not read from the vehicle.
- **Licence plates and make/model are not achievable.** An Indian plate is 15.3 × 3.7 px
  here, giving 2.0 px character height against ~16 px for OCR — an 8× shortfall — and at
  −63° pitch the plate surfaces face away from the camera entirely. Would need ~13 m
  altitude or a 193 mm-equivalent lens *and* a shallow angle.
- **Vehicle width is ~30% over-measured** (ill-conditioned half of the L/W solve).
- **Peak acceleration needs the percentile columns.** Smoother edge transients inflate the
  raw per-object maximum; use `a_p95_ms2` / `a_p05_ms2`, and the `kinematics_plausible`
  flag (0 of 602 objects fail it this run).
- **The cycle estimate is indicative**, not a measured signal plan (r = 0.18).
- **Multi_Road is deliberately not tracked.** At 18.5° gimbal pitch a motorcycle is 8 px
  across; trajectories there would be fabricated precision. It gets the detector-free
  congestion pass only.

## Next steps, in priority order

1. **Map-match road-buffer / detection filtering** for the ~81–83% OSM rate — the
   detection-coverage gap that two tracker re-architectures now confirm a tracker cannot
   fix. Either widen the road network's registration buffer or filter detections by
   plausible road-surface location before map-matching.
2. **Pedestrian detection recall.** The class-group gate (Revision 2) stopped the
   pedestrian-into-vehicle class leak; it can't manufacture confirmed pedestrian tracks
   out of detections too short-lived to hit `min_hits=3` on their own. Needs a
   detection-side fix, not a tracking-side one.
3. **A ReID appearance network for `mtrack.py`.** The current appearance cue is
   footprint size (`w_m,h_m`) — cheap and already earns its keep (ablated: turning it
   off raises duplicates 110→153), but cannot distinguish two similar-sized, same-class
   objects converging near-simultaneously. `detect.py --embed` already emits a
   20-float HSV+size vector as a hook for a `lambda_embed` cost term; unused so far
   because the tracker's default input lacks it (`EXP4.md`).

~~Aerial-pretrained weights (VisDrone/DOTA)~~ — **done**, see "Revision" below and
`EXP1.md`–`EXP3.md`.

~~Re-architect the tracker (decouple detection, associate in the metric frame,
re-emergence conflict filter)~~ — **done**, see "Revision 2" above and
`EXP4A.md`–`EXP4-FIX.md`.

~~SAHI tiling~~ — **closed, negative result**: 1.21× more detections for ~2.4× the
compute on top of VisDrone (vs 1.7–1.9× measured on COCO), with bus/truck tile-seam
artefacts (`EXP4A.md`). Not adopted; not planned to be revisited unless the detector
itself changes again.

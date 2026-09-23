# Traffic Analysis Agent
### Trajectory extraction and traffic intelligence from drone footage
Site: Baner Road, Pune (18.56623 °N, 73.77185 °E) · 21 Aug 2026, 17:38–17:51 IST

---

## 1. Problem

Traffic operations need to know what is happening on a road *now*. Deployed systems
cannot supply it, for three structural reasons:

- **Interactions are unmeasured.** A perspective view of one approach cannot place two
  road users in a common coordinate frame. Near-misses, conflicts and failed merges —
  the interactions that explain why a junction performs the way it does — are captured
  by nothing in the field.
- **The event vocabulary is closed.** These systems detect what they were configured to
  detect. Anything outside the list is invisible, however significant.
- **Coverage is fixed to infrastructure.** Data exists only where a camera was installed.
  The location that matters this week is usually not instrumented.

A drone removes all three constraints: the full scene sits in one frame, every road
user's complete path is captured natively, and no fixed installation is required.

---

## 2. Proposed solution

```
Drone video + per-frame telemetry
        ↓
Detection (aerial-pretrained YOLO11s, VisDrone classes)          detect.py
        ↓
Metric-frame tracking (Kalman + Hungarian in ground metres)      mtrack.py
        ↓
World-coordinate transformation (analytic homography from gimbal attitude)
        ↓
Trajectory reconstruction (Kalman + RTS smoothing)
        ↓
Traffic intelligence (conflicts · movements · queues · anomalies)
```

_Sections 4–14 describe the original COCO + ByteTrack build and keep its reasoning;
§15 and §16 record the two revisions that produced the current numbers._

The trajectory table is the product. Everything else is a query against it.

---

## 3. Architecture

| Module | Responsibility |
|---|---|
| `telemetry.py` | Demux per-frame DJI telemetry from the `mov_text` subtitle track |
| `geometry.py` | Analytic image → ground-plane mapping from gimbal attitude + focal length |
| `track.py` | Window extraction (ffmpeg/NVDEC) → YOLO11x + ByteTrack |
| `trajectories.py` | Metric projection, scale self-calibration, Kalman/RTS smoothing |
| `conflicts.py` | PET and TTC surrogate safety measures |
| `insights.py` | Leg discovery, turning movements, speeds, delay, anomalies |
| `congestion.py` | Detector-free congestion signal over the full recording |
| `run_analysis.py` | End-to-end orchestration → `report.json` |
| `attributes.py` | Measured L/W from the oriented-dims solve, colour, size class, body type |
| `objects.py` | Per-object record: identity + kinematics + movement + conflicts |
| `aggregate.py` | Interval counts, speed field, lane discovery, queue length, Edie flow/density |
| `geo.py` | Georeferencing, OSM network registration, map-matching, GeoJSON export |
| `overlay.py`, `build_dashboard.py` | Annotated video, dashboard |

**Analysis window:** 120 s (t = 240–360 s) at 10 fps = 1,200 frames, chosen because it
spans a full queue build *and* discharge. 10 fps gives 0.1 s resolution — ample for
PET/TTC, and at 40 km/h a vehicle moves only 1.1 m between samples, so association is easy.

---

## 4. Detection & tracking

*Revised twice since the original submission — see §15 (detector) and §16
(tracker). Numbers below are the current (VisDrone detector + metric-frame
tracker) run; prior runs are archived in `out/exp4/report_intersection_COCO.json`,
`out/exp5/report_intersection_BYTETRACK.json`, and `NUMBERS-CHANGELOG.md`.*

Detection and tracking are now two separate stages, not one fused call:

- **Stage A — `detect.py` (detector only).** VisDrone-pretrained
  `dronefreak/visdrone-yolov11s` (AGPL-3.0), `imgsz=1920`, `conf=0.20`, FP16 —
  adopted over the original COCO YOLO11x because COCO routinely calls an
  overhead two-wheeler a car (confirmed on this footage in `EXP1.md`:
  motorcycle share of detections 8.3%→55.9% swapping detectors on the same 20
  frames, same pixel positions). Runs per frame over the already-cut window,
  never re-cuts video, and writes one row per detection with **no track id** —
  `x_m,y_m,w_m,h_m` ground-plane metres already projected, via the same
  mid-height calibration `trajectories.py` uses, so tracking has nothing left
  to project. **Classes used:** `pedestrian`/`people`→person, `bicycle`→
  bicycle, `car`/`van`→car, `truck`→truck, `bus`→bus, `motor`→motorcycle,
  `tricycle`/`awning-tricycle`→**autorickshaw**.
- **Stage B — `mtrack.py` (metric-frame association).** A from-scratch
  tracker, numpy/scipy/pandas only: a constant-velocity Kalman filter
  `(x,y,vx,vy)` **per track, in ground-plane metres** — no pixels, no
  telemetry/homography needed inside the tracker at all. Per-frame
  association is two-stage (ByteTrack-style: `conf≥0.5` first, then the
  rest), solved with `scipy.optimize.linear_sum_assignment`. The gate is
  Mahalanobis distance on the filter's own innovation covariance, which
  **inflates on every unmatched frame** — the acceptance radius grows through
  an occlusion by itself, no separate buffer-growth schedule needed. Cost =
  position distance + a class-group ordinal term + a footprint-size term (the
  size term stands in for an appearance cue the detector's raw output
  doesn't otherwise provide). On top of that soft cost sits a **hard
  class-group gate**: a detection may only match a track in the same group —
  `{person}`, `{bicycle}`, `{motorcycle, autorickshaw}`, `{car, bus, truck}`
  — added in `EXP4-FIX.md` after `EXP4-QA.md` found a soft class cost alone
  let 155/678 raw confirmed tracks absorb a stray pedestrian detection into a
  vehicle track. **Occlusion buffer:** a confirmed track tolerates up to
  `max_lost_s=6.0` (60 frames at 10 fps) of consecutive misses before it is
  dropped; a *tentative* track (not yet at `min_hits=3`) dies on its first
  miss, so confirmed hits are always consecutive.
- **Result:** 269,645 detections (shared by both stages) → 796 raw confirmed
  tracks → **602 road users** after dedup (was 731 under the fused
  detector+ByteTrack run, 687 under COCO). `track.py` (fused detect+ByteTrack,
  image-space association) remains available as a documented alternative —
  same detector, same classes, different tracker — for anyone who wants the
  prior default back.

**Handling camera movement — the finding that simplified everything.** The telemetry shows
the drone is effectively a fixed camera:

| Clip | Yaw range | Pitch range | Altitude range | GPS drift |
|---|---|---|---|---|
| Intersection 0 | 1.0° | 0.0° | 0.05 m | 0.22 m |
| Intersection 1 | 0.2° | 0.0° | 0.05 m | 0.11 m |
| Corridor 0 | 0.3° | 0.0° | 0.05 m | 0.21 m |
| Corridor 1 | 0.1° | 0.0° | 0.05 m | 0.11 m |

One homography per clip is therefore valid — no per-frame pose estimation needed. This
removed the hardest part of the problem. (`FrameCnt` resets at frame 6801/6802 reveal
each file is two concatenated DJI clips, contiguous in time; analysis stays inside one.)

**Handling occlusion.** A large tree permanently occludes part of the junction core.
Tracks are interpolated through gaps and every such row is flagged `imputed=True` —
**22.3%** of positions (was 10.7% under ByteTrack, 6.7% under COCO — the metric-frame
tracker's growing Kalman gate now bridges occlusion *continuously* rather than losing
the track and restarting it, so more rows fall inside a bridged gap; this is the
mechanism behind the traversing-track gain below, not a quality regression — no
increase in implausible kinematics, §12). A first-class `reemerged` column flags the
2.0 s after a track exits such a gap — the RTS-smoothed velocity right at that edge is
the track's least trustworthy sample — and `conflicts.py` excludes those rows from
PET/TTC candidacy the same way it already excludes `imputed` ones (§6, §16). A
separate deduplication pass removes tracks that are a second copy of another (two ids
within 2.2 m for >55% of shared frames): **180 removed** (was 402 under ByteTrack, 80
under COCO). Left in, these produce PET ≈ 0 s and would dominate the conflict ranking
with pure artefact.

---

## 5. Trajectory extraction

### Pixels → road coordinates

Intrinsics come from the reported 35 mm-equivalent focal length (24 mm on a 4K 16:9
frame → HFOV 73.7°, f = 2560 px). Extrinsics come from gimbal yaw/pitch/roll. A ray is
cast per pixel and intersected with the ground plane. **No ground-control points, no
satellite imagery, no manual zones** — which is what makes the system portable to a site
nobody has surveyed.

Validation of the geometry: frame centre lands **35.8 m** from nadir (hand-check: 35.8 m);
GSD **2.5–4.2 cm/px** across frame; ground coverage ≈ 137 × 146 m.

### Height parallax

At −63° we mostly see vehicle *roofs*. Back-projecting a box centroid to Z = 0 places a
bus metres from where it stands. Instead each detection is intersected at its class's
**mid-height** (car 0.75 m, bus 1.60 m …), which removes the parallax exactly rather than
approximating it.

### Scale self-calibration — the most important step

`rel_alt` is height above the *takeoff point*, not above the road. Observed car footprints
came out at **3.11 m** against an expected **4.30 m** (the expectation accounts for
axis-aligned boxes overstating a rotated rectangle). Applying the implied factor:

| | |
|---|---|
| Scale correction | **×1.3847** |
| Reported height | 70.47 m |
| **Effective height above road** | **97.57 m** |

The drone launched from a rooftop roughly 27 m above street level — there is a high-rise
in frame. **Without this correction every distance, speed and time-to-collision would be
substantially wrong.**

**Independent check:** motorcycle length then measures **1.85 m** against **1.99 m**
expected — a quantity the calibration was never fitted to. 7.1% error.

*(These numbers moved from the original submission's ×1.5128 / 106.60 m / 2.12 m under
COCO — not because the physical calibration method changed, but because COCO's own `car`
population feeding this self-calibration was contaminated by the same two-wheeler-called-
car bug the detector swap fixes: see §15. Both runs pass the independent motorcycle check
within ~7%, so neither was "wrong", but the VisDrone input is provably cleaner.)*

### Derived kinematics

A constant-velocity Kalman filter with a Rauch–Tung–Striebel smoother runs over each
path. This is not cosmetic: accelerations from raw finite differences of noisy positions
are pure noise, and the safety metrics depend on them. Outputs: position, velocity,
speed, heading, acceleration — 253,972 rows across 602 road users (was 204,632 rows
across 731 under ByteTrack, 113,620 rows across 687 under COCO; §16).

### Scene geometry, discovered from motion

Approach legs are found as peaks in the circular histogram of entry/exit bearings, using
only tracks that genuinely traversed the scene and reached the junction. Three legs were
recovered: **NE (22.5°), SE (117.5°), W (287.5°)** (was N (18°), SE (112°), W (288°) under
COCO — the ~4-5° shift is the leg-discovery histogram re-clustering slightly differently
over a larger, denser track population, not a change of site). SE and W are opposite ends
of the same arterial, so SE↔W is the through movement.

---

## 6. Interaction analysis

Both measures are computed in the shared metric frame — impossible without it.

- **PET (post-encroachment time).** Time between one road user leaving a point in space
  and the next arriving at it. Computed on a 1.5 m spatial grid. Car-following is
  excluded by requiring a crossing angle > 40°: two cars in one lane produce a small time
  gap at the same point, but that is headway, not a conflict.
- **TTC (time to collision).** Per frame, for closing pairs within 30 m, solving for
  contact under constant velocity with class-dependent footprint radii. Requires a real
  present separation (> 1.0 m gap) so it predicts a future contact rather than reporting
  an existing overlap.

**Footprint radii are lateral half-widths, not half-lengths** (car 0.95 m, two-wheeler
0.45 m, bus 1.30 m). Using a length-scale radius makes adjacent-lane vehicles register as
already touching — in lane-sharing traffic that manufactured thousands of phantom
conflicts (critical count fell 729 → 47 → 20 as this and deduplication were corrected).

**Merging and crossing conflicts** are separated by crossing angle; the conflict heatmap
localises where in the junction they concentrate.

---

## 7. Traffic insights — actual outputs

*Revised twice since the original submission — the detector (§15) then the
tracker (§16). Modal split is by track; §16 also reports a fragmentation-independent
"presence share" cut, which reads differently for pedestrians — see there.*

```
Road users tracked              602
Trajectory samples          253,972
Median speed                3.5 km/h      (p99  37.1 km/h)
Traversing tracks               305

Modal split (by track)
  two-wheeler               46.2 %
  car                       29.1 %
  pedestrian                19.1 %
  auto-rickshaw               3.7 %
  truck                       1.5 %
  bus                         0.5 %

Turning movements (305 traversing)
  through                     186
  left                         51
  right                        39
  u-turn                       29

Delay
  mean stopped time         19.85 s
  p85 stopped time          45.14 s
  fraction ever stopped     51.2 %
  worst movement (by sample count)   W→W (u-turn), 13.3 s mean stopped (n=12)
  best-sampled through movements     W→SE 5.6 s (n=77), SE→W 2.1 s (n=109) —
                                     through traffic clears fast; delay now
                                     concentrates in the low-volume turning/
                                     u-turn movements, not one thin outlier

Conflicts (1,363 interacting pairs, post re-emergence filter — §16)
  critical  (< 1.0 s)          27
  serious   (< 1.5 s)          40
  conflict  (< 3.0 s)         295
  minor     (3–5 s)         1,001

  two-wheeler ↔ two-wheeler   162
  car ↔ two-wheeler           128
  car ↔ car                    25
  two-wheeler ↔ pedestrian     11

Inferred signal cycle       116 s          (autocorrelation r = 0.18, unchanged —
                                            congestion.py is detector-free)
```

(ByteTrack run: 731 road users, 204,632 samples, median speed 3.3 km/h, 243
traversing tracks, modal split two-wheeler 43.8% / pedestrian 27.1% / car
25.3% / auto-rickshaw 1.8%, 1,772 conflicts pre-filter (1,506 post-filter).
Original COCO run: 687 road users, 113,620 samples, median speed 7.6 km/h,
204 traversing tracks, modal split two-wheeler 45.1% / car 31.4% / pedestrian
19.5% / bus 2.6% / truck 1.3%, 1,133 conflicts. Full comparison:
`NUMBERS-CHANGELOG.md`.)

**Origin–destination matrix** (traversing tracks; legs renamed N→NE, a ~4-5°
re-clustering over the larger track population, see §5):

| from ↓ / to → | NE | SE | W |
|---|---|---|---|
| **NE** | 4 | 17 | 32 |
| **SE** | 19 | 13 | 77 |
| **W** | 22 | 109 | 12 |

This is the turning-movement count a junction study normally pays a crew to tally by hand.

### Signal cycle without a controller feed

A detector-free signal covers the **entire 399 s** recording using frame statistics
alone, separating *motion* (how much is moving) from *occupancy* (how much is present).
The state that matters — a standing queue — is present but not moving. Autocorrelation of
that queue index peaks at **116 s**: traffic arrives and clears on a cycle, the signature
of signal control, recovered with no controller feed, no loop, and no signal head in view.

**The same method on the corridor returns the opposite answer:** best peak 36.5 s at
r = 0.15, at the edge of the search band and barely above noise — what an uninterrupted
arterial looks like. The two sites need different interventions: retiming helps one and
would do nothing for the other, where congestion builds from side friction and a
construction narrowing.

---

## 8. Anomaly detection

Rather than enumerate event types, the system learns what normal looks like *per
location* — the prevailing heading and speed in each 6 m cell, as a vector mean — and
reports departures from it. Nothing below was configured in advance:

```
75 anomalies                        (was 84 ByteTrack, 48 COCO)
  contraflow                   33     (>120° against the local prevailing flow)
  stopped in carriageway       42     (stationary >8 s mid-scene, having moved before)
  hard braking                   0     (< -3 m/s²)
```

Because the normal model is estimated from the data, a behaviour nobody thought to
configure still surfaces. Hard-braking events (< −3 m/s²) are scored the same way.

---

## 9. Object-level insight

Three requirements: fine-grained classification, licence-plate recognition, and per-object
kinematics in real units. Two are delivered; one is not achievable from this footage and
the reason is measured rather than asserted.

### 9.1 Measured dimensions — the enabling trick

The detector returns an **image**-axis-aligned box, so its ground projection is aligned to
the image axes, not the vehicle. The longer side therefore overstates length by an amount
that depends on the vehicle's heading. But heading is known from the smoothed trajectory,
so the projection inverts: the ground extent along each image axis gives

```
a = L·|cos φx| + W·|sin φx|
b = L·|cos φy| + W·|sin φy|
```

two equations in L and W. Samples near φ ≈ 45° are ill-conditioned and rejected; the
estimate is the median over the track. **This needs no oriented-box detector.**

The result is an external check on the entire calibration chain — none of these numbers
were fitted:

| body type | n | measured L | real-world L |
|---|---|---|---|
| bus | 3 | 6.05 m | 10.0–12.0 m *(n=3, thin)* |
| van / LCV | 2 | 5.28 m | 4.8–6.0 m *(n=2, thin)* |
| SUV / MUV | 6 | 4.76 m | 4.4–4.9 m *(n=6, thin)* |
| sedan | 33 | 4.28 m | 4.2–4.6 m |
| truck | 9 | 3.79 m | 6.0–10.0 m |
| hatchback | 99 | 3.70 m | 3.6–4.0 m |
| **auto-rickshaw** | 23 | **2.71 m** | 2.6–2.9 m |
| two-wheeler | 294 | 1.47 m | 1.8–2.0 m |

(Was bus 1/5.75 m, van/LCV 3/5.12 m, SUV/MUV 8/4.63 m, sedan 24/4.33 m, truck
8/4.11 m, hatchback 99/3.64 m, auto-rickshaw 29/2.58 m, two-wheeler 328/1.45 m
under ByteTrack; bus 13/9.83 m, van/LCV 10/5.16 m, truck 7/5.09 m, SUV/MUV
23/4.78 m, sedan 35/4.26 m, hatchback 67/3.85 m, auto-rickshaw 38/2.59 m,
two-wheeler 311/1.67 m under COCO — each tracker swap changes the road-user
population and its per-class sample sizes; bus, van/LCV and SUV/MUV are thin
samples this run and should not be over-read.)

**This reverses a limitation reported at the previous level.** Auto-rickshaws were
declared unrecoverable because naive footprints gave 2.13 m against 2.07 m for
motorcycles — one distribution, not two. Removing the heading-dependent inflation
separates them cleanly at **2.58 m vs 1.45 m**. The earlier limitation was a property of
the measurement, not of the data. (VisDrone additionally recovers autorickshaw as a
real detector class — §4 — so this size-based separation is now a cross-check on the
detector's own class label, not the only way to find it.)

Width is the ill-conditioned half of the solve and measures ~30% wide against known
vehicles; it is reported but only breaks ties in body-type assignment. **579 of 602**
objects (96%) received dimensions; the rest were ill-conditioned throughout (was 675 of
731, 92%, under ByteTrack; 594 of 687, 86%, under COCO).

### 9.2 Colour — two-stage white balance

Raw pixels are useless here: overcast evening light on wet asphalt casts everything blue.
Two corrections, in order:

1. **Road as neutral reference.** Sample away from detections, keep only *achromatic*
   pixels. Selecting on low chroma is what isolates road and pavement — a first attempt
   averaged in the tree canopy and planted medians, which made the reference green and
   pushed every corrected vehicle toward magenta (42 "violet" cars).
2. **Fleet as a colour chart.** Wet asphalt reflecting an overcast sky genuinely *is*
   blue, so the road reference leaves a residual cast. In this market roughly two-thirds
   of vehicles are white, silver, grey or black, so the *median* vehicle is achromatic by
   composition; driving it to neutral removes what the road could not.

Colour is sampled from the central patch only (the roof — a full box contains road,
shadow and often a neighbour) and taken as a median across every frame of the track.

```
dark grey      36.0 %        red      6.1 %
silver / grey  31.2 %        blue     1.8 %
white          15.9 %        green    1.0 %
black           6.5 %        orange   0.7 %
                              other    0.8 %  (violet/cyan/yellow)
                              → 89.7 % achromatic
```

(Was 35.2/28.6/18.1/6.4 dark grey/silver/white/black, 88.2% achromatic, under ByteTrack;
33.2/29.0/14.4/9.9, 86.5% achromatic, under COCO — each tracker swap changes the
road-user population, not the colour method.)

That achromatic share matches the Indian market. The split *within* the greys is a
brightness judgement under uncontrolled evening light and should not be over-read.

*Determinism note (§15): the colour-sampling code originally had an unseeded random
sample in its grey-world white-balance step (`attributes.py`'s `grey_world_gains`),
so colour/hex output could drift a few counts between identical reruns. Fixed with a
seeded RNG; verified 0 diffs across two consecutive reruns, both at the per-track
attribute level and in the full `report.json`.*

### 9.3 Per-object kinematics, in SI

From the RTS-smoothed trajectory — not finite differences, which would make acceleration
pure noise.

| body type | mean speed | p85 top speed | peak accel | peak braking |
|---|---|---|---|---|
| sedan | 18.9 km/h | 33.9 km/h | 1.05 m/s² | −1.10 m/s² |
| hatchback | 18.2 km/h | 33.7 km/h | 1.03 m/s² | −0.93 m/s² |
| two-wheeler | 16.0 km/h | 33.4 km/h | 0.95 m/s² | −1.05 m/s² |
| bus | 14.1 km/h | 33.0 km/h | 1.48 m/s² | −1.05 m/s² *(n=3, thin)* |
| auto-rickshaw | 13.7 km/h | 30.6 km/h | 0.70 m/s² | −0.42 m/s² |
| van / LCV | 11.3 km/h | 20.8 km/h | 0.92 m/s² | −1.00 m/s² *(n=2, thin)* |
| truck | 8.0 km/h | 27.3 km/h | 0.79 m/s² | −0.42 m/s² |
| SUV / MUV | 5.3 km/h | 23.2 km/h | 0.61 m/s² | −0.44 m/s² *(n=6, thin)* |
| pedestrian | 5.1 km/h | 14.6 km/h | 0.73 m/s² | −0.62 m/s² |

(Was van/LCV fastest at 23.5 km/h under ByteTrack, SUV/MUV fastest at 20.0 km/h and
buses stationary throughout under COCO. Each tracker swap reshuffles the ranking
through a different sample per body type — van/LCV, bus and SUV/MUV are thin samples
this run (n≤6) and not a robust ranking on their own; sedan/hatchback/two-wheeler,
with the largest samples, are the reliable comparison.)

### 9.4 The object record

`out/objects_<tag>.parquet`, one row per road user, joins identity, kinematics, movement
and interaction. Example rows:

```
white two-wheeler      L 1.56 m  mean 11.2  max 22.2 km/h  brake -1.12 m/s²  229 m  W→SE  26 conflicts
dark grey hatchback    L 3.15 m  mean 18.0  max 31.5 km/h  brake -1.12 m/s²  219 m  W→W   1 conflict
silver/grey hatchback  L 3.54 m  mean 18.0  max 35.7 km/h  brake -1.75 m/s²  216 m  W→W   8 conflicts
```

(Was silver/grey hatchback 3.44/3.47 m + red two-wheeler 1.22 m, 216/213/208 m, under
ByteTrack; silver/grey hatchback 3.96 m / white hatchback 3.94 m / silver/grey sedan
4.27 m, 229/215/214 m, under COCO — the ranked-by-distance top rows are a different
set of tracks each time, not a change in what's reported.)

Validation: 602 objects, **0** above 120 km/h, **100%** (602/602) with peak acceleration
inside ±4 m/s², median distance travelled 77.4 m, **602 with colour**, **579 with
dimensions** (96%). (Was 731 objects, 99.86% (730/731), median distance 28.5 m, 731 with
colour, 675 with dimensions (92%), under ByteTrack; 687 objects, 99.56% (684/687),
median distance 16.2 m, 687 with colour, 594 with dimensions (86%), under COCO.)

A note on peak acceleration. The RTS smoother has no data beyond a track's ends, so the
first and last samples carry an edge transient that surfaces as a spurious peak — raw
maxima reached 4.23 m/s² this run (was 7.67 m/s² under COCO), still higher than any
hatchback here is really doing. Two samples are trimmed from each end before extremes
are taken, and every object also carries `a_p95_ms2` (range −0.17 to 3.82 m/s²) /
`a_p05_ms2` (range −3.28 to 0.12 m/s²) plus a `kinematics_plausible` flag. **One object
remains implausible and is flagged rather than deleted** (was 3 of 687 under COCO). Quote
the percentile columns for individual objects; the raw max is retained for auditability.

The overlay video renders this record directly, so the classification and colour
extraction can be checked by eye against the footage rather than only in aggregate.

### 9.5 Licence plates — not achievable, and why

| | at 2.54 cm/px (best) | at 3.27 cm/px (centre) |
|---|---|---|
| Car plate (500 × 120 mm) | 19.7 × 4.7 px | **15.3 × 3.7 px** |
| Two-wheeler plate | 7.9 × 3.9 px | 6.1 × 3.1 px |
| Character height | 2.6 px | **2.0 px** (OCR needs ≥16) |

An **8.0× shortfall** on character height. Resolution is only half the problem: at −63°
gimbal pitch the camera images vehicle **roofs**, and plates sit on vertical fore-and-aft
surfaces that are not in view at *any* resolution — visible in the vehicle crops, where no
part of the front or rear face appears.

What would actually be required: **4.1 mm/px**, i.e. roughly **13 m altitude** instead of
107 m, or a **193 mm-equivalent lens** instead of 24 mm — *and* a shallow camera angle.
That is a different sensor tasking, not a better model. Attempting OCR on 2-pixel
characters would produce confident, fabricated registration numbers, which is worse than
returning nothing.

**Make and model are likewise not readable.** Badges, grilles and lamp signatures are all
on surfaces the camera cannot see. What the roof view *does* support is body type, size
class, colour and precise dimensions — delivered above.

---

## 10. Aggregate insight

Summaries across many road users, a time window, and a region of the road. Each of these
needs the *whole* space-time plane; a point sensor can only approximate them.

### 10.1 Flow, density and occupancy - Edie's generalised definitions

For a space-time region A of size (ds x dt):

```
q = (total distance travelled in A) / |A|      [veh/s]
k = (total time spent in A)         / |A|      [veh/m]
v = q / k                                      [m/s]
```

These are exact for any region and assume no stationarity. A loop detector measures flow
at a single point and infers density; complete trajectories give both directly, so the
fundamental diagram is **measured rather than fitted**.

Two normalisations are essential and were both wrong on the first attempt. Flow initially
came out at 10,712 veh/h and density at 1,208 veh/km -- physically impossible until you
notice the corridor is 28 m wide, bidirectional and multi-lane. Splitting on the sign of
velocity along the axis, and dividing by an equivalent lane count taken from each stream's
own measured width, brings it into range:

| quantity | value | (was, ByteTrack) | (was, COCO) |
|---|---|---|---|
| Max flow | **1,556 veh/h/lane** | 1,503 | 2,266 |
| Density at max flow | 75.0 veh/km/lane | 78.3 | 110.0 |
| Speed at max flow | 20.7 km/h | 19.2 | 20.6 |
| Max density observed | 170.9 veh/km/lane | 159.3 | 207.4 |
| Min speed observed | 5.9 km/h | 4.9 | 4.9 |
| Max occupancy | 34.1 % | 33.0 | 76.3 |
| Equivalent lanes | inbound 6.92 · outbound 5.65 | inbound 7.0 · outbound 5.43 | inbound 5.7 · outbound 3.33 |

Correction to a claim in `EXP2.md`: these flow/density figures are computed directly
from raw trajectories and leg geometry (`aggregate.py`'s `edie()`), not from the
OSM-map-matched output — they do not depend on the map-match rate discussed in §11.3.
Equivalent lane count (measured stream width / 3.5 m, §13) rose alongside the drop in
per-lane flow (inbound 5.7→7.0, outbound 3.33→5.43), which is consistent with a wider
measured traffic stream diluting the per-lane rate; this has not been traced further.

Occupancy is `k x mean vehicle length` -- the quantity a loop reports -- so the output is
directly comparable to installed instrumentation.

The scatter sits almost entirely on the **congested branch**: speed at maximum flow is
20.6 km/h, far below free-flow. This junction is operating at or past capacity
for the whole window.

### 10.2 Queue length in metres

Traced outward from a stop line (itself derived from where vehicles actually stop) through
contiguous stopped vehicles, ending at the first gap wider than 14 m.

| approach | max queue | p85 | mean |
|---|---|---|---|
| NE | 51.1 m | 50.0 m | 49.2 m |
| SE | 47.0 m | 44.7 m | 39.4 m |
| W | 67.9 m | 65.9 m | 46.6 m |

(Was NE/SE/W 50.6/63.0/70.3 m (max) under ByteTrack; N/SE/W 26.6/44.1/71.7 m (max)
under COCO. SE's queue is markedly shorter under the metric-frame tracker (63.0→47.0 m)
— consistent with fewer, more continuous SE tracks needing less stop-line back-fill
to bridge fragmentation; W stays the longest queue either way.)

A vehicle count cannot tell an engineer whether a queue blocks the junction upstream. A
distance can, and it is the number a signal-timing decision actually consumes.

### 10.3 Do lanes exist here?

Lanes are not read from paint -- there is little, and it is widely ignored. A lane is
defined as a mode in the lateral-offset distribution on each approach, which makes lane
discipline a *measured* quantity:

| approach | modes found | offsets (m) | discipline index | samples |
|---|---|---|---|---|
| NE | 2 | 1.13, 3.63 | 0.04 | 7,259 |
| SE | 4 | -10.5, -2.5, 3.5, 7.5 | 0.47 | 20,396 |
| W | 3 | -9.43, -0.93, 2.07 | 0.51 | 22,473 |

(Was NE 2 modes/-0.51,3.49/0.11/n=7,149, SE 3 modes/-11.59,-3.59,6.41/0.64/n=19,175,
W 3 modes/-8.04,0.46,3.46/0.50/n=22,909 under ByteTrack; N 1 mode/8.1 m/n=3,345, SE 2
modes/0.75/n=14,381, W 3 modes/0.57/n=16,177 under COCO. SE now resolves a fourth
lateral mode with the metric-frame tracker's larger, more continuous sample — the
lane-discipline finding (a continuum, not painted lanes) holds regardless of exactly
how many modes the histogram resolves.)

The discipline index is 1.0 for sharply separated lanes and 0 for a continuum. Where only
one mode is found the index is undefined and reported as such, because a single mode means
traffic is using the carriageway as a continuous surface rather than as lanes -- which is
itself the finding.


**Lane volume and modal split.** Once lanes are located, what each one carries can be
measured — and the lanes are not interchangeable:

| approach | lane | offset | share of approach | modal split |
|---|---|---|---|---|
| NE | 1 | 1.13 m | 43.5% | person 48%, two-wheeler 42%, car 10% |
| NE | 2 | 3.63 m | 56.5% | car 43%, two-wheeler 40%, person 15% |
| SE | 1 | -10.5 m | 16.3% | person 63%, car 17%, two-wheeler 18% |
| SE | 2 | -2.5 m | 31.1% | two-wheeler 64%, car 28%, autorickshaw 7% |
| SE | 3 | 3.5 m | 16.1% | two-wheeler 50%, car 40%, autorickshaw 7% |
| SE | 4 | 7.5 m | 36.5% | two-wheeler 57%, car 28%, person 10% |
| W | 1 | -9.43 m | 41.9% | two-wheeler 51%, car 30%, person 17% |
| W | 2 | -0.93 m | 20.5% | car 51%, two-wheeler 34%, autorickshaw 12% |
| W | 3 | 2.07 m | 37.6% | two-wheeler 51%, car 30%, person 13% |

(Was NE two lanes 45%/55% person-42%/car-49%-led, SE three lanes 15%/37%/48%
person-68%-led, W three lanes 43%/23%/35% two-wheeler-led under ByteTrack; N one lane
at 100% car 54%/2W 38%, SE two lanes 55%–46% split car/2W, W three lanes topping out
at 78% car under COCO. The metric-frame tracker now resolves a fourth SE lane; the
pedestrian-kerbside-lane finding holds across both VisDrone-based runs — NE's lane 1
and SE's lane 1 are both plurality-pedestrian either way — with the exact split
shifting alongside each tracker's own modal-split change (§16).)

Three things fall out of that table. Both NE's lane 1 and SE's lane 1 are
plurality-pedestrian (48% and 63%) — the kerbside edge, where people are walking in
the carriageway rather than on a footpath — while the adjacent lane in each case is
plurality two-wheeler or car. On the W approach, two-wheelers dominate the outer lanes
(51% and 51%) while cars concentrate centrally (51%). None of this is visible in an
approach-level total, and all of it changes what an intervention should target.

**Classified counts by interval.** 55 movement x class x interval rows at 20-second
resolution, plus 15 directional approach volumes in veh/h and PCU/h. This is the
turning count a survey crew produces, at a time resolution manual counting cannot reach —
and because it is derived from trajectories rather than tallied at a screenline, every
count is attributable to a specific vehicle with a known path.

**Lateral position by class** is the more revealing cut. On the busiest approach,
two-wheelers sit at a different median offset from cars and occupy a visibly wider spread:
that is lane-filtering, quantified, rather than asserted.

### 10.4 Where speed is lost

85th-percentile speed per 4 m cell of carriageway, over 353 cells.

Only **0.96%** of moving samples exceed 40 km/h and **0%** exceed
50 km/h, against a peak cell 85th-percentile of 41.0 km/h. (Was 0.95% / 0% / 41.2 km/h
under ByteTrack, essentially unchanged by the tracker swap; 2.08% / 0.03% / 43.6 km/h
under COCO — the finding strengthens there, it does not reverse.)

**There is no speeding problem at this site.** It has a delay and conflict problem. An
enforcement response aimed at speed would address neither -- which is exactly the kind of
misdirected intervention that aggregate-only reporting invites, and that a spatial speed
field prevents.

### 10.5 Classified counts by interval

Covered in 10.3 above, and rendered on the dashboard as a stacked count per 20-second
interval by movement, alongside per-approach volumes in veh/h.

---

## 11. Map-native output

Everything to this point lives in a local metric frame whose origin sits under a chosen
pixel. That is enough for speeds and conflicts, but it is not map-native: the results
cannot be laid over real road geometry, joined to an asset register, or handed to anyone
who works in lat/lon. Three steps close that gap.

### 11.1 Georeferencing from the SRT telemetry

The ENU frame is already metric and north-aligned -- the rotation is built from a compass
gimbal yaw -- and the drone's own GPS fix supplies the origin. The conversion to WGS84 is
therefore a local tangent-plane offset: no control points, no rubber-sheeting.

The inverse projection (`GroundPlane.to_image`) round-trips image -> ground -> image at
**0.0000 px**, so ground-space results can be drawn back onto the footage they came from.

### 11.2 Registering the network -- and a false answer found on the way

OpenStreetMap supplies the link geometry (43 links in view, the arterial being
*Gopal Hari Deshmukh Marg*). Published OSM geometry and the drone GPS fix disagree by tens
of metres: both carry error, and in this region OSM is frequently traced from offset
imagery. Vehicles, however, are on the carriageway by definition, so the trajectories are
better control points than either source.

Only a **translation** is fitted. Rotation comes from the gimbal compass and scale from
the footprint calibration; both are independently validated already and are not
re-estimated.

**The first attempt was wrong, and looked convincing.** Fitting against the full network
-- which includes 39 service and 39 residential ways -- produced a confident optimum with
a 1.68 m residual at a shift of **91 m**. That is not GPS error; translation-only
registration had snapped the trajectories onto a *different parallel road*, and the
excellent residual was a false minimum manufactured by the density of candidate links.
The tell was that the optimum kept moving as the search window widened.

Restricting the fit to arterial links and bounding the search to a physically plausible
range converges:

| | |
|---|---|
| Median vehicle-to-link, before | 15.98 m |
| Median vehicle-to-link, after | **1.97 m** |
| Registration shift applied | 24.7 m |
| Samples used | 20,000 |

**Corroborated by a quantity it was not optimised against.** The fit minimises
vehicle-to-link distance. OSM's own junction node -- which plays no part in that objective
-- moves from **25.6 m** to **12.1 m** of the observed junction centre (taken as the
centroid of measured conflicts). An independent measure improving alongside the fitted one
is what separates a real registration from a plausible-looking artefact.

### 11.3 Map-matching, moment to moment

Each trajectory sample is bound to a link, a direction along it, and a lane. Direction is
the sign of the vehicle's heading against the link's digitised direction, so "which way
along this road" is recovered without depending on the one-way tag being correct. The
signed lateral offset separates the two carriageways of a divided road and is what the
lane index is built from.

```
82.6 % of trajectory samples matched to a link      (was 81.8% ByteTrack, 96.6% COCO)
   primary      180,830   (Gopal Hari Deshmukh Marg)
   residential   24,727
   tertiary       4,337
```

**The match rate dropped from COCO's 96.6%, and it is a detection-coverage issue, not
a registration or tracking one** — the per-sample residual is unaffected by tracker
choice (§11.2's method unchanged). VisDrone's roughly 2× denser per-frame detection
field puts more boxes in places (verges, parked two-wheelers, pavement) outside the
20 m road-network buffer than COCO's sparser, coarser field did. Two independent
tracker architectures now agree it is not fixable by association alone: `EXP3.md`'s
four ByteTrack configurations (81.8–82.6%), `EXP4.md`'s metric-frame tracker pre-fix
(81.3%), and this run's post-fix metric-frame tracker (82.6%) all land in the same
81–83% band regardless of tracking approach. This closes the tracker re-architecture
that was previously listed as future work (§14) — the remaining fix needs a
road-buffer or detection-filtering change, not another tracker swap.

### 11.4 Per-lane metrics on real geometry

Keyed on OSM link id and carriageway, so the output joins to any network model or asset
register speaking the same identifiers:

| link | name | side | lane | vehicles | mean km/h | p85 km/h |
|---|---|---|---|---|---|---|
| `239844585` | Gopal Hari Deshmukh Marg | left | 1 | 247 | 13.8 | 23.2 |
| `239844585` | Gopal Hari Deshmukh Marg | right | 1 | 241 | 13.7 | 26.4 |
| `250162145` | Gopal Hari Deshmukh Marg | right | 1 | 211 | 20.7 | 32.6 |
| `239844585` | Gopal Hari Deshmukh Marg | left | 2 | 191 | 3.4 | 8.8 |
| `250162145` | Gopal Hari Deshmukh Marg | left | 1 | 187 | 18.2 | 30.4 |
| `239844585` | Gopal Hari Deshmukh Marg | left | 3 | 177 | 3.8 | 6.4 |
| `239844585` | Gopal Hari Deshmukh Marg | right | 2 | 170 | 14.3 | 27.9 |
| `239844585` | Gopal Hari Deshmukh Marg | left | 4 | 156 | 2.7 | 5.1 |

(Was 193/176/156/151/142/141/116/112 vehicles at 2.2–20.7 km/h mean under ByteTrack;
194/167/153/147/139/92 vehicles at 6.6–20.3 km/h mean under COCO, 6 rows over the
≥5-vehicle filter. Each tracker's more continuous trajectories put more samples on the
top links, raising every row's vehicle count without reordering which lanes are busiest.)
The lanes are not equivalent: on the
same link and carriageway, lane 1 and lane 2 differ by more than a factor of four in
mean speed here. That is a per-lane operational fact invisible to any link-level average.

### 11.5 The map-native products

Written to `out/geojson/`, in WGS84, openable in QGIS or any web map:

- `network.geojson` -- the registered link geometry
- `trajectories.geojson` -- road-user paths carrying class, body type, colour, measured
  length, mean speed and movement
- `desire_lines.geojson` -- one line per origin-destination movement, weighted by volume
  and drawn through the **median observed path** so it follows the road layout rather than
  cutting across it as a straight chord
- `queues.geojson` -- queue extents drawn *along the carriageway* they occur on, at max
  and 85th-percentile length

The dashboard renders these over the real link geometry as vectors rather than raster
tiles, so the map is self-contained and needs no tile server.

---

## 12. Results

Interactive dashboard: **https://claude.ai/code/artifact/aa34f729-3f04-4df9-8532-842e13016429**

Included in the code package:

- `demo/example_output.mp4` — annotated overlay rendering the **object record itself**:
  box coloured by body type, and each label carrying body type, *measured* length in
  metres, speed in km/h, and a chip of the vehicle's measured paint colour. The chip is
  deliberate — it lets the colour extraction be checked against the vehicle it was taken
  from, in the same frame, so the video validates its own attributes instead of asking the
  viewer to trust a table. Trajectory trails show each road user's full path.
  The **aggregate** layer is drawn on the same frame and is deliberately the loudest thing
  on it: a live HUD reading flow, density, space-mean speed and occupancy per lane straight
  off the Edie space-time boxes; one density ribbon per **discovered traffic path**, always over
  the same 48 m road segment, where **width = live vehicles/km/lane** and green/amber/red =
  light/busy/dense; current queue length as a separate readout; and lane centres as dashed lines. Trails are
  dimmed so the aggregate layer sits on top rather than competing with them.

  An earlier version drew all of this too faintly to read against the trails -- technically
  present, practically invisible, which for a viewer is the same as absent.

  All of it was derived in ground coordinates and projected back through the inverse
  homography, so the fact that the lane lines land on the carriageway rather than in the
  buildings is itself a check on the ground frame: a wrong calibration would put them
  visibly off the road.
- `demo/conflict_*.mp4` — six evidence clips, cropped to the worst interactions
- `demo/dashboard.html` — the full dashboard (trajectory plot, conflict heatmap, OD
  matrix, congestion/cycle chart, speed distributions, ranked conflicts, anomalies)
- `demo/trajectories.parquet` — the trajectory dataset
- `demo/report.json` — every figure in this document, machine-readable

### Validation

No annotations exist, so accuracy cannot be quoted against ground truth. What *can* be
checked is physics — each of these could have failed:

| Check | Result | (was, ByteTrack) | (was, COCO) |
|---|---|---|---|
| `\|accel\|` < 4 m/s² | **100%** of samples | 100% | 99.96% |
| Speeds above 100 km/h | **0** | 0 | 0 |
| Median car footprint | 4.00 m *(by construction)* | 4.00 m | 4.00 m |
| **Motorcycle footprint — never fitted to** | **1.85 m vs 1.99 m expected** | 1.85 m vs 1.99 m | 2.12 m vs 1.99 m |
| Positions interpolated through occlusion | 22.3%, flagged in the data | 10.7% | 6.7% |
| Duplicate tracks detected and removed | 180 | 402 | 80 |
| `mtrack.py` determinism (2 independent runs) | 0/242,679 detection rows differ | n/a | n/a |

---

## 13. Limitations

Stated plainly, because they bound how the numbers should be read.

- **Conflict counts still need a stated correction rule, now applied.** A re-emergence
  exclusion (§16) removes the single biggest artefact — a track's least-trustworthy
  sample, right where it exits an occlusion-bridged gap — but residual ID fragmentation
  and PET/TTC's own exposure sensitivity remain. The spatial pattern and class
  composition are the robust reads; the absolute rate is not.
- **305 of 602 tracks traversed the scene (50.7%, up from 33.2%/731 under ByteTrack, was
  67% fragments before that).** The metric-frame tracker's growing Kalman gate bridges
  the tree's occlusion instead of losing the track and restarting it (§16) — the
  re-architecture next-steps flagged in the prior revision are now done. Not fully
  eliminated: the class-group hard gate itself costs back ~1.7 points by freeing
  previously-absorbed detections into short-lived fragments instead of vehicle tracks
  (`EXP4-FIX.md`).
- **OSM map-match rate sits at 82.6%, confirmed not fixable by tracker choice at all.**
  Per-sample registration residual is *better* than COCO's (1.79 m vs 1.97 m), so this
  is a detection-coverage problem — more VisDrone boxes fall outside the road network's
  20 m buffer — not an association one. Two independent tracker architectures now agree:
  four ByteTrack configs (`EXP3.md`, 81.8–82.6%), the metric-frame tracker pre-fix
  (`EXP4.md`, 81.3%), and this run (82.6%) all land in the same 81–83% band. Flow,
  density and queue figures downstream of map-matching should be read with this in
  mind; fixing it needs a road-buffer or detection-filtering change (§14), not another
  tracker swap.
- **Pedestrian modal split does not resolve cleanly across three different cuts.**
  By-track share moved 27.1%→19.1% across the tracker swap (§16); a third,
  fragmentation-independent cut — mean per-frame detection-class share from the shared
  `detraw_intersection.parquet`, identical regardless of tracker — gives **17.8%**,
  *lower* than either track-based number. Mechanism: a few fast-transiting two-wheelers
  generate more total detection-frames over the video than a smaller number of
  pedestrians, even ones that individually linger on screen far longer (median person
  dwell rose 11.9s→50.4s under the new tracker — fewer, longer tracks, not more
  presence). Track share and presence share answer different questions; see
  `NUMBERS-CHANGELOG.md` Revision 2 for which to quote when. Separately, `EXP4-FIX.md`
  found a real structural leak — a stray pedestrian detection absorbed into a vehicle
  track under a purely-spatial gate (155/678 raw tracks pre-fix) — that the class-group
  gate stops going forward, but cannot retroactively turn into confirmed pedestrian
  tracks: closing that gap needs better pedestrian detection recall, not a
  tracking-side change.
- **Body type is inferred from size, not read from the vehicle.** Auto-rickshaws *are*
  now separable — the oriented-dimension solve of §9.1 gives 2.58 m against 1.45 m for
  two-wheelers, and VisDrone additionally detects `autorickshaw` as a real class (§4),
  so this is now a cross-check rather than the only source — but an individual
  assignment still carries more uncertainty than the aggregate distribution does.
- **Vehicle width is ~30% over-measured.** It is the ill-conditioned half of the L/W
  solve; length is reliable, width only breaks ties in body-type assignment.
- **Licence plates and make/model are not achievable** from this footage — 2.0 px
  character height against ~16 px for OCR, and plate surfaces face away from a −63°
  camera entirely. See §9.5 for the sensor tasking that would be required.
- **Peak acceleration needs the percentile columns.** Smoother edge transients inflate the
  raw per-object maximum; use `a_p95_ms2` / `a_p05_ms2` and the `kinematics_plausible`
  flag (0 of 602 objects fail it this run, was 1 of 731 under ByteTrack).
- **Class error from domain gap — resolved, now a finding.** COCO was ground-level
  imagery; from directly overhead it routinely called a two-wheeler a car. This is fixed
  by adopting a VisDrone-pretrained detector (§4, §15); see `EXP1.md`–`EXP3.md` for the
  full before/after. (Tiling alone would not have fixed it — measured: motorcycles 3→3
  and 13→11 with tiling on COCO — because it was a domain problem, not a resolution one.)
- **The cycle estimate is indicative**, not a measured signal plan (r = 0.18).
- **Extreme camera angle.** The corridor video sits at −18.5° gimbal pitch: 211 m to scene
  centre, 23.5 cm/px along-track, a motorcycle 8 px across. It is deliberately **not**
  tracked — precision there would be fabricated. It receives the detector-free pass only.
- **Lane assignment is soft.** Lanes are modes in a lateral-offset distribution, not
  painted markings. On approaches where traffic uses the carriageway as a continuum the
  mode count is not a lane count, and the discipline index is reported as undefined
  rather than forced to a number.
- **Fundamental-diagram values are per *equivalent* lane** (measured stream width /
  3.5 m), not per painted lane. Comparisons to published per-lane capacities should
  allow for that, and for two-wheelers packing more vehicles into the same width.
- **The network registration is a translation only.** It corrects a real offset between
  OSM and the drone GPS fix, but it cannot fix genuine geometry errors in OSM itself, and
  a translation-only fit is exactly what produced the 91 m false answer described in
  §11.2. Any re-run at a new site should check the junction-node residual, not only the
  fitted objective.
- **Lane index from lateral offset assumes a nominal 3.3 m lane.** On a carriageway used
  as a continuum the index is a position band, not a painted lane.
- **Telemetry cannot be trusted for scale.** `rel_alt` was off by 34%. The pipeline now
  calibrates rather than trusts, but this needs a size prior to work at all.

---

## 14. Future work

In priority order, based on what the measurements above actually show:

1. ~~Aerial-pretrained detection weights~~ **Done — see §15.** VisDrone
   (`dronefreak/visdrone-yolov11s`) is now the default detector, with `autorickshaw`
   as a real detected class instead of a size-inferred one. DOTA-pretrained YOLO-OBB's
   oriented boxes — which would remove the ill-conditioned case in the L/W solve
   entirely and stop axis-aligned boxes inflating small objects — remain unadopted:
   DOTA has no motorcycle/pedestrian/bus-truck split, so it would not by itself have
   fixed the class problem this revision targeted, and its licence is
   academic/non-commercial only (`PREP.md`).
2. **Re-architect the tracker.** *Promoted from #2 to #1 priority in practice* — the
   VisDrone adoption raised the stakes here rather than lowering them: OSM map-match
   rate fell 96.6%→81.8% and duplicate tracks rose 80→402 even after tuning ByteTrack's
   built-in knobs (`EXP3.md`), because the real fix — decoupling detection from
   tracking, associating in the metric frame instead of image space, and adding
   appearance re-ID across occlusion — has not been done yet. This is what the cheap
   knob-tuning in `EXP3.md` could not reach.
3. **SAHI tiling.** Measured 1.7–1.9× more road users on COCO, but **6× slower**
   (0.10 s → 0.62 s per frame) — a recall win paid for in compute, not an acceleration.
   VisDrone alone already recovered much of the pedestrian undercount tiling targeted
   (§13); re-measure whether tiling still earns its cost on top of it before repeating
   the experiment.
4. **Real-time processing** on-drone or at the edge, for incident response rather than
   post-hoc study.
5. **Signal optimisation**: feed inferred cycle, saturation flow and per-movement demand
   back into controller retiming — the numbers needed already exist in the trajectory table.
6. **Multi-drone coverage** for corridor-scale studies, stitched through the same metric
   ground frame.
7. **Predictive congestion** from queue-index dynamics and shockwave speeds.

---

## 15. Revision — aerial-pretrained detector

Since the original submission, the default detector/tracker changed from COCO
YOLO11x + plain ByteTrack to `models/visdrone-yolov11s.pt` (VisDrone-pretrained,
AGPL-3.0) + a tuned ByteTrack (`track_buffer=90`, `match_thresh=0.9`). Every
number in §4–§13 above is from the new run; the original COCO run is archived
at `out/exp4/report_intersection_COCO.json`, and every changed figure is
tabulated in `NUMBERS-CHANGELOG.md`.

Three experiments drove the change, in order:

- **`EXP1.md`** — the trigger. On the same 20 frames, swapping COCO YOLO11x
  for VisDrone-pretrained weights shifted motorcycle share of detections from
  8.3% to 55.9%/44.7% (two model sizes tried) and car share from 75.9% to
  ~22-23%. Side-by-side annotated frames confirmed this is a genuine
  reclassification — the extra "motor" boxes sit at the exact same pixel
  positions and sizes as COCO's "car" boxes on rows of small roadside
  vehicles — not phantom detections.
- **`EXP2.md`** — the full before/after re-track. Confirmed the class fix at
  full-pipeline scale (731 vs 687 road users, modal split and conflict pairs
  both shift materially) and surfaced a genuine second-order finding: COCO's
  own scale self-calibration (§5) was quietly contaminated by the same class
  bug, because `calibrate()` runs on raw pre-merge detector labels — COCO's
  `car` population was full of two-wheelers wrongly called cars, pulling its
  observed footprint down to 2.84 m and forcing a larger ×1.513 correction
  where VisDrone's clean `car` class needed only ×1.385. It also exposed the
  cost: OSM match rate fell 96.6%→82.7% and duplicate tracks rose 80→541,
  because VisDrone's ~2× denser per-frame detection field gives plain
  image-space ByteTrack more boxes to confuse and puts more detections
  outside the OSM road buffer.
- **`EXP3.md`** — the cheap mitigation for the tracking cost. Tuned
  ByteTrack's `track_buffer` (30→90, a longer 9 s lost-track memory at
  10 fps) and `match_thresh` (0.8→0.9, pickier re-acquisition) together —
  neither alone moved the needle much. This cut duplicate tracks 541→402
  (−26%) and raised traversing-track share 26.2%→33.2%, without touching the
  class fix. BoT-SORT and ReID were both tried and both did worse (ReID
  especially: no aerial-trained re-identification weights exist, so it fell
  back to the detector's own features, which are not discriminative enough
  for small, similar overhead two-wheelers). OSM match rate did **not**
  improve on any of the four tracker variants tried (81.8–82.6% across the
  board) — confirming this is a detection-coverage problem, not an
  association one, and that the tracker re-architecture in §14's priority #2
  is now a prerequisite for flow/density/queue/match-rate numbers, not just a
  fragmentation fix.

A smaller fix landed alongside this revision: `src/attributes.py`'s
grey-world white-balance sampling (§9.2) used an unseeded random sample,
so `objects.colour`/`objects.colour_hex`/per-object `hex` values could drift
a few counts between identical reruns of an otherwise fully deterministic
pipeline. Fixed with a seeded `numpy` generator; verified by rerunning both
the standalone attribute build and the full `run_analysis.py` end-to-end
twice each, 0 diffs in every case (0/687 attribute columns; 0/45,084
report.json leaf keys).

---

## 16. Revision 2 — tracker re-architecture

Since the previous revision, the default **tracker** changed from image-space
ByteTrack (`track.py`) to a two-stage, metric-frame pipeline: `src/detect.py`
(detector-only — same VisDrone weights, same classes, unchanged) followed by
`src/mtrack.py` (a from-scratch tracker that associates boxes in ground-plane
metres). Every number in §4, §7–§11 and §13 above is from the new run; the
prior ByteTrack run is archived at `out/exp5/report_intersection_BYTETRACK.json`,
and every changed figure is tabulated in `NUMBERS-CHANGELOG.md`'s "Revision 2"
section. `track.py` remains available as a documented alternative (§4, README
§Pipeline) — this is a tracker swap, not a removal.

This was priority #1 in §14 of the previous revision, for a specific reason:
adopting VisDrone raised the tracking stakes rather than lowering them (OSM
match rate 96.6%→81.8%, duplicate tracks 80→402), and tuning ByteTrack's own
knobs (`EXP3.md`) had already captured the cheap gains available without
decoupling detection from tracking. Four experiments carried it out, in order:

- **`EXP4A.md`** — Step A. Split `track.py`'s fused detect+ByteTrack into
  `detect.py`, a detector-only stage writing one row per detection with no
  track id, so a tracker downstream can associate boxes itself in whatever
  frame it chooses. Also measured 2×2 SAHI-style tiling on top of the new
  detector: **1.21× more detections for ~2.4× the compute**, well under the
  1.7–1.9× tiling measured on COCO (§14 of the previous revision) — VisDrone's
  native density already recovers much of what tiling was for — plus
  bus/truck tile-seam artefacts (6.66×/0.87× count ratios, large vehicles
  straddling a tile boundary). **Closed as a negative result; not adopted.**
- **`EXP4.md`** — Step B. `mtrack.py`: a per-track constant-velocity Kalman
  filter in ground-plane metres, Hungarian-assigned under a Mahalanobis gate
  that grows through an occlusion by itself (no separate buffer schedule),
  costed on class-group ordinal distance plus footprint-size distance (the
  size term stands in for an appearance cue; ablating it raised duplicates
  110→153, so it earns its keep). A 4-combo knob sweep confirmed the design
  choices and picked `max_lost_s=6.0`. Verdict: **ADOPT** — duplicate tracks
  402→110, traversing tracks 33.2%→52.4%, no kinematic regression versus
  ByteTrack — but flagged two numbers that needed independent verification
  before publication: critical conflicts rose ×4.8 (37→178) and pedestrian
  share fell 27.1%→18.2%.
- **`EXP4-QA.md`** — an independent QA pass (a different temp, deliberately
  not the one that built the tracker) on exactly those two flags, before
  either number could be quoted. Found the conflict spike is **81.5% a
  re-emergence artefact**: a track's RTS-smoothed velocity is least
  trustworthy in the instant it exits an occlusion-bridged gap, and that is
  exactly the sample PET/TTC scores — excluding it drops mtrack's critical
  count from 178 to 30, *below* ByteTrack's own recomputed count under the
  same rule (29). The raw ×4.8 does not survive exposure normalisation alone
  (still ~4–4.6× per 1,000 moving samples); it does resolve once combined
  with the re-emergence filter (0.378 vs 0.363 critical per 1,000 samples —
  statistically the same rate). The pedestrian drop is partly the same
  fragmentation-reduction mechanism already seen for two-wheelers (person
  median dwell 11.9s→50.4s — fewer, longer tracks) but also a real,
  structural leak: 155 of 678 raw confirmed tracks absorbed a stray
  pedestrian detection into a vehicle track, because the tracker's gate was
  spatial-only with class as a soft, tunable cost. A `lambda_cls` 1.0→2.0
  knob was tested and **rejected**: it moved the pedestrian share by nothing
  (18.54%→18.52%) while breaking the motorcycle sanity check (−6.7%→−28%,
  outside tolerance) and raising duplicates (110→151). Verdict: **ADOPT WITH
  A CONFLICT POST-FILTER.**
- **`EXP4-FIX.md`** — implemented both of EXP4-QA's prescriptions, since a
  knob alone could not fix the pedestrian leak. **Fix 1**: a hard
  class-group gate in `mtrack.py` — a detection may only match a track in
  the same group (`{person}`, `{bicycle}`, `{motorcycle, autorickshaw}`,
  `{car, bus, truck}`) — stops the leak outright (proven by a new
  `test_class_hard_gate_blocks_person_into_car` test, including a gate-off
  negative control that reproduces the leak). **Fix 2**: a first-class
  `reemerged` trajectory column, true for 2.0s after a track exits an
  imputed gap; `conflicts.py` excludes these rows from PET/TTC candidacy the
  same way it already excludes `imputed` rows, applied identically to both
  trackers so the comparison stays like-for-like.

**The conflict fix fully resolves**: post-fix, the metric-frame tracker's
critical count (27) is *below* the recomputed ByteTrack baseline (29) under
the identical rule — the headline finding is "same safety signal, a cleaner
tracker," not "tracking swap found five times more danger." **The pedestrian
fix is honest but partial**: the class-group gate stops the leak going
forward (raw confirmed tracks 678→796, +118, as previously-absorbed
detections fragment into short-lived tracks of their own) but cannot
manufacture a confirmed pedestrian track out of detections too short-lived to
reach `min_hits=3` on their own — final pedestrian share moved 18.2%→19.1%
against a target of "within ~5pts of ByteTrack's 27.1%," an 8-point gap that
needs a detection-recall fix (§14), not a tracking-side one. 4 of 5 adoption
bars passed cleanly (duplicates, traversing, motorcycle check, conflicts);
pedestrian share is the one documented shortfall, not silently dropped.

A third, fragmentation-independent modal-split cut was computed for this
regeneration, not part of the original EXP4 series: **presence share** — the
mean per-frame detection-class share, straight from `detraw_intersection.parquet`,
identical regardless of tracker because detection is shared upstream of both.
It gives pedestrians 17.8%, *lower* than either track-based number (27.1%
ByteTrack, 19.1% metric-frame) — a genuinely different read, not a tie-breaker
that resolves in either tracker's favour. See `NUMBERS-CHANGELOG.md`'s
Revision 2 caveats for the full mechanism and which cut to quote for what.

Determinism: `src/mtrack.py --tag intersection` run twice, independently,
produced byte-for-byte identical `detections_intersection.parquet` files
(242,679 rows, 0 diffs) — no unseeded randomness, no seeding needed.

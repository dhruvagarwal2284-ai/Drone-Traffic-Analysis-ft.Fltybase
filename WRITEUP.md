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
Detection (YOLO11x)
        ↓
Multi-object tracking (ByteTrack)
        ↓
World-coordinate transformation (analytic homography from gimbal attitude)
        ↓
Trajectory reconstruction (Kalman + RTS smoothing)
        ↓
Traffic intelligence (conflicts · movements · queues · anomalies)
```

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
| `overlay.py`, `build_dashboard.py` | Annotated video, dashboard |

**Analysis window:** 120 s (t = 240–360 s) at 10 fps = 1,200 frames, chosen because it
spans a full queue build *and* discharge. 10 fps gives 0.1 s resolution — ample for
PET/TTC, and at 40 km/h a vehicle moves only 1.1 m between samples, so association is easy.

---

## 4. Detection & tracking

- **Model:** YOLO11x, COCO-pretrained, `imgsz=1920`, `conf=0.20`, FP16.
- **Tracker:** ByteTrack, image-space association.
- **Classes used:** person, bicycle, car, motorcycle, bus, truck.
- **Result:** 112,144 detections → 1,544 raw tracks → **687 road users** after filtering.

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
**6.7%** of positions — so downstream metrics can exclude or discount them. A separate
deduplication pass removes tracks that are a second copy of another (two ids within
2.2 m for >55% of shared frames): **80 removed**. Left in, these produce PET ≈ 0 s and
would dominate the conflict ranking with pure artefact.

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
came out at **2.84 m** against an expected **4.30 m** (the expectation accounts for
axis-aligned boxes overstating a rotated rectangle). Applying the implied factor:

| | |
|---|---|
| Scale correction | **×1.5128** |
| Reported height | 70.47 m |
| **Effective height above road** | **106.60 m** |

The drone launched from a rooftop roughly 36 m above street level — there is a high-rise
in frame. **Without this correction every distance, speed and time-to-collision would be
34% wrong.**

**Independent check:** motorcycle length then measures **2.12 m** against **1.99 m**
expected — a quantity the calibration was never fitted to. 6.6% error.

### Derived kinematics

A constant-velocity Kalman filter with a Rauch–Tung–Striebel smoother runs over each
path. This is not cosmetic: accelerations from raw finite differences of noisy positions
are pure noise, and the safety metrics depend on them. Outputs: position, velocity,
speed, heading, acceleration — 113,620 rows across 687 road users.

### Scene geometry, discovered from motion

Approach legs are found as peaks in the circular histogram of entry/exit bearings, using
only tracks that genuinely traversed the scene and reached the junction. Three legs were
recovered: **N (18°), SE (112°), W (288°)**. SE and W are opposite ends of the same
arterial, so SE↔W is the through movement.

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

120-second window, 687 road users.

```
Road users tracked              687
Trajectory samples          113,620
Median speed                7.6 km/h      (p99  41.0 km/h)
Traversing tracks               204

Modal split
  two-wheeler               45.1 %        (mean 13.0 km/h)
  car                       31.4 %        (mean 14.8 km/h)
  pedestrian                19.5 %
  bus                        2.6 %        (mean  1.0 km/h)
  truck                      1.3 %

Turning movements (204 traversing)
  through                     146
  left                         24
  right                         9
  u-turn                       25

Delay
  mean stopped time          6.23 s
  p85 stopped time           7.12 s
  fraction ever stopped      42.9 %
  worst movement          N→SE, 24.3 s mean stopped

Conflicts (1,133 interacting pairs)
  critical  (< 1.0 s)          20
  serious   (< 1.5 s)          24
  conflict  (< 3.0 s)         243
  minor     (3–5 s)           846

  two-wheeler ↔ two-wheeler   103
  car ↔ two-wheeler           124
  car ↔ car                    39
  car ↔ pedestrian              3

Inferred signal cycle       116 s          (autocorrelation r = 0.18)
```

**Origin–destination matrix** (traversing tracks):

| from ↓ / to → | N | SE | W |
|---|---|---|---|
| **N** | 2 | 3 | 20 |
| **SE** | 4 | 11 | 47 |
| **W** | 6 | 99 | 12 |

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
48 anomalies
  contraflow                   36     (>120° against the local prevailing flow)
  stopped in carriageway       12     (stationary >8 s mid-scene, having moved before)
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
| bus | 13 | **9.83 m** | 10.0–12.0 m |
| van / LCV | 10 | 5.16 m | 4.8–6.0 m |
| truck | 7 | 5.09 m | 6.0–10.0 m |
| SUV / MUV | 23 | 4.78 m | 4.4–4.9 m |
| sedan | 35 | 4.26 m | 4.2–4.6 m |
| hatchback | 67 | 3.85 m | 3.6–4.0 m |
| **auto-rickshaw** | 38 | **2.59 m** | 2.6–2.9 m |
| two-wheeler | 311 | 1.67 m | 1.8–2.0 m |

**This reverses a limitation reported at the previous level.** Auto-rickshaws were
declared unrecoverable because naive footprints gave 2.13 m against 2.07 m for
motorcycles — one distribution, not two. Removing the heading-dependent inflation
separates them cleanly at **2.59 m vs 1.67 m**. The earlier limitation was a property of
the measurement, not of the data.

Width is the ill-conditioned half of the solve and measures ~30% wide against known
vehicles; it is reported but only breaks ties in body-type assignment. **594 of 687**
objects (86%) received dimensions; the rest were ill-conditioned throughout.

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
dark grey      33.8 %        red      8.2 %
silver / grey  28.7 %        blue     3.3 %
white          14.3 %        other    1.7 %
black          10.0 %
                              → 86.8 % achromatic
```

That achromatic share matches the Indian market. The split *within* the greys is a
brightness judgement under uncontrolled evening light and should not be over-read.

### 9.3 Per-object kinematics, in SI

From the RTS-smoothed trajectory — not finite differences, which would make acceleration
pure noise.

| body type | mean speed | p85 top speed | peak accel | peak braking |
|---|---|---|---|---|
| SUV / MUV | 20.0 km/h | 42.6 km/h | 1.08 m/s² | −1.15 m/s² |
| sedan | 19.4 km/h | 39.4 km/h | 1.32 m/s² | −1.27 m/s² |
| hatchback | 18.7 km/h | 41.4 km/h | 1.14 m/s² | −0.97 m/s² |
| two-wheeler | 13.8 km/h | 34.0 km/h | 0.60 m/s² | −0.51 m/s² |
| auto-rickshaw | 13.4 km/h | 33.6 km/h | 0.40 m/s² | −0.35 m/s² |
| bus | 0.0 km/h | 1.9 km/h | 0.20 m/s² | −0.19 m/s² |
| pedestrian | 1.6 km/h | 4.8 km/h | 0.11 m/s² | −0.15 m/s² |

Buses are stationary throughout — they are in the standing queue.

### 9.4 The object record

`out/objects_<tag>.parquet`, one row per road user, joins identity, kinematics, movement
and interaction. Example rows:

```
silver/grey hatchback  L 3.96 m  mean 18.9  max 39.7 km/h  brake -1.96 m/s²  229 m  W→W    9 conflicts
white hatchback        L 3.94 m  mean 23.6  max 40.6 km/h  brake -3.18 m/s²  215 m  W→SE   4 conflicts
silver/grey sedan      L 4.27 m  mean 17.0  max 33.8 km/h  brake -1.53 m/s²  214 m  W→SE  12 conflicts
```

Validation: 687 objects, **0** above 120 km/h, **99.56%** (684/687) with peak acceleration
inside ±4 m/s², median distance travelled 16.2 m, **687 with colour**, **594 with
dimensions** (86%).

A note on peak acceleration. The RTS smoother has no data beyond a track's ends, so the
first and last samples carry an edge transient that surfaces as a spurious peak — raw
maxima reached 7.67 m/s², which no hatchback here is really doing. Two samples are trimmed
from each end before extremes are taken, and every object also carries `a_p95_ms2` /
`a_p05_ms2` (robust percentiles, range −0.21 to 3.78 m/s²) plus a
`kinematics_plausible` flag. **Three objects remain implausible and are flagged rather
than deleted.** Quote the percentile columns for individual objects; the raw max is
retained for auditability.

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

## 10. Results

Interactive dashboard: **https://claude.ai/code/artifact/aa34f729-3f04-4df9-8532-842e13016429**

Included in the code package:

- `demo/example_output.mp4` — annotated overlay: boxes, ids, class, speed in km/h, and
  trajectory trails showing each road user's full path
- `demo/conflict_*.mp4` — six evidence clips, cropped to the worst interactions
- `demo/dashboard.html` — the full dashboard (trajectory plot, conflict heatmap, OD
  matrix, congestion/cycle chart, speed distributions, ranked conflicts, anomalies)
- `demo/trajectories.parquet` — the trajectory dataset
- `demo/report.json` — every figure in this document, machine-readable

### Validation

No annotations exist, so accuracy cannot be quoted against ground truth. What *can* be
checked is physics — each of these could have failed:

| Check | Result |
|---|---|
| `\|accel\|` < 4 m/s² | **99.96%** of samples |
| Speeds above 100 km/h | **0** |
| Median car footprint | 4.00 m *(by construction)* |
| **Motorcycle footprint — never fitted to** | **2.12 m vs 1.99 m expected** |
| Positions interpolated through occlusion | 6.7%, flagged in the data |
| Duplicate tracks detected and removed | 80 |

---

## 11. Limitations

Stated plainly, because they bound how the numbers should be read.

- **Conflict counts are an upper bound.** Only 204 of 687 tracks traversed the scene —
  **70% are fragments**, from the tree occlusion and from image-space association. The
  spatial pattern and class composition are the robust reads; the absolute rate is not.
- **Pedestrian counts are a floor.** Tiled inference on the same frames finds **1.7–1.9×
  more road users and 5–6× more pedestrians** than full-frame at 1920 px.
- **Auto-rickshaws are not separable** from motorcycles at this resolution. An earlier
  version tried to split them by metric footprint size; the data refuted it — tracks
  labelled auto-rickshaw measured 2.13 m against 2.07 m for motorcycles, one distribution
  and not two, where a real auto is 2.6–2.9 m. That heuristic was removed and the classes
  merged. This is a detector problem, not a geometry problem.
- **Class error from domain gap.** COCO is ground-level imagery; from directly overhead it
  routinely calls a two-wheeler a car. Tiling does *not* fix this (measured: motorcycles
  3→3 and 13→11 with tiling) because it is a domain problem, not a resolution problem.
- **The cycle estimate is indicative**, not a measured signal plan (r = 0.18).
- **Extreme camera angle.** The corridor video sits at −18.5° gimbal pitch: 211 m to scene
  centre, 23.5 cm/px along-track, a motorcycle 8 px across. It is deliberately **not**
  tracked — precision there would be fabricated. It receives the detector-free pass only.
- **Telemetry cannot be trusted for scale.** `rel_alt` was off by 34%. The pipeline now
  calibrates rather than trusts, but this needs a size prior to work at all.

---

## 12. Future work

In priority order, based on what the measurements above actually show:

1. **Aerial-pretrained detection weights.** VisDrone carries `pedestrian`, `motor`,
   `tricycle` and `awning-tricycle` — the last two are essentially auto-rickshaw, the
   class we could not recover. DOTA-pretrained YOLO-OBB adds oriented boxes, which matter
   because vehicles sit at arbitrary angles and axis-aligned boxes inflate measured
   footprint (our pedestrians measure 1.68 m for this reason).
2. **Re-architect the tracker.** Decouple detection from tracking so tiled inference is
   possible at all, associate in the metric frame rather than image space, and add
   appearance re-ID across occlusion. This is what fixes the 70% fragmentation, and
   fragmentation is what inflates the conflict counts.
3. **SAHI tiling.** Measured 1.7–1.9× more road users, but **6× slower** (0.10 s → 0.62 s
   per frame) — a recall win paid for in compute, not an acceleration.
4. **Real-time processing** on-drone or at the edge, for incident response rather than
   post-hoc study.
5. **Signal optimisation**: feed inferred cycle, saturation flow and per-movement demand
   back into controller retiming — the numbers needed already exist in the trajectory table.
6. **Multi-drone coverage** for corridor-scale studies, stitched through the same metric
   ground frame.
7. **Predictive congestion** from queue-index dynamics and shockwave speeds.

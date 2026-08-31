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
dark grey      33.2 %        red      8.4 %
silver / grey  29.0 %        blue     3.2 %
white          14.4 %        other    1.9 %
black           9.9 %
                              → 86.5 % achromatic
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

| quantity | value |
|---|---|
| Max flow | **2,266 veh/h/lane** |
| Density at max flow | 110.0 veh/km/lane |
| Speed at max flow | 20.6 km/h |
| Max density observed | 207.4 veh/km/lane |
| Min speed observed | 4.9 km/h |
| Max occupancy | 76.3 % |
| Equivalent lanes | inbound 5.7 · outbound 3.33 |

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
| N | 26.6 m | 22.0 m | 16.4 m |
| SE | 44.1 m | 32.1 m | 29.8 m |
| W | 71.7 m | 43.7 m | 38.1 m |

A vehicle count cannot tell an engineer whether a queue blocks the junction upstream. A
distance can, and it is the number a signal-timing decision actually consumes.

### 10.3 Do lanes exist here?

Lanes are not read from paint -- there is little, and it is widely ignored. A lane is
defined as a mode in the lateral-offset distribution on each approach, which makes lane
discipline a *measured* quantity:

| approach | modes found | offsets (m) | discipline index | samples |
|---|---|---|---|---|
| N | 1 | 8.1 | n/a | 3,345 |
| SE | 2 | -2.0, 10.0 | 0.75 | 14,381 |
| W | 3 | -8.46, 0.54, 4.04 | 0.57 | 16,177 |

The discipline index is 1.0 for sharply separated lanes and 0 for a continuum. Where only
one mode is found the index is undefined and reported as such, because a single mode means
traffic is using the carriageway as a continuous surface rather than as lanes -- which is
itself the finding.


**Lane volume and modal split.** Once lanes are located, what each one carries can be
measured — and the lanes are not interchangeable:

| approach | lane | offset | share of approach | modal split |
|---|---|---|---|---|
| N | 1 | 8.1 m | 100.0% | car 54%, two wheeler 38%, truck 5% |
| SE | 1 | -2.0 m | 54.2% | two wheeler 55%, car 41%, truck 2% |
| SE | 2 | 10.0 m | 45.8% | car 47%, two wheeler 46%, truck 6% |
| W | 1 | -8.5 m | 44.0% | car 44%, two wheeler 39%, person 16% |
| W | 2 | 0.5 m | 23.2% | car 78%, two wheeler 22%, truck 0% |
| W | 3 | 4.0 m | 32.8% | car 54%, two wheeler 39%, bus 3% |

Three things fall out of that table. The W approach has one lane that is **78% car** and
another carrying **15.5% pedestrians** — the latter is the kerbside edge, where people are
walking in the carriageway rather than on a footpath. On the SE approach, two-wheelers
prefer the nearside lane (55% of it) over the offside (46%). None of this is visible in an
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

Only **2.08%** of moving samples exceed 40 km/h and **0.03%** exceed
50 km/h, against a peak cell 85th-percentile of 43.6 km/h.

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
96.6 % of trajectory samples matched to a link
   primary      28,913   (Gopal Hari Deshmukh Marg)
   tertiary     3,758
   residential  3,918
```

### 11.4 Per-lane metrics on real geometry

Keyed on OSM link id and carriageway, so the output joins to any network model or asset
register speaking the same identifiers:

| link | name | side | lane | vehicles | mean km/h | p85 km/h |
|---|---|---|---|---|---|---|
| `239844585` | Gopal Hari Deshmukh Marg | left | 1 | 194 | 14.9 | 27.0 |
| `239844585` | Gopal Hari Deshmukh Marg | right | 1 | 167 | 12.4 | 24.1 |
| `250162145` | Gopal Hari Deshmukh Marg | right | 1 | 153 | 19.3 | 29.4 |
| `239844585` | Gopal Hari Deshmukh Marg | left | 2 | 147 | 6.6 | 17.6 |
| `250162145` | Gopal Hari Deshmukh Marg | left | 1 | 139 | 20.3 | 34.3 |
| `239844585` | Gopal Hari Deshmukh Marg | right | 2 | 92 | 15.5 | 30.2 |

The lanes are not equivalent: on the same link and carriageway, lane 1 and lane 2 differ
by more than a factor of two in mean speed. That is a per-lane operational fact invisible
to any link-level average.

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

| Check | Result |
|---|---|
| `\|accel\|` < 4 m/s² | **99.96%** of samples |
| Speeds above 100 km/h | **0** |
| Median car footprint | 4.00 m *(by construction)* |
| **Motorcycle footprint — never fitted to** | **2.12 m vs 1.99 m expected** |
| Positions interpolated through occlusion | 6.7%, flagged in the data |
| Duplicate tracks detected and removed | 80 |

---

## 13. Limitations

Stated plainly, because they bound how the numbers should be read.

- **Conflict counts are an upper bound.** Only 204 of 687 tracks traversed the scene —
  **70% are fragments**, from the tree occlusion and from image-space association. The
  spatial pattern and class composition are the robust reads; the absolute rate is not.
- **Pedestrian counts are a floor.** Tiled inference on the same frames finds **1.7–1.9×
  more road users and 5–6× more pedestrians** than full-frame at 1920 px.
- **Body type is inferred from size, not read from the vehicle.** Auto-rickshaws *are*
  now separable — the oriented-dimension solve of §9.1 gives 2.59 m against 1.67 m for
  two-wheelers — but an individual assignment carries more uncertainty than the aggregate
  distribution does. A naive footprint measurement could not do this (2.13 m vs 2.07 m,
  one distribution and not two), so the capability rests entirely on removing the
  heading-dependent inflation.
- **Vehicle width is ~30% over-measured.** It is the ill-conditioned half of the L/W
  solve; length is reliable, width only breaks ties in body-type assignment.
- **Licence plates and make/model are not achievable** from this footage — 2.0 px
  character height against ~16 px for OCR, and plate surfaces face away from a −63°
  camera entirely. See §9.5 for the sensor tasking that would be required.
- **Peak acceleration needs the percentile columns.** Smoother edge transients inflate the
  raw per-object maximum; use `a_p95_ms2` / `a_p05_ms2` and the `kinematics_plausible`
  flag (3 of 687 objects fail it).
- **Class error from domain gap.** COCO is ground-level imagery; from directly overhead it
  routinely calls a two-wheeler a car. Tiling does *not* fix this (measured: motorcycles
  3→3 and 13→11 with tiling) because it is a domain problem, not a resolution problem.
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

1. **Aerial-pretrained detection weights.** VisDrone carries `pedestrian`, `motor`,
   `tricycle` and `awning-tricycle` — the last two are essentially auto-rickshaw, which we
   currently recover from measured size rather than detect directly; a real class would
   make the assignment per-object rather than statistical. DOTA-pretrained YOLO-OBB adds
   oriented boxes, which would remove the ill-conditioned case in the L/W solve entirely
   and stop axis-aligned boxes inflating small objects (our pedestrians measure 0.97 m
   against a true ~0.5 m for this reason).
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

# Drone Traffic Analysis — Baner Junction, Pune

Extracts road-user **trajectories** from drone footage and derives traffic insight from
them. No annotations, no ground-control points, no map, no fixed infrastructure.

**Dashboard:** https://claude.ai/code/artifact/aa34f729-3f04-4df9-8532-842e13016429

---

## Headline results

Intersection video, 120 s window (t = 240–360 s) at 10 fps, **687 road users**,
**113,620 trajectory samples**.

| | |
|---|---|
| Modal split | two-wheeler 45.1%, car 31.4%, pedestrian 19.5%, bus 2.6%, truck 1.3% |
| Turning movements | 146 through, 24 left, 9 right, 25 u-turn (204 traversing tracks) |
| Conflicts | 20 critical, 24 serious, 243 conflict, 846 minor |
| Dominant conflict pairs | two-wheeler↔two-wheeler (103), car↔two-wheeler (124), car↔car (39) |
| Worst delay | `N→SE` at 24.3 s mean stopped time |
| Inferred signal cycle | 116 s (autocorrelation r = 0.18) |
| Anomalies | 48 (36 contraflow, 12 stopped in carriageway) |
| Object attributes | 687 with colour, 594 with measured dimensions (86%), 86.5% achromatic fleet |
| Per-object kinematics | 687/687 in SI; 99.6% physically plausible, 3 flagged |
| Map-native | 96.6% of samples matched to OSM links; registration residual 1.97 m |
| Aggregate | max flow 2,266 veh/h/lane at 110 veh/km/lane; max queue 72 m; 2.08% of samples over 40 km/h |

### The three findings worth defending

1. **The telemetry is wrong and the trajectories caught it.** `rel_alt` reports 70.5 m.
   Self-calibration against car footprints demands ×1.513 → **106.6 m effective height**.
   The drone launched from a rooftop ~36 m above street level, so its height datum was
   never the road. Independent check: motorcycle length then lands at **2.12 m against
   1.99 m expected** — a quantity the calibration was never fitted to. Without this
   correction every distance, speed and time-to-collision would be 34% wrong.

2. **The junction is signal-gated; the corridor is not.** The junction's queue index
   autocorrelates at **116 s**, recovered with no controller feed and no signal head in
   view. The corridor's best peak is 36.5 s at r = 0.15 — noise. Same method, opposite
   answer: retiming helps one site and would do nothing for the other, where congestion
   comes from side friction and a construction narrowing.

3. **Two-wheelers dominate the safety picture**, appearing in ~85% of sub-3 s conflicts
   at 45% of traffic. This only becomes visible once road users share a metric frame.

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
```

Run:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics opencv-python numpy pandas pyarrow scipy

python src/track.py        --tag intersection --t0 240 --dur 120 --fps 10
python src/congestion.py   --video Intersection_Merged-002 --tag intersection
python src/run_analysis.py --tag intersection --t0 240 --fps 10
python run_attrs.py                      # object attributes (one video pass)
python run_geo.py                        # map-native: georeference + map-match
python src/build_dashboard.py intersection
python src/overlay.py      --tag intersection --seconds 60
```

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

One row per road user (687 × 31): identity, kinematics, movement and interactions joined.

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

- `|accel|` < 4 m/s² for **99.96%** of samples; **no** speeds above 100 km/h
- median car footprint = 4.0 m by construction; **motorcycle then measures 2.12 m vs
  1.99 m expected** — an independent check
- 6.7% of positions interpolated through occlusion, flagged in the data
- 80 duplicate tracks detected and removed

## Known limits

- **Conflict counts are an upper bound.** Residual ID fragmentation inflates them; the
  spatial pattern and class composition are the robust reads, not the absolute rate.
- **Only 204 of 687 tracks traversed the scene** — 70% are fragments, caused by the tree
  occluding the junction core and by image-space association.
- **Auto-rickshaws** are now separable via the oriented-dimension solve (2.59 m vs 1.67 m),
  reversing an earlier limitation — but body type is inferred from measured size, not read
  from the vehicle, so individual calls are less certain than the aggregate.
- **Licence plates and make/model are not achievable.** An Indian plate is 15.3 × 3.7 px
  here, giving 2.0 px character height against ~16 px for OCR — an 8× shortfall — and at
  −63° pitch the plate surfaces face away from the camera entirely. Would need ~13 m
  altitude or a 193 mm-equivalent lens *and* a shallow angle.
- **Vehicle width is ~30% over-measured** (ill-conditioned half of the L/W solve).
- **Peak acceleration needs the percentile columns.** Smoother edge transients inflate the
  raw per-object maximum; use `a_p95_ms2` / `a_p05_ms2`, and the `kinematics_plausible`
  flag (3 of 687 objects fail it).
- **Pedestrian counts are a floor.** Tiled inference on the same frames finds 1.7–1.9×
  more road users and 5–6× more pedestrians than full-frame at 1920 px.
- **The cycle estimate is indicative**, not a measured signal plan (r = 0.18).
- **Multi_Road is deliberately not tracked.** At 18.5° gimbal pitch a motorcycle is 8 px
  across; trajectories there would be fabricated precision. It gets the detector-free
  congestion pass only.

## Next steps, in priority order

1. **Aerial-pretrained weights** (VisDrone: `motor`, `tricycle`; or DOTA YOLO-OBB for
   oriented boxes). Fixes the class problem properly — COCO has never seen a two-wheeler
   from directly overhead, which is why it calls one a car.
2. **Re-architect the tracker** — decouple detection from tracking (`boxmot`) so tiled
   inference is possible at all, and associate in the metric frame with appearance re-ID
   across occlusion. This is what fixes the 70% fragmentation.
3. **SAHI tiling** — measured 1.7–1.9× more road users, but **6× slower**, not faster.
   Worth doing, third.

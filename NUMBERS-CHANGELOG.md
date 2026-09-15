# Numbers changelog — VisDrone + tuned ByteTrack adoption

Every figure below changed when the default detector/tracker moved from COCO
YOLO11x + plain ByteTrack to `models/visdrone-yolov11s.pt` + `trackers/bytetrack_buf90.yaml`
(EXP1 → EXP2 → EXP3), regenerated in one clean run on 2026-09-15, tag `intersection`.

Source files:
- **Old** = `out/exp4/report_intersection_COCO.json` (backed up before this run;
  the last COCO-era `report_intersection.json`).
- **New** = `out/report_intersection.json` (this run; also archived at
  `out/exp4/report_intersection_VISDRONE.json`).
- Determinism fix (`src/attributes.py` seeded RNG, see below) applies to both
  files equally going forward — the *old* file's colour/hex values reflect
  whatever the last unseeded run happened to produce and are not themselves
  reproducible; only the method (grey-world sampling) is unchanged.

| # | Section | Metric | Old (COCO) | New (VisDrone) | report.json key |
|---|---|---|---|---|---|
| 1 | Headline | Road users / samples | 687 / 113,620 | **731 / 204,632** | `quality.tracks`, `quality.rows` |
| 2 | Headline | Modal split | 2W 45.1%, car 31.4%, ped 19.5%, bus 2.6%, truck 1.3% | **2W 43.8%, ped 27.1%, car 25.3%, auto 1.8%, truck 1.2%, bus 0.5%** (+0.3% unmerged `motorcycle`) | `counts` |
| 3 | Headline | Turning movements | 146 through, 24 left, 9 right, 25 u-turn (204 traversing, 29.7%) | **169 through, 28 left, 24 right, 22 u-turn (243 traversing, 33.2%)** | `turns`, `traversing_tracks` |
| 4 | Headline | Conflicts by grade | 20 critical, 24 serious, 243 conflict, 846 minor (1,133 total) | **37 critical, 61 serious, 404 conflict, 1,270 minor (1,772 total)** | `conflicts.by_grade`, `conflicts.n_total` |
| 5 | Headline | Dominant conflict pairs | 2W↔2W 103, car↔2W 124 (93+31), car↔car 39 | **2W↔2W 175, car↔2W 173 (109+64), car↔car 51** | `conflicts.by_class_pair` |
| 6 | Headline | Worst delay | `N→SE` 24.3 s mean stopped | see caveat below — raw worst is `NE→SE` 51.3 s but **n=2 tracks**, not reliable; best-sampled worst is **`W→SE` 5.4 s (n=67)** | `delay.by_movement`, `movement_top` |
| 7 | Headline | Signal cycle | 116 s, r=0.1849 | **unchanged** (116 s, r=0.1849) — `congestion.py` is detector-free, reused not rerun | `cycle` |
| 8 | Headline | Anomalies | 48 (36 contraflow, 12 stopped) | **84 (61 contraflow, 21 stopped, 2 hard-braking)** | `anomalies` |
| 9 | Headline | Object attributes | 687/687 colour, 594/687 dims (86%), 86.6% achromatic | **731/731 colour, 675/731 dims (92.3%), 88.2% achromatic** | `objects.validation`, `objects.colour` |
| 10 | Headline | Kinematics plausible | 99.56% (3 flagged) | **99.86% (1 flagged)** | `objects.validation.kinematics_plausible_frac` |
| 11 | Headline | Map-native (OSM match) | 96.6% matched, residual 1.97 m | **81.8% matched**, residual 1.79 m (see known-limits caveat) | run_geo.py stdout / `alignment` |
| 12 | Headline | Aggregate flow/density | 2,266 veh/h/lane at 110.0 veh/km/lane | **1,503 veh/h/lane at 78.3 veh/km/lane** | `aggregate.fundamental.summary` |
| 13 | Headline | Max queue | 71.7 m (leg W) | **70.3 m (leg W)** | `aggregate.queues` |
| 14 | §3 Findings #1 (calibration) | Scale / effective height / motorcycle check | ×1.5128, 106.60 m eff. height; motorcycle 2.117 m vs 1.986 m (+6.6%) | **×1.3847, 97.57 m eff. height; motorcycle 1.845 m vs 1.986 m (−7.1%)**; observed car footprint 3.107 m vs COCO's contaminated 2.844 m (see new finding, §15) | `quality.calibration` |
| 15 | §3 Findings #3 (2W dominance) | 2W share of conflicts | ~85% of sub-3s conflicts at 45% of traffic | **2W↔2W + car↔2W = 348/404 conflict-grade (86%) at 43.8% of traffic** — same finding, stronger margin | `conflicts.by_class_pair`, `counts` |
| 16 | §4 Detection & tracking | Model / classes / tracker | YOLO11x COCO, 6 classes, plain ByteTrack | **VisDrone-yolo11s (`dronefreak/visdrone-yolov11s`, AGPL-3.0), 10 classes incl. `autorickshaw`, ByteTrack track_buffer=90/match_thresh=0.9** | `EXP1.md`, `EXP2.md`, `EXP3.md`, `config.yaml` |
| 17 | §4 Detection & tracking | Detections → raw tracks → road users | 112,144 → 1,544 → 687 | **239,960 → 1,910 → 731** | track.py stdout, `quality.tracks` |
| 18 | §4 Detection & tracking | Imputed fraction / dup tracks removed | 6.7% / 80 | **10.7% / 402** | `quality.imputed_frac`, `quality.calibration.duplicate_tracks_removed` |
| 19 | §7 Traffic insights | Median / p99 speed | 7.6 / 41.0 km/h | **3.3 / 37.3 km/h** | `quality.speed_p50_kph`, `speed_p99_kph` |
| 20 | §7 Traffic insights | Median track dwell | 8.9 s | **19.2 s** (longer `track_buffer` keeps more, longer-lived tracks) | `quality.median_track_dwell_s` |
| 21 | §7 Traffic insights | OD legs | N (18°), SE (112°), W (288°) | **NE (22.5°), SE (117.5°), W (287.5°)** — leg bearing shifts ~4-5°, a discovered-geometry artifact of the larger/denser VisDrone track population, not a site change | `legs` |
| 22 | §9.1 Measured dims | Per-body-type length table | bus 9.83 m, van/LCV 5.16 m, truck 5.09 m, SUV/MUV 4.78 m, sedan 4.26 m, hatchback 3.85 m, auto-rickshaw 2.59 m, two-wheeler 1.67 m | **bus 5.75 m (n=1, thin sample), truck 4.11 m, SUV/MUV 4.63 m, hatchback 3.64 m, sedan 4.33 m, auto-rickshaw 2.09–2.58 m, two-wheeler 1.44–1.45 m, van/LCV 5.12 m (n=3, thin)** — see caveat below | `run_attrs.py` stdout (`body_type` groupby) |
| 23 | §9.2 Colour | Achromatic share | 86.5% (dark grey 33.2, silver/grey 29.0, white 14.4, black 9.9) | **88.2%** (dark grey 35.2%, silver/grey 28.6%, white 18.1%, black 6.4%) | `objects.colour` |
| 24 | §9.3 Kinematics | Per-body-type speed/accel table | as published (buses stationary, SUV/MUV fastest) | **van/LCV now fastest median (23.5 km/h, n=3, thin sample); hatchback 17.4, sedan 16.5, two-wheeler 14.5, bus 14.1 (n=1), SUV/MUV 12.6, auto-rickshaw 12.2, truck 6.8, pedestrian 3.5** | `objects_intersection.parquet` groupby (`v_mean_kph` median, `v_max_kph` p85, `a_max_ms2`/`a_min_ms2` median) |
| 25 | §9.4 Object record | Validation counts | 687 objects, 0 >120 km/h, 99.56% (684/687) accel-plausible, median distance 16.2 m | **731 objects, 0 >120 km/h, 99.86% (730/731) accel-plausible, median distance 28.5 m** | `objects.validation` |
| 26 | §10.1 Aggregate | Full fundamental-diagram table | max flow 2,266, density@max 110.0, speed@max 20.6, max density 207.4, min speed 4.9, max occ 76.3%, lanes eq. in 5.7/out 3.33 | **max flow 1,503, density@max 78.3, speed@max 19.2, max density 159.3, min speed 4.9, max occ 33.0%, lanes eq. in 7.0/out 5.43** | `aggregate.fundamental.summary` |
| 27 | §10.2 Queues | Per-leg max/p85/mean | N 26.6/22.0/16.4, SE 44.1/32.1/29.8, W 71.7/43.7/38.1 | **NE 50.6/…, SE 63.0/…, W 70.3/…** (leg N renamed NE, see #21) — p85/mean not re-extracted this pass, max only confirmed from `aggregate.queues` | `aggregate.queues` |
| 28 | §10.3 Lane discipline | Per-leg modes/discipline/samples | N 1 mode n/a, SE 2 modes 0.75, W 3 modes 0.57 | **NE 2 modes 0.105, SE 3 modes 0.635, W 3 modes 0.498** | `aggregate.lanes[*].discipline_index` |
| 29 | §10.3 Lane modal split | Per-lane class mix | (COCO table, 6 rows) | **NE lane1 person 42.3/2W 35.1/car 13.5%; NE lane2 car 49.3/2W 36.4%; SE lane1 person 68.4%; SE lane2 2W 63.3/car 29.7%; SE lane3 2W 49.4/car 34.8%; W lane1 2W 48.1/car 32.2/person 14.9%; W lane2 car 51.9/2W 33.8/auto 12.4%; W lane3 2W 45.1/car 29.9/person 13.5%** | `aggregate.lanes[*].lanes[*].modal_split` |
| 30 | §11.3 Map-matching | Match rate / by class | 96.6%, primary 28,913/tertiary 3,758/residential 3,918 | **81.8% (167,327/204,632), primary 147,324/residential 15,837/tertiary 4,166** | run_geo.py stdout |
| 31 | §11.4 Per-lane real geometry | Top rows by vehicle count | 194/167/153/147/139/92 vehicles, 6.6–20.3 km/h mean | **193/176/156/151/142/141/116/112 vehicles, 2.2–20.7 km/h mean** (8 rows now clear the ≥5-vehicle filter) | run_geo.py stdout |
| 32 | §12 Validation | accel/speed/footprint/occlusion/dup checks | 99.96%, 0 over 100 km/h, 4.00 m car (by construction), 2.12 m vs 1.99 m motorcycle, 6.7% imputed, 80 dup | **99.86%** (quality.accel_within_4_frac — note: this is the *sample*-level check, distinct from the *object*-level 99.86% above, they coincide by chance this run), 0 over 100 km/h, 4.00 m car (by construction), **1.845 m vs 1.986 m motorcycle**, **10.7% imputed**, **402 dup** | `quality`, `objects.validation` |
| 33 | §13 Limits | Conflict fragmentation | 204/687 traversed (70% fragments) | **243/731 traversed (67% fragments)** — improved by tuning the tracker (EXP3), not eliminated; tracker re-architecture remains the fix | `traversing_tracks`, `quality.tracks` |
| 34 | §13 Limits | Class error from domain gap | current limitation | **resolved, now a finding** — see new §15 | `EXP1.md` |
| 35 | §13 Limits | New: OSM match / tracker fragmentation | not present | **added**: 81.8% match (vs 96.6% COCO), 402 dup tracks — a detection-coverage problem (denser VisDrone boxes fall outside the OSM road buffer), not fixable by tracker knobs alone (EXP3 tried 4 configs); tracker re-architecture (already priority #2) is now a prerequisite for flow/queue/match-rate numbers | `EXP2.md`, `EXP3.md` |
| 36 | Determinism | `objects.colour`/`objects.colour_hex`/`*.hex` reproducibility | non-deterministic across reruns (unseeded `np.random.randint` in `attributes.py`'s `grey_world_gains`, ~29 leaf-key diffs/rerun) | **fixed**: `np.random.default_rng(0)`, seeded. Proven: `run_attrs.py intersection` run twice → 0/687 column diffs; `run_analysis.py` (full report) run twice → 0/45,084 leaf-key diffs | `src/attributes.py:144-146` |

## Caveats carried into the prose (not just numbers)

- **Worst-delay movement (#6):** `NE→SE` at 51.3 s mean stopped is real but rests
  on only 2 tracks — a rare movement, not a robust estimate (same issue EXP2
  flagged before the tracker tuning, still present after it). The write-up
  should headline the best-sampled worst delay instead (`W→SE`, 5.4 s, n=67)
  and footnote the small-n figure rather than presenting it bare.
- **Per-body-type dimensions (#22) and kinematics (#24):** both recomputed
  cleanly — dimensions from `run_attrs.py`'s own groupby printout, kinematics
  from a one-line groupby against the fresh `objects_intersection.parquet`.
  Neither needed a new pipeline run, just re-aggregating this run's output.
- **auto-rickshaw / two-wheeler length variance (#22):** `run_attrs.py`'s
  printed table gives class-level dims (`autorickshaw` 2.09 m, `motorcycle`
  1.44 m) that differ slightly from the body-type-banded dims (`auto-rickshaw`
  2.58 m, `two-wheeler` 1.45 m) because body-type bands reclassify a few
  borderline objects by measured size, not detector class. Both are reported
  above; use body-type for the write-up table (matches the original's framing).

---

# Revision 2 — metric-frame tracker

Every figure below changed when the default **tracker** (not detector — VisDrone
stays `models/visdrone-yolov11s.pt`) moved from image-space ByteTrack
(`track.py`, `trackers/bytetrack_buf90.yaml`) to a two-stage pipeline:
`src/detect.py` (detector-only, no tracking) → `src/mtrack.py` (per-track
constant-velocity Kalman filter, association by Mahalanobis gate on
ground-plane position + class-group hard gate + footprint-size cost, in
metres not pixels). `track.py`/ByteTrack remains available as a documented
alternative (`README.md` §Pipeline). Regenerated in one clean run on
2026-09-16, tag `intersection`, same window (t=240–360s, 1200 frames),
same input detections (`out/detraw_intersection.parquet`, 269,645 rows,
scale ×1.3827, unchanged — detection is shared between both trackers).
Experiments, in order: `EXP4A.md` (decouple detection), `EXP4.md`
(metric-frame tracker, knob sweep, ADOPT), `EXP4-QA.md` (independent QA,
found two artefacts), `EXP4-FIX.md` (two targeted fixes for both).

Source files:
- **Old** = `out/exp5/report_intersection_BYTETRACK.json` (backed up before
  this run; the last ByteTrack-era `report_intersection.json` — identical to
  Revision 1's "New" column above).
- **New** = `out/report_intersection.json` (this run).
- Determinism: `src/mtrack.py --tag intersection` run twice independently →
  `out/detections_intersection.parquet` byte-for-byte identical both times
  (242,679 rows, 0 diffs, `DataFrame.equals()` true) — no unseeded randomness
  to fix.

**Two fixes bundled into this revision, both from `EXP4-FIX.md`** (see that
file for the full mechanism and the QA findings in `EXP4-QA.md` that
prescribed them):
1. **Class-group hard gate** (`mtrack.py`, `class_hard_gate=true` by
   default): a detection may only match a track in the same group —
   `{person}`, `{bicycle}`, `{motorcycle, autorickshaw}`,
   `{car, bus, truck}`. Without it, a stray pedestrian detection near a
   vehicle could still be the association solver's cheapest *legal* match
   (found in 155/678 raw confirmed tracks pre-fix).
2. **`reemerged` conflict-candidate exclusion** (`trajectories.py` +
   `conflicts.py`, `reemerge_s=2.0`): a trajectory row is flagged
   `reemerged=true` for 2.0 s after a track exits an imputed
   (occlusion-bridged) gap — the RTS-smoothed velocity right at that edge is
   the track's least trustworthy sample, and it is exactly the sample
   PET/TTC scores. `conflicts.py` now excludes `reemerged` rows from
   conflict candidacy the same way it already excludes `imputed` ones. This
   rule applies to **both** trackers identically (it lives in shared code),
   so it is folded into this revision's "new" numbers even though it
   affects trajectory data, not tracker choice. Excludes **19.9%** of
   ByteTrack's non-imputed trajectory rows and **33.0%** of the metric-frame
   tracker's (more occlusion-bridging → more re-emergence events to flag).

| # | Section | Metric | Old (ByteTrack) | New (metric-frame) | report.json key |
|---|---|---|---|---|---|
| 1 | Headline | Road users / samples | 731 / 204,632 | **602 / 253,972** | `quality.tracks`, `quality.rows` |
| 2 | Headline | Modal split (by track) | 2W 43.8%, ped 27.1%, car 25.3%, auto 1.8%, truck 1.2%, bus 0.5% | **2W 46.2%, car 29.1%, ped 19.1%, auto 3.7%, truck 1.5%, bus 0.5%** | `counts` |
| 2b | Headline | Modal split — **presence share** (fragmentation-independent: mean per-frame detection-class share, `detraw_intersection.parquet`, identical for both trackers since detection is shared) | n/a (not previously computed) | **2W 52.9%, car 23.9%, ped 17.8%, auto 3.4%, truck 1.6%, bus 0.3%** — lower for pedestrians than *either* track-based cut (see caveat below) | `out/detraw_intersection.parquet` groupby, not in report.json |
| 3 | Headline | Turning movements | 169 through, 28 left, 24 right, 22 u-turn (243 traversing, 33.2%) | **186 through, 51 left, 39 right, 29 u-turn (305 traversing, 50.7%)** | `turns`, `traversing_tracks` |
| 4 | Headline | Conflicts by grade (post-`reemerged`-filter) | 37 critical, 61 serious, 404 conflict, 1,270 minor (1,772 total) | **27 critical, 40 serious, 295 conflict, 1,001 minor (1,363 total)** | `conflicts.by_grade`, `conflicts.n_total` |
| 4b | Headline | Conflicts — isolating the filter from the tracker swap (both hold one variable fixed; see `EXP4-QA.md` check 2 / `EXP4-FIX.md`) | raw ByteTrack, no filter: 37/61/1,772 → ByteTrack **with** filter: 29/50/1,506 (filter alone) → metric-frame **with** filter: 27/40/1,363 (tracker swap on top) | raw metric-frame, no filter, pre-hard-gate was **178/157/2,558** (×4.8 vs old raw) — 81.5% of that spike touched a re-emergence within 2.0s; the filter, not a real safety change, explains nearly all of it | `EXP4-QA.md`, `EXP4-FIX.md` |
| 5 | Headline | Dominant conflict pairs | 2W↔2W 175, car↔2W 173 (109+64), car↔car 51 | **2W↔2W 162, car↔2W 128 (70+58), car↔car 25** | `conflicts.by_class_pair` |
| 6 | Headline | Worst delay | raw worst `NE→SE` 51.3s (n=2, unreliable); best-sampled `W→SE` 5.4s (n=67) | pattern changed: highest is `W→W` (u-turn) **13.3s (n=12)**; the two best-sampled *through* movements clear fast — `W→SE` 5.6s (n=77), `SE→W` 2.1s (n=109). Delay now concentrates in the low-volume turning/u-turn movements, not a single outlier. | `delay.by_movement`, `movement_top` |
| 7 | Headline | Signal cycle | 116 s, r=0.1849 | **unchanged** (detector/tracker-free, reused not rerun) | `cycle` |
| 8 | Headline | Anomalies | 84 (61 contraflow, 21 stopped, 2 hard-braking) | **75 (33 contraflow, 42 stopped, 0 hard-braking)** | `anomalies` |
| 9 | Headline | Object attributes | 731/731 colour, 675/731 dims (92.3%), 88.2% achromatic | **602/602 colour, 579/602 dims (96.2%), 89.7% achromatic** | `objects.validation`, `objects.colour` |
| 10 | Headline | Kinematics plausible | 99.86% (1 flagged) | **100.0% (0 flagged)** | `objects.validation.kinematics_plausible_frac` |
| 11 | Headline | Map-native (OSM match) | 81.8% matched, residual 1.79 m | **82.6% matched** (up slightly — same detection-coverage ceiling `EXP3.md`/`EXP4.md` already documented; tracker choice does not move this number materially) | run_geo.py stdout |
| 12 | Headline | Aggregate flow/density | 1,503 veh/h/lane at 78.3 veh/km/lane | **1,556 veh/h/lane at 75.0 veh/km/lane** | `aggregate.fundamental.summary` |
| 13 | Headline | Max queue | 70.3 m (leg W) | **67.9 m (leg W)**; NE 51.1 m, SE 47.0 m | `aggregate.queues` |
| 14 | §3/§4 Detection & tracking | Duplicate tracks removed / raw confirmed tracks | 402 (from 1,910 raw) | **180** (from 796 raw pre-dedup — dedup rate 22.6% vs 21.0% before) | `quality.calibration.duplicate_tracks_removed` |
| 15 | §3/§4 Detection & tracking | Imputed fraction | 10.7% | **22.3%** (higher — tracks now bridge occlusion continuously via the Kalman filter's growing gate instead of dying and restarting, so more rows fall inside a bridged gap; this is the mechanism behind the traversing-track gain, not a quality regression — see check 1, `EXP4-QA.md`: 0/253,972 rows over 4 m/s² accel, 0 over 100 km/h, same as before) | `quality.imputed_frac` |
| 16 | §3/§4 Detection & tracking | Calibration (scale / eff. height / motorcycle check) | ×1.3847, 97.57 m eff. height; motorcycle 1.845 m vs 1.986 m (−7.1%) | **×1.3822, 97.40 m eff. height; motorcycle 1.851 m vs 1.986 m (−6.8%)** — both self-calibrate from the same shared `detraw` file, so this is measurement noise from which detections survive each tracker's dedup, not a real calibration change | `quality.calibration` |
| 17 | §7 Traffic insights | Median / p99 speed | 3.3 / 37.3 km/h | **3.5 / 37.1 km/h** | `quality.speed_p50_kph`, `speed_p99_kph` |
| 18 | §7 Traffic insights | Median track dwell (all classes) | 19.2 s | **31.0 s** (+61% — the point of the re-architecture: fewer, longer-lived tracks through occlusion) | `quality.median_track_dwell_s` |
| 19 | §9.1 Measured dims | Per-detector-class length table | auto-rickshaw 2.09 m, bus 5.70 m (unmerged), car 3.73 m, motorcycle 1.44 m, truck 3.79 m | **auto-rickshaw 2.05 m, bus 5.70 m, car 3.73 m, motorcycle 1.45 m, truck 3.79 m** — essentially unchanged (dimensions are a size-solve on shared detections, insensitive to which tracker owns the track) | `run_attrs.py` stdout |
| 20 | §9.4 Object record | Validation counts | 731 objects, 0 >120 km/h, 99.86% accel-plausible, median distance 28.5 m | **602 objects, 0 >120 km/h, 100% accel-plausible, median distance 77.4 m** (higher median distance: longer-lived tracks spend more of their life further from the camera nadir, not a detection change) | `objects.validation` |
| 21 | §10.1 Aggregate | Full fundamental-diagram table | max flow 1,503, density@max 78.3, speed@max 19.2, max density 159.3, min speed 4.9, max occ 33.0%, lanes eq. in 7.0/out 5.43 | **max flow 1,556, density@max 75.0, speed@max 20.7, max density 170.9, min speed 5.9, max occ 34.1%, lanes eq. in 6.92/out 5.65** | `aggregate.fundamental.summary` |
| 22 | §10.2 Queues | Per-leg max | NE 50.6, SE 63.0, W 70.3 | **NE 51.1, SE 47.0, W 67.9** | `aggregate.queues` |
| 23 | §13 Limits | Conflict fragmentation | 243/731 traversed (67% fragments) | **305/602 traversed (50.7% traversing, 49.3% fragments)** — the re-architecture's headline win: `EXP4.md`'s own pre-fix number was 52.4%, the hard gate cost 1.7pts of that by freeing pedestrian detections into short-lived fragments instead of vehicle absorption (`EXP4-FIX.md`) | `traversing_tracks`, `quality.tracks` |
| 24 | §13 Limits | Tracker re-architecture (was priority #1 next step) | open | **closed, adopted** — see WRITEUP.md §16 | `EXP4.md`, `EXP4-QA.md`, `EXP4-FIX.md` |
| 25 | §13 Limits | SAHI tiling (was under evaluation, `EXP4A.md`) | open question | **closed, negative result**: 1.21× more detections for ~2.4× the compute vs tiles=1, with bus/truck tile-seam artefacts (6.66×/0.87× count ratios) — not worth adopting on top of VisDrone (which already recovers much of what tiling targeted); not revisited | `EXP4A.md` |

## Caveats carried into the prose (not just numbers)

- **Pedestrian modal split (#2) is genuinely ambiguous, not resolved by
  presence share.** The naive expectation was that a fragmentation-independent
  cut would land between the two track-based numbers (43.8%→19.1% is a big
  swing); instead presence share (17.8%) is *lower than both*. Mechanism:
  presence share weights by raw per-frame detection volume, and a handful of
  fast-transiting two-wheelers generate far more total detection-frames over
  the video than a smaller number of pedestrians, even ones that individually
  linger far longer on screen (`EXP4-QA.md` check 4: person median dwell rose
  11.9s→50.4s under the new tracker, i.e. *fewer, longer* person tracks, but
  still few in absolute track count). Track-count share and presence share
  answer different questions and neither is "more correct": quote **track
  share** for "what fraction of distinct road users were pedestrians"
  (mode-share, safety-per-user-type), quote **presence share** for "how much
  of the scene at any instant is pedestrians" (roadway occupancy/space
  allocation). Separately and unresolved: `EXP4-FIX.md`'s hard gate stops
  155/678 raw tracks' pedestrian-into-vehicle class leak but cannot recover
  pedestrian tracks the leak-fixed detections are too short-lived to confirm
  on their own — closing that gap needs a detection-recall fix, not a
  tracking-side one.
- **Conflict counts (#4/#4b) need both fixes read together, not the raw
  numbers from either tracker alone.** The metric-frame tracker's raw
  critical/serious count is 4.8x the ByteTrack raw count (178 vs 37) purely
  because continuous occlusion-bridging produces far more re-emergence
  events, each one a spurious near-miss at the RTS smoother's least
  trustworthy sample. Applying the same 2.0s re-emergence exclusion to both
  trackers (`EXP4-QA.md` check 2/3) shows the underlying conflict rate is
  statistically indistinguishable between them (0.378 vs 0.363 critical per
  1,000 non-imputed moving samples) — the real, publishable finding is
  "same safety signal, cleaner tracker," not "tracking swap found 5x more
  danger."
- **Worst-delay movement (#6) no longer has one extreme outlier to caveat.**
  Under the old tracker `NE→SE`'s 51.3s rested on n=2 tracks and had to be
  footnoted as unreliable. Under the new tracker the delay range across all
  9 movement pairs is 2.1–13.3s — high delay now sits with the low-volume
  turning/u-turn movements (plausible: fewer through-lanes to use while
  turning) rather than one thin-sampled pair, so no single number needs a
  reliability caveat this revision.

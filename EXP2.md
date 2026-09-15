# EXP2 — full re-track with VisDrone-yolo11s vs the current COCO run

Run 2026-09-15 by temp `worker-fh-retrack`, branch `visdrone`. Full pipeline (not
the 20-frame sample EXP1 used) on the same 120 s intersection window (t=240-360,
10 fps), same window cut (`out/window_intersection.mp4`, hard-linked as
`out/window_visdrone.mp4` — no re-cut), tag `visdrone` throughout so nothing under
`out/*_intersection.*` was overwritten. Weights: `models/visdrone-yolov11s.pt`
(EXP1 pick #1). Total wall time ≈ 8.5 min (track.py 90 s is the only real GPU
step; everything else is CPU/deterministic), inside the 15 GPU-minute budget.

## Code changes (branch `visdrone` only, COCO path unchanged by default)

- `src/track.py`: auto-selects a VisDrone 10-class keep-list and a
  `VISDRONE_MAP` (`pedestrian/people→person`, `bicycle→bicycle`, `car/van→car`,
  `truck→truck`, `bus→bus`, `motor→motorcycle`, `tricycle/awning-tricycle→`
  **`autorickshaw`** (new class, see below)) whenever `--weights` contains
  `visdrone`; the map is applied once, at detection time. Added `--window` to
  reuse an existing cut instead of re-cutting 4K video.
- New `autorickshaw` class wired with one line each in `src/trajectories.py`
  (`CLASS_HEIGHT`, `PCU`) and `src/conflicts.py` (`RADIUS`); `src/attributes.py`
  needed **no change** — its `BODY_BANDS` already had an `auto-rickshaw`
  size band (dimension-derived, detector-agnostic), and `src/objects.py`,
  `src/insights.py`, `src/aggregate.py` are all `groupby("cls")` generic.
  Two cosmetic fallback-colour entries added in `src/overlay.py`.
- `run_attrs.py`, `run_geo.py`: hard-coded `"intersection"` tag replaced with a
  positional tag argument (`python run_attrs.py visdrone`), matching
  `build_dashboard.py`'s existing style. `src/run_analysis.py` and
  `src/overlay.py` already took `--tag`.
- `config.yaml`: added a documentation-only `detector_visdrone` block next to
  `detector:` (nothing in this repo actually parses config.yaml at runtime —
  every script's CLI defaults just mirror it, same as before this change).

PCU for `autorickshaw` (1.2) is an IRC-106-style estimate, not fit to this
data — flag if it matters; it affects only the PCU-weighted flow column, not
vehicle counts or conflicts.

## Headline comparison

| metric | COCO (current, `intersection`) | VisDrone-yolo11s (`visdrone`) |
|---|---|---|
| Road users / samples | 687 tracks / 113,620 rows | **909 tracks / 187,977 rows** |
| Modal split | 2W 45.1%, car 31.4%, ped 19.5%, bus 2.6%, truck 1.3% | 2W 46.2%, **ped 29.5%**, car 20.5%, auto-rickshaw 1.7%, truck 1.4%, bus 0.4% |
| Turning movements | 146 through, 24 left, 9 right, 25 u-turn (204 traversing) | 164 through, 27 left, 19 right, 28 u-turn (238 traversing) |
| Conflicts by grade | 20 critical, 24 serious, 243 conflict, 846 minor (1,133 total) | **29 critical, 57 serious**, 368 conflict, 1,206 minor (1,660 total) |
| Dominant conflict pairs | 2W↔2W 103, car↔2W 93+31=124, car↔car 39 | 2W↔2W 165, car↔2W 107+54=161, car↔car 36 |
| Worst delay | `N→SE` 24.3 s mean stopped | `NE→SE` **95.6 s** mean stopped (same physical leg pair; see caveats) |
| Signal cycle | 116 s (r=0.18) | 116 s (r=0.18) — re-run, detector-free, see below |
| Anomalies | 48 (36 contraflow, 12 stopped) | 95 (76 contraflow, 19 stopped) |
| Attribute coverage | 687/687 colour, 594/687 dims (86%) | 909/909 colour, 834/909 dims (92%) |
| Kinematics plausible | 99.56% (3 flagged) | 99.12% (8 flagged) |
| OSM match | **96.6%**, residual 1.97 m | **82.7%**, residual 1.76 m |
| Aggregate flow/density | 2,266 veh/h/lane at 110 veh/km/lane | 1,521 veh/h/lane at 79.6 veh/km/lane |
| Max queue | 71.7 m | 69.8 m |
| Track fragmentation | 204/687 traversing (29.7%), 80 dup tracks removed | 238/909 traversing (26.2%), **541 dup tracks removed** |
| Calibration (independent check) | scale ×1.513, effective height 106.6 m; **motorcycle 2.117 m vs 1.986 m expected** (+6.6%) | scale ×1.385, effective height 97.6 m; **motorcycle 1.843 m vs 1.986 m expected** (−7.2%) |
| Calibration input | observed car footprint 2.844 m (contaminated, see obs. 1) | observed car footprint 3.107 m |

Flow/density/queue and delay all derive from Edie's generalised measures over
the *same* map-matched trajectories, so the drop there is a direct consequence
of the OSM-match-rate drop, not an independent regression.

## Three observations

1. **COCO's own scale self-calibration was quietly contaminated by the same
   bug this whole experiment is about.** `calibrate()` in `trajectories.py`
   runs on raw pre-merge `cls` labels, *before* `merge_two_wheelers()` folds
   misclassified two-wheelers back out of the `car` bucket. COCO's `car`
   population is therefore full of two-wheelers-called-cars, pulling the
   observed median car footprint down to 2.84 m and forcing a large ×1.513
   correction. VisDrone's clean `car` class observes 3.11 m directly, needing
   only ×1.385. Both pass the independent motorcycle-length check within
   ~7%, so neither run is "wrong", but VisDrone's calibration input is
   provably less contaminated — a second-order fix nobody asked for.

2. **Conflict counts and pedestrian counts both jump — for two different,
   legitimate reasons, not one artifact.** Total conflicts rise 1,133→1,660
   (+47%) and critical 20→29, serious 24→57 (+137%), tracking the two-wheeler
   share increase and the extra 222 tracked road users. Pedestrians roughly
   double, 134→268 (19.5%→29.5% share) — this matches the README's own known
   limit ("pedestrian counts are a floor... COCO undercounts"), now confirmed
   without needing SAHI tiling: VisDrone alone finds most of what tiling was
   flagged as necessary for.

3. **The tracker is now the weaker link.** Duplicate-ID pairs removed jumped
   80→541 and OSM match rate fell 96.6%→82.7%, even though the *per-sample*
   registration residual is actually slightly better (1.76 m vs 1.97 m) —
   meaning the drop is about which samples get matched at all, not how well.
   VisDrone's ~2× denser per-frame detection field (confirmed in EXP1) gives
   plain-IoU ByteTrack more boxes to confuse frame to frame, and more
   detections in places (verges, parked two-wheelers, pavement) the OSM road
   network's 20 m buffer doesn't reach. This is exactly the README's
   already-flagged next-step #2 ("re-architect the tracker... decouple
   detection from tracking, associate in the metric frame") — VisDrone makes
   it more urgent, not less relevant.

## What broke / needs a caveat

- **Pipeline-order bug (fixed here, not in `src/`):** `run_analysis.py`
  conditionally reads `attributes_{tag}.parquet` only *if it already exists*,
  but the README's documented run order puts `run_analysis.py` before
  `run_attrs.py`. For a brand-new tag this silently produces `objects_*.parquet`
  and `report_*.json` with **zero** colour/body-type/dimension coverage — it
  happened here first (attrs coverage briefly read 0/909) and was masked in
  the existing `intersection` outputs only because `attributes_intersection.parquet`
  already existed on disk from an earlier session. Fixed by re-running
  `run_analysis.py` after `run_attrs.py` (both harmless, deterministic, ~1 min
  each). **Recommend swapping the documented order**, or having `run_analysis.py`
  call attrs itself, before this bites the next fresh tag.
- **Shared (non-tag-suffixed) output paths in `run_geo.py`:** `out/geojson/*.geojson`
  and `out/geo_shift.npy` are written without a tag suffix, so running
  `run_geo.py visdrone` silently overwrote the `intersection` tag's map
  exports. Restored by re-running `python run_geo.py intersection` afterward
  — verified it reproduces the README's exact 113,620 matched samples. No
  git-tracked file was touched (`out/` is fully gitignored), but a future run
  against >1 tag should tag-suffix those paths.
- **`congestion.py` (signal-cycle detection) wasn't in the dispatch's run
  list.** It turned out to be fully detector-free (raw ffmpeg frame-diff over
  the whole 399 s recording, no detections/trajectories involved at all), so
  I ran it for `visdrone` too — it reproduced `116.0 s / r=0.1849` exactly,
  confirming it is provably tag-independent, not a numeric coincidence.
- **Worst-delay movement `NE→SE` at 95.6 s (vs `N→SE` 24.3 s) deserves a
  caveat, not a red flag.** Leg-bearing discovery re-clusters slightly
  differently with the larger, denser VisDrone track population (a leg
  labelled `N` before is now `NE`), and rare movements have small sample
  counts — this could be one or two long-occluded tracks dominating a thin
  mean, not a real 4× delay increase. Worth checking against `n` per movement
  before quoting 95.6 s anywhere.
- **Visual QA (2 frames, `out/exp2/frame_10s.jpg`, `frame_40s.jpg`, extracted
  from `out/overlay_visdrone.mp4`):** boxes and labels look correct. Rows of
  two-wheelers along the road get tight small boxes labelled `2W <length>m`,
  distinct from `car`/`sedan`/`hatch` boxes on actual cars: no more
  wall-to-wall car boxes over two-wheeler rows. Auto-rickshaw-shaped tracks
  get their own `auto <length>m` label and cyan colour from the measured
  size, e.g. near the roundabout. Colour chips, map ribbons and the aggregate
  overlay all render normally — nothing looks broken.

## Recommendation: **adopt with caveats**

The class fix is real and decisive at full-pipeline scale, not just in EXP1's
20-frame sample: it changes the modal split, the PCU-weighted flow numbers,
which conflict pairs dominate, and — unexpectedly — makes the geometric
self-calibration *more* trustworthy (observation 1). This is worth adopting.

Caveats to resolve before replacing the submitted numbers:
1. Fix the `run_analysis.py`/`run_attrs.py` ordering and the shared-path bug
   in `run_geo.py` first — both are latent in the existing pipeline, not
   specific to VisDrone, and will bite again.
2. The OSM match-rate drop (96.6%→82.7%) and the 541 duplicate tracks say the
   README's own next-step #2 (tracker re-architecture) has become a
   prerequisite, not a nice-to-have, if VisDrone is adopted — flow/density/
   queue numbers are now downstream of a lower match rate.
3. Sanity-check the 95.6 s worst-delay figure against its sample count before
   quoting it anywhere public.
4. If adopted, README/WRITEUP headline numbers, the dashboard and `demo/`
   need regenerating from the `visdrone` tag — out of scope here per the
   dispatch boundary (no `demo/`/`README.md`/`WRITEUP.md` edits on this
   branch).

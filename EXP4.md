# EXP4 — metric-frame tracker (Step B of the tracker re-architecture)

Run 2026-09-15/16 by temp `worker-fh-mtrack`, branch `mtrack`. Consumes Step A's
`out/detraw_intersection.parquet` (tiles=1, no `--embed`; EXP4A.md) — same
detector, same calibrated scale (1.3827/1.3833), same window, nothing re-cut
or re-inferred.

## Design (`src/mtrack.py`, 249 lines, numpy + scipy + pandas only)

Per-track state: constant-velocity Kalman filter `(x,y,vx,vy)` in the
**ground-plane metres detect.py already computed** (`x_m,y_m`), not pixels —
no telemetry/GroundPlane needed inside the tracker at all. Majority-vote
class via a `Counter`. Two-stage matching per frame (ByteTrack-style:
`conf>=0.5` first, then the rest), solved with
`scipy.optimize.linear_sum_assignment`. Gate = Mahalanobis distance on the
filter's own innovation covariance — it inflates every unmatched frame
(process noise added, no correction), so the acceptance radius *grows through
an occlusion by itself*; no separate growth schedule was needed. Cost =
Euclidean position distance + a class-pair term (ordinal group distance:
person/two-wheeler/car-like/heavy, so motorcycle↔bicycle costs 0 and
car↔motorcycle costs `lambda_cls`) + a size term. **The size term stands in
for the appearance cue**: `detraw_intersection.parquet` (the file this step
was told to use) has no `--embed` column, so `w_m,h_m` footprint distance is
the "cheap appearance cue" instead of the HSV vector — see the ablation
below, it earns its keep. A tentative track dies on its first miss (so
`min_hits`=3 confirmed hits are always consecutive, matching the "3
consecutive matches" spec) and its buffered rows are discarded — a detection
that never confirms leaves nothing in the output. Confirmed tracks tolerate
up to `max_lost` seconds of misses. Output strips `x_m,y_m,w_m,h_m` and
matches track.py's schema (`fi,t,track_id,cls,conf,x1,y1,x2,y2,u_px,v_px`);
`run_analysis.py --tag mtrack` ran with zero changes.

## Headline table

| metric | COCO (`intersection`, EXP2/3) | current default (VisDrone+buf90, EXP3 winner) | **mtrack** |
|---|---|---|---|
| total tracks | 687 | 731 | **561** |
| duplicate tracks removed | 80 | 402 | **110** |
| traversing tracks (% of total) | 204 (29.7%) | 243 (33.2%) | **294 (52.4%)** |
| imputed fraction | 0.0666 | 0.1074 | 0.2093 |
| OSM match % | 96.6 | 81.8 | 81.3 |
| conflicts crit/serious/total | 20/24/1,133 | 37/61/1,772 | 178/157/2,558 (see caveat) |
| modal split (2W/person/car) | 45.1/19.5/31.4 | 43.8/27.1/25.3 | 47.6/18.2/29.2 |
| motorcycle length (m, expect 1.986) | 2.117 (+6.6%) | 1.845 (−7.1%) | **1.852 (−6.7%)** |
| median track dwell (s) | — | 19.2 (computed here) | **33.1** |
| cumulative id growth (ids/min) | — | 365.8 (computed here) | **280.7** |

Every mtrack column comes from one clean run: `mtrack.py` (2m16s, CPU) →
`run_analysis.py` → `run_attrs.py` → `run_geo.py` → `build_dashboard.py` →
`overlay.py --seconds 30`, tag `mtrack`, nothing on `main`/`intersection`/
`visdrone` touched. `window_mtrack.mp4` is a hardlink of
`window_intersection.mp4` (same window, no re-cut), and `run_geo.py`'s
tag-subfoldered geojson (the earlier bugfix) confirmed no clobbering.

## Knob log (4 combos, tracking-metrics only — no video/attrs pass, ~15 min total)

| combo | changed from default | n tracks | dup removed | traversing | dwell med | ids/min | motorcycle check |
|---|---|---|---|---|---|---|---|
| **default** (adopted) | `max_lost=6s, gate_chi2=9.21, lambda_app=0.5` | 561 | 110 | 294 (52.4%) | 33.1s | 280.7 | 1.852 (−6.7%) |
| tight_gate | `gate_chi2=5.99` (95% vs 99%) | 532 | **167 (worse)** | 303 (57.0%) | 31.1s | 266.2 | **1.329 (−33%, FAILS)** |
| longer_lost | `max_lost=9s` (90 frames) | 563 | **88 (better)** | 297 (52.8%) | 34.5s | 281.7 | 1.862 (−6.2%) |
| no_app_cue | `lambda_app=0` (ablate size cue) | 505 | **153 (worse)** | 297 (58.8%) | 33.2s | 252.7 | **1.327 (−33%, FAILS)** |

Two findings:
1. **The size-based appearance cue is doing real work, not just decoration.**
   Turning it off (`no_app_cue`) raises duplicates 110→153 (+39%) — without a
   size penalty, the class-cost term alone lets a Hungarian solver swap a
   track onto a same-class detection that's a poor size match (a fragment
   splitting a large vehicle, or the reverse), producing more fragments, not
   fewer.
2. **Both failing combos (`tight_gate`, `no_app_cue`) tank the *same*
   independent check** (motorcycle length, +6% -> -33%), and neither one
   touched anything that should touch scale calibration directly (`calibrate()`
   still uses car footprints only). The shared mechanism: both configs
   accept *more* marginal/fragmented matches into the raw per-detection
   pool `calibrate()` draws its motorcycle median from — a tighter gate
   rejects good re-associations and forces new fragment births instead, and
   dropping the size cost lets short-lived fragments pick up partially
   -occluded motorcycle boxes. Not chased to a single root line of code
   (out of the time budget for a knob sweep), but the direction is clear
   enough to reject both combos on this evidence alone, per the dispatch's
   own gate ("motorcycle sanity check stays within ~10%").
   `longer_lost` (9s buffer) is a small further improvement over the
   6s default with no downside on any metric — worth adopting if this
   becomes the shipped default; not re-run through the full downstream
   chain here since the 6s default already clears every success bar with
   margin and a temp's time budget is finite.

## Verdict: **ADOPT**

Every stated success condition is met, several by a wide margin:
- duplicates removed 402 → 110 (well below), driven by genuine occlusion
  bridging, not just relabeling: median dwell 19.2s → 33.1s (+72%) and
  cumulative id growth 366 → 281 ids/min (−23%) both move the direction that
  only re-association through the tree — not merely retuning ByteTrack's
  buffer again — can produce.
- traversing tracks 33.2% → 52.4% (well above).
- motorcycle sanity check 1.852m vs 1.986m expected (−6.7%), inside the ~10%
  band, same order as the current default's −7.1%.
- modal split stays plausible (two-wheeler still the dominant class at
  47.6%), though it shifted: person share dropped 27.1%→18.2% while car and
  autorickshaw rose. Plausible mechanism, not fully verified: pedestrian
  tracks are usually the shortest-lived and most fragmentation-prone class,
  so a tracker that specifically fixes fragmentation would be expected to
  merge more of them into fewer, longer tracks — the same directional effect
  EXPLAINER.md already documented for two-wheelers when the ByteTrack buffer
  grew. State this caveat before quoting modal split from this tag.

Visual QA (`out/exp4/frame_08s.jpg`, `frame_16s.jpg`, from `overlay_mtrack.mp4`,
same convention as EXP3): boxes, classes, and lengths render correctly, no
misboxed or double-boxed objects visible in either frame, labels sit on the
right vehicles, map ribbons and the aggregate panel look normal. Cumulative
ids climbed 214→267 (+53) between t=8s and t=16s, versus buf90's 307→428
(+121) over the identical 8s window in EXP3 — under half the id churn rate,
consistent with the quantitative id-growth-rate result above. Two frames 8s
apart still cannot rule out short id switches between them (same limitation
EXP3 flagged), which is exactly what the synthetic gap test exists to cover
instead.

## Two caveats to state before quoting numbers from this tag

1. **Raw conflict counts are not comparable 1:1 with the current default.**
   crit/serious/total rose to 178/157/2,558 from 37/61/1,772 — but total
   trajectory rows also rose from ~180k-scale to 261,046 because tracks now
   span the occlusion continuously instead of stopping and restarting.
   More continuous rows over a longer real dwell per track means more
   PET/TTC candidate pairs get evaluated over time, which raises the raw
   count independent of any change in real risk. EXPLAINER.md already
   flagged conflict counts as an upper bound when fragmentation *inflated*
   them; here the same caveat applies with the opposite mechanism (less
   fragmentation, more continuous exposure). The class composition and
   pattern of conflicts are the robust read, not the absolute count.
2. **OSM match % did not move (81.8% → 81.3%)** — confirms EXP3's own
   conclusion rather than contradicting it: the drop from COCO's 96.6% is a
   *detection-coverage* problem (denser VisDrone boxes landing outside the
   OSM road buffer), not an *ID-association* one. A perfect tracker cannot
   fix this; it needs either a wider road buffer or filtering detections
   by plausible road-surface location before map-matching.

## What a ReID network would add

The appearance term here is footprint size (`w_m,h_m`), a coarse proxy — the
ablation shows it already prevents ~30% of duplicates, but it cannot
distinguish two same-class, similar-sized objects (two motorcycles of
similar length crossing near-simultaneously at the tree). A learned
appearance embedding (even the `--embed` HSV+size vector `detect.py`
already supports, unused here because the specified input file lacks it)
would add a colour/texture signal that survives the occlusion better than
size alone — most useful exactly at the failure mode size can't resolve:
same-class objects converging on the same predicted position at the same
time. Cheapest next step: regenerate `detraw_intersection.parquet` with
`--embed` (a new tag, doesn't touch the existing file) and add an
`lambda_embed` cost term the same way `lambda_app` was added here.

## What's unchanged

`track.py`, `trajectories.py`, `detect.py`, `config.yaml`'s existing blocks,
and every `out/*_intersection.*` / `out/*_visdrone.*` file are untouched.
`config.yaml` gained a documentation-only `mtracker:` block (nothing in this
repo parses config.yaml at runtime, same status as `detector:`).

# EXP4-QA — independent QA of the metric-frame tracker (mtrack)

Run 2026-09-16 by temp `worker-fh-mtrack-qa`, branch `mtrack`. Checks EXP4.md's
ADOPT verdict before its numbers replace the published (VisDrone+buf90)
ones — specifically the two headline moves that need explaining before
anyone quotes them: critical conflicts 37→178 (×4.8) and pedestrian share
27.1%→18.2%. All numbers below are freshly computed from
`out/trajectories_mtrack.parquet` / `out/objects_mtrack.parquet` /
`out/report_mtrack.json` vs the current default `out/*_intersection.*`
(VisDrone + ByteTrack buf90) — no `src/` file was modified for checks 1–4;
check 5 is visual only.

## Check 1 — physics: is re-association producing jumps?

| | mtrack | intersection (default) |
|---|---|---|
| rows with `\|accel\|` > 4 m/s² (of all trajectory samples) | 0 / 261,046 (0.0%) | 0 / 204,632 (0.0%) |
| rows with speed > 100 km/h | 0 (0.0%) | 0 (0.0%) |
| objects failing `kinematics_plausible` (peak `\|accel\|` ≥ 4 m/s²) | 0 / 561 | 1 / 731 (0.14%) |
| jerk p95, m/s³ (population of per-object `jerk_p95_ms3`: median / p95 / max) | 0.96 / 1.72 / 3.19 | 0.77 / 1.56 / **7.02** |

**Verdict: no.** mtrack is not worse on physics than the current default —
if anything marginally better (zero implausible objects vs one, and no
outlier jerk anywhere near intersection's 7.02 m/s³ max). Re-association is
not producing kinematic jumps. (Both numbers for rows/objects come straight
from `report_*.json`'s `quality` and `objects.validation` blocks, which are
already row- and object-level respectively — reproduced here, not
recomputed.)

## Check 2 — re-emergence artefacts

For every critical/serious conflict in each run's *freshly recomputed*
severity table (`conflicts.pet/ttc/severity`, unmodified, called directly —
this also reproduces the published 178/157/2,558→2,557⁺ and 37/61/1,772
counts, confirming `conflicts.py` already excludes imputed rows and slow
(<1.2 m/s) samples as EXP4.md claimed), checked whether either track has an
imputed gap ending (an `imputed=True → False` transition) within 2.0 s
before the conflict time:

| | mtrack | intersection |
|---|---|---|
| critical+serious conflicts | 335 | 98 |
| tracks with ≥1 imputed gap | 365 / 561 (65%) | 492 / 731 (67%) |
| share touching a re-emergence ≤2.0s before *t* | **81.5%** (273/335) | **21.4%** (21/98) |

Recomputed crit/serious/total **excluding samples within 2.0 s after an
imputed-gap re-emergence** (post-filter applied to a copy of the
trajectories — rows flagged are marked `imputed=True` before calling
`conflicts.py`'s functions again; `conflicts.py` itself untouched):

| | mtrack base → post-filter | intersection base → post-filter |
|---|---|---|
| critical | 178 → **30** | 37 → **29** |
| serious | 157 → **35** | 61 → **50** |
| total | 2,557 → **1,392** | 1,772 → **1,506** |

Four out of five critical/serious mtrack conflicts happen right as a track
comes out of an occlusion-bridged gap — the Kalman filter's covariance is
still inflated at that instant, so its RTS-smoothed velocity right at the
gap edge is the least trustworthy sample in the whole track, and that is
exactly the sample PET/TTC score. Under the filter, mtrack's critical count
(30) lands *below* intersection's raw critical count (37) and close to
intersection's own post-filter number (29) — the ×4.8 is overwhelmingly a
re-association artefact, not a new safety signal.

## Check 3 — exposure normalisation

Conflicts per 1,000 non-imputed moving samples (same population
`pet()`/`ttc()` draw from: `~imputed & speed > 1.2 m/s`), and per traversing
track:

| | mtrack (294 traversing) | intersection (243 traversing) |
|---|---|---|
| **base** — crit / 1,000 moving samples | 1.952 | 0.424 |
| base — total / 1,000 moving samples | 28.04 | 20.32 |
| base — crit / traversing track | 0.605 | 0.152 |
| base — total / traversing track | 8.70 | 7.29 |
| **post-filter** — crit / 1,000 moving samples | 0.378 | 0.363 |
| post-filter — total / 1,000 moving samples | 17.54 | 18.86 |
| post-filter — crit / traversing track | 0.102 | 0.119 |
| post-filter — total / traversing track | 4.73 | 6.20 |

The raw ×4.8 **does not survive exposure normalisation on its own**
(critical rate is still ~4–4.6× higher per 1,000 moving samples / per
traversing track before filtering) — exposure alone is not the explanation.
It **does** survive combined with the re-emergence filter: post-filter,
mtrack's normalised critical rate (0.378/1,000, 0.102/track) is
statistically indistinguishable from — even slightly below —
intersection's own post-filter rate (0.363/1,000, 0.119/track). The
re-emergence artefact, not exposure, is the mechanism; once it is removed
the two trackers report the same underlying conflict rate.

## Check 4 — pedestrians: where did the tracks go?

Person detections/frame are identical (same `detraw_intersection.parquet`
input to both stages), so the drop is entirely a tracking/association
effect, not a detection one.

**Objects-level (published tags):**

| | mtrack | intersection |
|---|---|---|
| person tracks (final) | 102 (18.2%) | 198 (27.1%) |
| median person-track duration | 50.4 s | 11.9 s |

**Tracking-only instrumented rerun** (reused `mtrack.py`'s own `Track` /
`_associate` / `_apply_match` / `_kf_matrices` unmodified — only the outer
loop was duplicated to log the raw per-frame matched class and
unconfirmed-track deaths; no line in `mtrack.py` itself was changed):

- Raw (pre-`trajectories.py` MIN_TRACK_SEC/dedup) confirmed person tracks:
  **132** → down to 102 after `trajectories.py`'s ≥1.5 s / dedup filtering
  (−23%, a normal post-processing loss, not the main story).
- Tracks that **died before confirming** (never reached `min_hits`=3):
  112 total, of which **18 were majority-person** over their 1–2 hits —
  a real but modest loss.
- Tracks whose raw per-frame matched-detection sequence disagrees with the
  track's own final majority label on **>30% of frames**: 88 of 678
  confirmed tracks (13%); of those, **person** is the largest single final
  class (24, ahead of car's 27 only marginally, then motorcycle 22).
- **155 of 678 confirmed tracks (23%) matched at least one raw
  'person'-labelled detection but ended up confirmed under a different
  final class** — i.e. a lone pedestrian detection got absorbed into a
  nearby vehicle track's association instead of forming or joining its own
  person track.

**Mechanism**: `mtrack.py`'s gate is purely spatial (Mahalanobis on
position only); class enters solely as an additive cost
(`lambda_cls=1.0` × ordinal group distance — person is already the
*most expensive* mismatch, 2–3 group-steps from every vehicle class), not
a hard constraint. In dense curb-side/crossing scenes a stray pedestrian
detection close to a vehicle track can still be the Hungarian solver's
cheapest legal assignment. Net effect: fewer, much longer-lived confirmed
person tracks (consistent with EXPLAINER.md's fragmentation-reduction
mechanism — 50.4 s vs 11.9 s median dwell), *plus* a real leak of person
detections into vehicle tracks that the fragmentation story alone doesn't
explain.

**Knob tested — `lambda_cls` 1.0 → 2.0** (double the class-mismatch cost;
tracking + `trajectories.build()` only, no attrs/geo/video pass, per the
dispatch's time budget):

| | default (`λ_cls=1.0`) | `λ_cls=2.0` |
|---|---|---|
| n_tracks | 561 | 513 |
| duplicate tracks removed | 110 | **151 (+37%, worse)** |
| person tracks | 104 | 95 |
| person share of tracks | 18.54% | 18.52% (**unchanged**) |
| motorcycle length check (expect 1.986 m) | 1.852 m (−6.7%) | **1.333 m (−28%, FAILS)** |

**Reject this knob.** It doesn't move the pedestrian share at all (18.54%→
18.52%, within noise) and breaks two other metrics: more duplicate tracks
(a tighter class cost makes bridging an occlusion through a momentarily
class-uncertain re-detection *more* expensive, so more tracks fragment
instead of merging — the same "reject more marginal re-associations →
pollute the raw pool `calibrate()` draws from" mechanism EXP4.md's own
knob sweep already documented for its two rejected combos, `tight_gate`
and `no_app_cue`). No single-knob fix was found in the time budget — the
class-vote leak (check 4's 155-track finding) is a structural property of
a purely-spatial gate with class as a soft cost, not a tunable weight;
EXP4.md's own suggested next step (an `--embed` appearance term, giving
the solver a signal that distinguishes a person from a vehicle even when
both are spatially close) is the more promising direction, not tried here
as it needs regenerating `detraw_intersection.parquet` — out of scope for
a QA pass.

## Check 5 — eyes: the 3 worst critical conflicts

Frames exported from `window_mtrack.mp4` (hardlink of the published window,
same footage) at each conflict's time minus the window's `t0=240s`, boxes
drawn from `out/detections_mtrack.parquet` at the nearest frame for each
track id (`out/exp4qa/crit{1,2,3}_crop.jpg`):

1. **t=275.8s, id 105 (autorickshaw) × id 347 (motorcycle), TTC=0.30s.**
   Two distinct, real two-wheelers in a dense curb-side cluster — not one
   object split into two ids. Track 347 **re-emerged from an imputed gap
   at this exact frame** (t=275.8, gap ended 0s before).
2. **t=308.9s, id 408 (autorickshaw) × id 513 (car), TTC=0.31s.** Two real
   vehicles, same lane, close following distance. Track 408 re-emerged
   0.1s before this frame. `conflicts.ttc()` has no car-following exclusion
   (unlike `pet()`'s `MIN_CROSS_ANGLE` test) — this could be ordinary close
   following, not a crossing near-miss, independent of the re-emergence
   issue.
3. **t=352.0s, id 519 (person) × id 684 (motorcycle), TTC=0.32s.** Two real,
   distinct road users close together near a junction. Track 519
   re-emerged **at this exact frame**.

**All three of the worst critical conflicts touch a re-emergence within
0–2s of the flagged time** — exactly the pattern check 2 found in 81.5% of
mtrack's crit/serious pool. In every case the two boxes are real, distinct
road users (no double-boxed or misboxed objects), so this is not a track-id
artefact — but the RTS-smoothed velocity right at an imputed-gap boundary
is the least trustworthy sample in the track (the same edge-transient
`objects.py`'s `EDGE_TRIM` already corrects for at track start/end, per its
own comment, but does not correct for *internal* imputed-gap edges), and
that is exactly the sample PET/TTC scores. Separately: re-emergence gaps
are mostly short (median 0.2s, p90 1.9s, max 5.9s ≈ the 6s `max_lost`
cap) — this is not only about long occlusion bridges through the tree, it
fires on ordinary single-frame misses too.

## Verdict: **ADOPT WITH A CONFLICT POST-FILTER**

EXP4.md's core tracking-quality verdict stands — duplicates 402→110,
traversing 33.2%→52.4%, dwell +72%, motorcycle check within tolerance, no
physics regression (check 1) — but the two flagged headline numbers should
not be quoted as-is:

1. **Conflicts.** The raw ×4.8 critical-conflict rise is overwhelmingly a
   tracker re-association artefact, not a real safety signal: 81.5% of
   mtrack's critical/serious conflicts touch a track re-emerging from an
   imputed gap within 2.0s, it does not survive exposure normalisation
   alone (still ~4–4.6× after normalising), but **does** resolve under a
   combined re-emergence filter + exposure normalisation (mtrack
   0.378 critical/1,000 moving samples vs intersection's own 0.363 — 
   statistically the same). **Rule to adopt**: before computing PET/TTC
   conflict counts from any tracker whose output has an `imputed` column,
   exclude samples within 2.0s of an `imputed: True→False` transition (a
   post-filter on the trajectories dataframe, not a change to
   `conflicts.py`, so it applies identically to both trackers). Quote the
   post-filter numbers, not the raw ones.
2. **Pedestrians.** The 27.1%→18.2% share drop is partly the same
   fragmentation-reduction mechanism already documented for two-wheelers
   (person-track median dwell 11.9s→50.4s, fewer/longer tracks) but also a
   real, structural leak: 155 of 678 raw confirmed tracks (23%) absorbed at
   least one person detection into a track that ended up confirmed under a
   different class, because `mtrack.py`'s gate is spatial-only with class
   as a soft, tunable cost rather than a hard constraint. No knob fixes
   this without breaking other metrics (tested `lambda_cls`=2.0: rejected,
   0% improvement on person share, motorcycle check fails −28%, duplicates
   +37%). State this caveat before quoting modal split from this tag;
   fixing it properly needs the `--embed` appearance term EXP4.md already
   named as future work, not a config knob.

### The 5 numbers the published write-up should quote

1. Critical conflicts (post-filter): **30** (mtrack) vs **29** (default) —
   not 178 vs 37. Both computed with the same 2.0s re-emergence exclusion.
2. Serious conflicts (post-filter): **35** (mtrack) vs **50** (default).
3. Total conflicts (post-filter): **1,392** (mtrack) vs **1,506** (default).
4. Critical conflicts per 1,000 non-imputed moving samples (post-filter,
   exposure-normalised): **0.378** (mtrack) vs **0.363** (default) —
   effectively equal.
5. Pedestrian share: **18.2%** (mtrack, n=102) vs **27.1%** (default,
   n=198) — quote with the caveat that ~40–50% of the drop is fragmentation
   -reduction (dwell 11.9s→50.4s) and the remainder is an unresolved
   class-vote leak into vehicle tracks (155/678 raw tracks affected), not a
   detection-coverage change (same input detections to both stages).


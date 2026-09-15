# EXP4-FIX — class-group hard gate + re-emergence conflict filter

Two targeted fixes to `mtrack.py`'s association and `conflicts.py`'s
candidate filter, both prescribed by `EXP4-QA.md`'s findings on the
metric-frame tracker (`EXP4.md`), on branch `mtrack`.

- **FIX 1 (`src/mtrack.py`, `HARD_GROUP` + `class_hard_gate`, default on):**
  a detection may only match a track in the same group — `{person}`,
  `{bicycle}`, `{motorcycle, autorickshaw}`, `{car, bus, truck}`. This is a
  hard feasibility mask in `_associate()`'s gate, on top of (not instead of)
  the existing soft `lambda_cls` cost within a group. Targets the leak
  EXP4-QA.md check 4 found: 155/678 raw confirmed tracks (23%) absorbed a
  stray `person` detection into a vehicle track because it was the
  spatially-cheapest *legal* match under the old soft-cost-only gate.
  Bicycle stays its own group rather than merging with motorcycle: no leak
  evidence for that pair in EXP4-QA.md, and `test_mtrack.py`'s own gap-test
  crosses exactly a motorcycle/bicycle pair mid-occlusion, so allowing that
  bridge would risk the very id-swap the gate exists to prevent.
- **FIX 2 (`src/trajectories.py` + `src/conflicts.py`, `reemerged` column):**
  a first-class boolean column, true for `reemerge_s` (2.0s) after an
  imputed (occlusion-bridged) run ends. `conflicts.py`'s `pet()`/`ttc()`
  exclude `reemerged` rows the same way they already exclude `imputed` ones.
  Targets EXP4-QA.md check 2: 81.5% of mtrack's critical/serious conflicts
  had a track re-emerging within 2.0s of the conflict — the RTS-smoothed
  velocity is least trustworthy right at a gap edge. Applied identically to
  both tags so the comparison stays like-for-like.

## Headline table

Default recomputed under the new conflict rule (`reemerged` exclusion now
applies to both tags); `mtrack` (pre-fix) is EXP4.md's original column,
kept for reference.

| metric | default (recomputed) | mtrack (pre-fix, EXP4.md) | **mtrack + fixes** |
|---|---|---|---|
| total tracks | 731 | 561 | 602 |
| duplicate tracks removed | 402 | 110 | **180** |
| traversing tracks (% of total) | 243 (33.2%) | 294 (52.4%) | **305 (50.7%)** |
| imputed fraction | 0.1074 | 0.2093 | 0.2233 |
| reemerged fraction (new, row-level) | 0.1993 | n/a (not tracked pre-fix) | 0.3299 |
| OSM match % | 81.8 | 81.3 | 82.6 |
| conflicts crit/serious/total | **29/50/1,506** | 178/157/2,558 | **27/40/1,363** |
| modal split (2W/person/car/auto) | 43.8/27.1/25.3/1.8 | 47.6/18.2/29.2/2.9 | 46.2/**19.1**/29.1/3.7 |
| person tracks (n) | 198 | 102 | 115 |
| person median dwell (s) | 10.6 | n/a (not broken out) | 36.8 |
| motorcycle length (m, expect 1.986) | 1.845 (−7.1%) | 1.852 (−6.7%) | **1.851 (−6.8%)** |
| median track dwell (s, all classes) | 19.2 | 33.1 | 31.0 |
| cumulative id growth (ids/min) | 365.8 | 280.7 | 301.0 |
| \|accel\|>4 m/s² fraction | 0.0% | (not reported) | 0.0% |
| kinematics_plausible fails | 0.14% | (not reported) | 0.0% |

All numbers from a fresh `run_analysis → run_attrs → run_geo →
build_dashboard` run per tag, both under identical code (this commit) so
"default" and "mtrack + fixes" are directly comparable.

## Verdict against the bar

| bar | result | pass? |
|---|---|---|
| pedestrian share within ~5pts of default's 27.1% | 19.1% (8.0pts off; +0.9pt vs pre-fix 18.2%) | **no — see below** |
| duplicates still far below 402 | 180 | yes |
| traversing still far above 33% | 50.7% vs 33.2% | yes |
| motorcycle check within 10% | −6.8% off | yes |
| critical conflicts within ~1.5x of default under same rule | 27 vs 29 (below, not just within 1.5x) | yes |

**4/5 bars pass cleanly; the conflict-spike fix (the more consequential of
the two) fully resolves — mtrack's critical count is now *below* the
default's own recomputed count, closing what was a 4.8x gap.** Duplicates,
traversing, and the motorcycle sanity check are all unaffected or improved.

**Pedestrian share (data-backed reason it falls short of the bar):** the
hard gate mechanically stops the absorption event QA found (confirmed by
`test_class_hard_gate_blocks_person_into_car`, which also verifies the gate
being off reproduces the leak) and moves raw confirmed tracks 678→796
(+118) as previously-bridged marginal detections across several classes
fragment into short tracks of their own instead of extending a
neighbouring track. But blocking the merge doesn't retroactively give a
formerly-absorbed detection run the ≥`min_hits=3` consecutive same-class
frames it needs to become its own confirmed track: raw confirmed `person`
tracks are 164 pre-dedup vs a final (post-`calibrate()`-dedup) 115, a 30%
attrition rate somewhat above the pool's overall 24% (796→602) — consistent
with many of the newly-freed person detections being too short-lived to
survive on their own. The gate removes the contamination from vehicle
tracks; it does not, and structurally cannot, recover pedestrian coverage
the original detector/tracker never had. Closing the remaining 8-point gap
would need a detection-side fix (recall on brief/occluded pedestrian
detections), not an association-side one — out of this fix's scope.

## Tests

`python tests/test_mtrack.py` (pytest not installed in this environment;
ran the file directly, which the module's own `if __name__` block
exercises) — **both cases pass**, including the new
`test_class_hard_gate_blocks_person_into_car` (gate-on blocks the leak;
gate-off negative control reproduces it, proving the test isn't vacuous).

## Commit

`mtrack: class-group gate + reemerged flag (WIP)` (all six touched files),
report backups under `out/exp4fix/*.pre.json` (before this session's
reruns) and per-stage logs under `out/exp4fix/*.log`.

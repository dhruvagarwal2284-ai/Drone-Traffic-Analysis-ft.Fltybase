# What was done to the traffic-analysis agent, and why

_A plain-language briefing on the 15 Sep 2026 revision of the drone traffic-analysis
pipeline (Baner junction, Pune). Every claim below points at a file in this repo that
holds the evidence. Written for presenting; the technical detail lives in the linked files._

---

## 1. Starting point

The submission (22 Aug, extended 31 Aug) extracts road-user trajectories from drone
video and turns them into traffic insight: turning movements, near-miss conflicts,
queues, an inferred signal cycle, per-vehicle attributes, and a map-matched export.
It worked end to end and was well documented.

Its own write-up named its biggest weakness: the detector was **YOLO11x trained on
COCO**, a ground-level image set. Seen from directly overhead, COCO "has never seen a
two-wheeler", so it routinely labelled motorcycles as **cars**. In a city where
two-wheelers are the dominant road user, that error sits underneath every downstream
number. The write-up listed "aerial-pretrained weights" as next step #1 and a tracker
re-architecture as #2.

The work below followed that list — but proved each step cheaply before paying for
the next one.

## 2. What was done, in order

| Step | What | Why | Evidence |
|---|---|---|---|
| 0 | **Reproducibility check** — re-ran every deterministic stage from the cached detections and compared the report value by value. | No point changing a pipeline you cannot reproduce. Result: 265 values, 0 differences, 106 s. Also found the submission zip was one revision stale and rebuilt it. | `PREP.md` |
| 1 | **Scouted aerial-pretrained weights** — which exist, are downloadable, licensed, and load in the installed ultralytics. | Picked `dronefreak/visdrone-yolov11s` (VisDrone dataset: has `motor`, `tricycle`, `awning-tricycle` = auto-rickshaw). Ruled out DOTA-OBB: its 15 classes have no motorcycle or pedestrian, so it cannot fix the class problem. | `PREP.md` §Aerial weights |
| 2 | **20-frame bake-off** — three detectors on the same 20 frames of the same 120 s window. | The cheapest test that could falsify the idea: 19 s of GPU. COCO: motorcycles 8% of detections, cars 76%. VisDrone-YOLO11s: motorcycles **56%**, cars 22%, and 2.8× faster per frame. Side-by-side images confirmed the new boxes sit where COCO's "car" boxes were on rows of small vehicles — a reclassification, not noise. | `EXP1.md`, `out/exp1/sidebyside_*.jpg` |
| 3 | **Full pipeline re-run with VisDrone** on a git branch, new output tag, nothing on main touched. | Proves the win holds at full scale and shows the cost: 687 → 909 road users, pedestrians 19.5% → 29.5%, auto-rickshaws get their own class. Cost: duplicate (fragmented) tracks 80 → 541 and OSM map-match 96.6% → 82.7%, because the image-space tracker strains under twice the detection density. | `EXP2.md` |
| 4 | **Fixed two latent pipeline bugs** found during step 3. | (a) Running the analysis before the attributes stage silently zeroed every colour/size attribute. (b) The map-export stage wrote un-tagged files, so a second run overwrote the first. Both were independent of the detector and would have bitten again. Verified: 0 differences on all 45,084 non-colour report values after the fix. | commit `088f162` |
| 5 | **Tracker tuning bake-off** — four built-in tracker configurations, same detections. | Before publishing numbers, try the cheap fix for fragmentation. Winner: ByteTrack with `track_buffer=90`, `match_thresh=0.9` — duplicates 541 → 402 (−26%), traversing tracks 26% → 33%. BoT-SORT, with or without re-identification, was worse. The OSM match rate stayed ~82% on every tracker, which proves that gap is a detection-coverage effect, not an ID-association one. | `EXP3.md`, `trackers/` |
| 6 | **Made the new configuration the default, regenerated every artefact** from one clean run: headline numbers, README, WRITEUP (with a new "Revision" section), dashboard, demo clips, package. Before editing any prose, every changed figure was logged old → new → source key. | The numbers people will quote must all trace to one run of one configuration. Also fixed a non-determinism (an unseeded random sample in the colour stage) so the run reproduces exactly: two runs, 0 differences. | `NUMBERS-CHANGELOG.md`, commits `3fed267`, `0ec8f26`, `0945895` |

## 3. The headline before / after

| | COCO YOLO11x (submitted) | VisDrone-YOLO11s + tuned ByteTrack (now) |
|---|---|---|
| Road users / trajectory samples | 687 / 113,620 | **731 / 204,632** |
| Modal split | 2W 45.1%, car 31.4%, ped 19.5% | **2W 43.8%, ped 27.1%, car 25.3%, auto-rickshaw 1.8%** |
| Traversing (complete) tracks | 204 (29.7%) | **243 (33.2%)** |
| Conflicts (critical / serious / total) | 20 / 24 / 1,133 | **37 / 61 / 1,772** |
| Two-wheeler share of conflict-grade events | ~85% | **86%** — same finding, stronger margin |
| Anomalies | 48 | **84** (61 contraflow, 21 stopped, 2 hard-braking) |
| Objects with measured dimensions | 594 (86%) | **675 (92%)** |
| OSM map-match | 96.6% | **81.8%** (see caveats) |
| Detector speed | 267 ms/frame | **95 ms/frame** |

Full list of every changed number: `NUMBERS-CHANGELOG.md`.

## 4. Two findings worth presenting

1. **The class error was contaminating the scale calibration.** The pipeline fixes
   its metric scale by assuming the median "car" is 4.0 m long. With COCO, many of
   those "cars" were motorcycles, so the observed car footprint was 2.84 m and the
   scale factor came out ×1.51 (effective height 106.6 m). With correct classes the
   footprint is 3.11 m, the scale ×1.38, effective height 97.6 m. Every distance and
   speed in the original run carried that error. The independent sanity check
   (motorcycle length, never fitted) moved from +6.6% to −7.1% of expected — no worse,
   opposite sign, and now measured on the right objects.
2. **Fragmentation, not classes, is now the ceiling.** Denser, correct detections
   exposed the image-space tracker. Tuning recovered a quarter of the duplicates; the
   rest — and the map-match drop — need the tracker re-architecture the write-up
   already proposed (associate in the metric ground frame, add appearance re-ID
   across the tree occlusion). That is the honest next step, and the numbers now
   show exactly why.

## 5. Caveats to state out loud

- **Conflict counts remain an upper bound.** More real interactions are seen, but
  residual fragmentation inflates the absolute count; the pattern and class
  composition are the robust reads.
- **OSM match dropped to 81.8%.** More detections fall outside the OSM road buffer
  (footpaths, the construction narrowing). Flow/density figures are computed from
  the trajectory table, not from the map-match, so they are affected by track
  quality rather than by this rate — an earlier draft got that causal chain wrong
  and it was corrected in the write-up.
- **Worst-delay figure needed a caveat.** The raw worst movement (`NE→SE`, 51 s)
  rests on 2 tracks; the reported figure is the best-sampled worst (`W→SE`, 5.4 s,
  n=67) with the raw one footnoted.
- **Body-type length tables are thinner for buses/vans** (n=1–3) in this window.
- **Modal split moved when the tracker buffer grew** (2W 46.2% → 43.8%), because
  fewer fragments means fewer double-counted two-wheelers — the direction you want,
  but it shows how tracker settings shape a count-based split.
- The VisDrone weights are **AGPL-3.0** (`models/README.md`); fine for research and
  a hackathon, worth knowing before any commercial packaging.

## 6. What was deliberately not done

- No tracker re-architecture, no SAHI tiling: each is a bigger job than the evidence
  justified today. They stay as next steps #1 and #2 in the README.
- No retraining, no new dependencies, no changes to the analysis mathematics.
- The COCO-era outputs were not deleted: `out/exp4/report_intersection_COCO.json`,
  the Aug-22 package at the folder root, and git history all keep them.

## 7. Reproduce it

```bash
# weights (see models/README.md)      -> models/visdrone-yolov11s.pt
python src/track.py        --tag intersection --t0 240 --dur 120 --fps 10
python src/run_analysis.py --tag intersection --t0 240 --fps 10
python run_attrs.py && python run_geo.py
python src/build_dashboard.py intersection
bash make_package.sh
```
One clean run: ~90 s on the GPU for tracking, ~2 min for the rest. The run is
deterministic (verified twice, 0 differences).

## 8. Effort

Six short jobs on 15 Sep, each gated by the previous one's evidence; under 15 GPU-minutes
in total. Git history on `main` tells the same story commit by commit
(`git log --oneline` from `66fd15b` to `0945895`).

---

# Revision 2 — the tracker re-architecture (16 Sep 2026)

_Follows on from the steps above. Same rule: every claim points at a file._

## 9. Why a second round

Revision 1 fixed the classes and exposed the next ceiling: the image-space tracker
fragmented tracks (402 duplicates, only a third of tracks crossing the scene), and
tuning its knobs recovered only a quarter of that. The write-up's own next step #2
was "associate in the metric frame, re-identify across the occlusion". You said go.

## 10. What was done, in order

| Step | What | Why | Evidence |
|---|---|---|---|
| 7 | **Decoupled detection from tracking** — `src/detect.py` writes every detection with its ground-plane position in metres, reusing the repo's own projection and calibration code. | The old stage fused detector and tracker, so nothing could be changed in between. Calibrated scale matched the tracked run to 0.14%. | `EXP4A.md` |
| 7b | **Tested tiling (next step #3) while the door was open.** | 2×2 tiles found only 1.21× more road users with VisDrone (COCO had promised 1.7–1.9×) and produced seam artefacts on buses and trucks. **Closed as a negative result** — the aerial detector already covers what tiling was for. | `EXP4A.md` |
| 8 | **Wrote a metric-frame tracker** — `src/mtrack.py`, ~300 lines: constant-velocity Kalman filter in metres, Hungarian assignment, class-aware costs, an occlusion buffer that predicts *through* the tree for up to 6 s and re-attaches the track when the vehicle re-emerges. Same output schema, so every downstream stage ran unchanged. One synthetic test. | Duplicates 402 → 110, traversing 33% → 52%, median track life 19 → 33 s. | `EXP4.md`, `tests/test_mtrack.py` |
| 9 | **Held it at independent QA** instead of publishing. Two headline numbers had moved suspiciously: critical conflicts 37 → 178, pedestrians 27% → 18%. | **81.5% of the conflict rise was an artefact**: near-misses "detected" within 2 s of a track re-emerging from the occlusion, where the re-attached position is least certain. Under a 2 s exclusion the rates were equal (30 vs 29). The pedestrian drop was half real de-fragmentation, half a defect: person detections being absorbed into vehicle tracks. | `EXP4-QA.md` |
| 10 | **Two prescribed fixes**: a hard class-group gate (a person can never join a vehicle track) and a first-class `reemerged` flag that the conflict stage excludes — applied to the old tracker too, so the comparison stays like-for-like. | Critical conflicts 27 vs 29 under the same rule; duplicates 180; traversing 50.7%; motorcycle check −6.8%; new test proves the gate. | `EXP4-FIX.md` |
| 11 | **Made it the default and regenerated everything** from one clean, deterministic run (two runs, 0 of 242,679 rows differ); every changed figure logged. | Same discipline as Revision 1. | `NUMBERS-CHANGELOG.md` §Revision 2, WRITEUP §16 |

## 11. Headline now

| | Revision 1 (ByteTrack) | Revision 2 (metric-frame tracker) |
|---|---|---|
| Road users / samples | 731 / 204,632 | **602 / 253,972** — fewer objects, more of each one |
| Complete (traversing) tracks | 243 (33%) | **305 (51%)** |
| Duplicate tracks removed | 402 | **180** |
| Median track life | 19 s | **33 s** |
| Modal split by track | 2W 43.8 / ped 27.1 / car 25.3 | 2W 46.2 / car 29.1 / ped 19.1 / auto 3.7 |
| Modal split by presence (per-frame detections, tracker-independent) | — | **2W 52.9 / car 23.9 / ped 17.8** |
| Conflicts, same 2 s rule (critical / serious / total) | 37 / 61 / 1,772 | **27 / 40 / 1,363** |
| Kinematics physically plausible | 99.9% | **100%** |
| OSM map-match | 81.8% | 82.6% |

## 12. Three things to say when presenting this

1. **A tracker that follows vehicles through the tree changes what "a road user"
   means.** 731 became 602 not because traffic vanished but because fragments were
   glued back into whole journeys — half the tracks now cross the entire scene.
2. **The conflict count is the cautionary tale.** The first metric-frame run showed
   4.8× more critical conflicts. Publishing that would have been wrong: 81.5% of
   them were re-attachment artefacts. The fix is now a rule in the pipeline
   (`reemerged` samples are never used for conflicts), and the honest number is
   27 — slightly *below* the old tracker's 29 under the same rule.
3. **Modal split depends on the tracker, so we now publish two.** By track, each
   pedestrian used to count ~4 times (fragments); by presence in frames the
   pedestrian share is 17.8%. Quote "by track" for volumes, "by presence" for
   composition.

## 13. Caveats

- OSM map-match is unchanged at ~82%: it was never a tracking problem; it needs a
  wider road buffer or filtering detections to plausible road surface before
  matching. Flow/density/queue numbers do not depend on it.
- Pedestrian coverage is a detection-recall limit now, not an association one.
- No appearance re-identification network yet; `EXP4.md` states what one would add.
- Conflict counts remain an upper bound, now for a smaller and better-understood
  reason.

## 14. What is next (and what is closed)

Closed: aerial-pretrained detector (done), tracker re-architecture (done), SAHI
tiling (negative result). Open, in order: map-match road buffer / detection
filtering for the OSM rate; pedestrian recall; a ReID network for the occlusion.

## 15. Reproduce Revision 2

```bash
python src/detect.py   --tag intersection
python src/mtrack.py   --tag intersection --from out/detraw_intersection.parquet
python src/run_analysis.py --tag intersection --t0 240 --fps 10
python run_attrs.py && python run_geo.py
python src/build_dashboard.py intersection
bash make_package.sh
```
Git history on `main`: `ba50b9f` → `fbc1b60`. Round-2 cost: eight short jobs, one
of them a deliberate QA hold; about 10 GPU-minutes.

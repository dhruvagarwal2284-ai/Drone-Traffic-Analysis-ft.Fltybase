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

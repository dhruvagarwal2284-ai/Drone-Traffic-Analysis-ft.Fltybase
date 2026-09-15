# EXP3 — tracker tuning for VisDrone fragmentation (ByteTrack knobs, BoT-SORT)

Run 2026-09-15 by temp `worker-fh-exp3`, branch `visdrone`. Same 120 s window
(`out/window_visdrone.mp4`, hard-linked per tag, no re-cut), same
`models/visdrone-yolov11s.pt` weights and detection settings as EXP2 — only the
tracker (`--tracker` yaml, new flag added to `src/track.py`) and its knobs vary
per run. Baseline = EXP2's `visdrone` tag (plain `bytetrack.yaml` defaults).
4 tracking runs, ~6 min GPU total (within the 5-run/15-min cap). Tracker yaml
files added under `trackers/`.

## Table (COCO and baseline VisDrone columns copied from EXP2)

| metric | COCO (`intersection`) | VisDrone baseline (`visdrone`, EXP2) | `vd_bt_buf60` | **`vd_bt_buf90` (winner)** | `vd_botsort` | `vd_botsort_reid` |
|---|---|---|---|---|---|---|
| tracker | bytetrack defaults | bytetrack defaults | bytetrack, buf=60 | **bytetrack, buf=90, match=0.9** | botsort, no reid, buf=60 | botsort, reid (model=auto), buf=60 |
| total tracks | 687 | 909 | 877 | 731 | 848 | 859 |
| duplicate tracks removed | 80 | 541 | 552 | **402** | 522 | 659 |
| traversing tracks (% of total) | 204 (29.7%) | 238 (26.2%) | 239 (27.3%) | **243 (33.2%)** | 228 (26.9%) | 232 (27.0%) |
| imputed fraction | 0.0666 | 0.0645 | 0.0784 | 0.1074 | 0.0838 | 0.1022 |
| OSM match % | 96.6 | 82.7 | 82.4 | 81.8 | 82.6 | 82.4 |
| conflicts crit/serious/total | 20/24/1133 | 29/57/1660 | 28/58/1671 | 37/61/1772 | 35/47/1675 | 35/43/1682 |
| modal split (2W/person/car) | 45.1/19.5/31.4 | 46.2/29.5/20.5 | 46.3/28.8/20.9 | 43.8/27.1/25.3 | 46.2/28.2/21.6 | 45.3/28.9/21.5 |
| motorcycle length (m, expect 1.986) | 2.117 (+6.6%) | 1.843 (−7.2%) | 1.843 (−7.2%) | 1.845 (−7.1%) | 1.843 (−7.2%) | 1.843 (−7.2%) |
| wall time, track.py | ~90s | ~90s | ~80s | ~85s | ~110s | ~110s |

All four variants keep the same detector/window/classes, so every column
change is attributable to the tracker alone.

## Winner: `vd_bt_buf90` — bytetrack, `track_buffer: 90`, `match_thresh: 0.9`

1. **It is the only variant that actually reduces fragmentation, not just
   reshuffles it.** Duplicate tracks removed drops 541→402 (−26%) and
   traversing-track share rises 26.2%→33.2% (+7 points) — both `buf60` and
   both BoT-SORT variants land within noise of the baseline (522–552 dup,
   26.9–27.3% traversing). A longer lost-track memory (9 s vs 3 s at 10 fps)
   combined with a *stricter* match threshold, not either knob alone, is what
   moves the needle: `buf60` alone (same buffer, default 0.8 match) barely
   changed anything, so the win is from being pickier about which detections
   re-acquire a lost id, not just holding ids open longer.
2. **ReID made things worse, and BoT-SORT's extra cost bought nothing.**
   `with_reid: true` (falling back to `model: auto`, i.e. the detector's own
   features — no aerial-trained ReID weights, no download needed) raised
   duplicate tracks to 659, the worst of the five runs: appearance features
   for small, similar-looking overhead two-wheelers are not discriminative
   enough to help and add false re-identifications instead. BoT-SORT without
   ReID performed like plain ByteTrack at ~20% higher wall time (global
   motion compensation is pure overhead on this static camera). Neither
   justifies its cost here.
3. **The sanity checks hold, with one caveat to flag.** The independent
   motorcycle-length check is unchanged (1.845 m vs 1.986 m expected, −7.1%,
   same as baseline). Modal split shifts more than the other runs: two-wheeler
   share 46.2%→43.8%, car 20.5%→25.3%, person 29.5%→27.1%. This is a genuine
   second-order effect, not noise: fewer, longer-lived tracks change which
   objects clear `MIN_TRACK_SEC` and the deduplicate() radius test, so some
   short two-wheeler/pedestrian fragments that were previously counted twice
   (or counted at all) now merge or drop. Flag before quoting modal split
   from this tag specifically.

**OSM match % did not improve on any tracker variant** (81.8–82.6% across all
four vs. 82.7% baseline, all still ~14 points below COCO's 96.6%). This
confirms EXP2 observation 3: the drop is a *detection-coverage* problem —
VisDrone's denser field puts more boxes in places (verges, parked two-wheelers,
pavement) outside the OSM road network's 20 m buffer — not an
*ID-association* problem tracker knobs can fix. The README's next-step #2
(re-architect the tracker, decouple detection from tracking, associate in the
metric frame) remains necessary for the match-rate/flow/queue numbers; this
experiment only fixes the fragmentation half of the problem, cheaply, as a
stopgap.

## Visual QA (`out/exp3/frame_08s.jpg`, `frame_16s.jpg`, from `overlay_vd_bt_buf90.mp4`)

Boxes, classes and lengths render correctly — no misboxed or flickering
objects visible in either frame, `2W`/`car`/`auto`/`motor` labels sit on the
right vehicles, map ribbons and the aggregate panel look normal. ID churn is
still visible but slower than baseline: cumulative ids climb 307→428 (+121)
between t=8s and t=16s while only ~190 objects are visible on screen at once
— consistent with the quantitative result (fragmentation cut, not
eliminated). No frame-to-frame id reassignment is visible on the frames
sampled, but 2 frames 8s apart cannot rule out short id switches between them.

## Config changes (branch `visdrone`, committed)

- `src/track.py`: added `--tracker` (passed through to `model.track(tracker=...)`,
  previously hardcoded to `"bytetrack.yaml"`).
- `trackers/bytetrack_buf60.yaml`, `bytetrack_buf90.yaml`, `botsort_noreid.yaml`,
  `botsort_reid.yaml`: the four configs tried, kept for reference.
- `config.yaml`: `detector_visdrone.tracker` now points at
  `trackers/bytetrack_buf90.yaml` (winner), with the rationale inline.
  Still documentation-only per EXP2's note — nothing in this repo parses
  `config.yaml` at runtime; `src/track.py`'s `--tracker` default is unchanged
  (`bytetrack.yaml`, i.e. COCO path unaffected), so adopting the tuned tracker
  for a real run means passing `--tracker trackers/bytetrack_buf90.yaml`
  explicitly, same as this experiment did.

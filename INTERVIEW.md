# Interview guide: Drone Traffic Analysis (Baner Junction, Pune)

How to present this project in 2, 5 or 15 minutes, with the numbers and the likely
questions. Every figure below comes from `demo/report.json` and can be checked on
`demo/dashboard.html`.

---

## The 30-second pitch

> "I built a pipeline that turns ordinary drone video into a traffic-engineering study,
> with no annotations, no ground-control points and no map. It detects every road user,
> tracks them in real-world metres and derives what a traffic engineer would pay a
> survey crew for: turning counts, queues in metres, near-miss conflicts, the signal
> cycle, and per-vehicle size, colour and speed. On a 2-minute window of a Pune junction
> it tracked 602 road users over 254k positions. It found that the drone's reported
> altitude was off by 27 m, that the junction runs on a 116 s signal cycle, and that
> two-wheelers are in 89% of the conflicts."

---

## The 2-minute version (problem → approach → result → honesty)

**Problem.** Fixed traffic cameras see one approach in perspective. They cannot put two
road users in the same coordinate frame, so near-misses go unmeasured. They also only
exist where someone installed one.

**Approach**, five steps:
1. **Telemetry → geometry.** The DJI video carries per-frame GPS, altitude and gimbal
   angles in a subtitle track. From those I build an analytic image-to-ground
   homography. No surveyed points are needed.
2. **Detect** with an aerial-trained YOLO (VisDrone). It knows what a motorbike looks
   like from above, which a ground-level COCO model does not.
3. **Track in metres, not pixels.** I wrote my own tracker: one Kalman filter per road
   user, with Hungarian assignment on ground-plane distance. It predicts through a tree
   that hides the junction centre for up to 6 s.
4. **Smooth and calibrate.** An RTS smoother gives clean speeds and accelerations. A
   self-calibration fixes the scale: the median car must be 4 m long.
5. **Query the trajectory table** for PET/TTC conflicts, turning movements, Edie
   flow-density, queue length in metres, signal cycle, anomalies and OSM map-matching.

**Result.** 602 road users, 305 of them tracked across the whole junction. 1,362
interaction pairs, 67 of them serious or critical. A 116 s signal cycle recovered
without a controller feed. 100% of samples are physically plausible.

**Honesty.** There is no ground truth, so I validate with physics and with independent
checks the pipeline could have failed. The main one: after calibrating on cars,
motorcycles measure 1.85 m against 1.99 m expected, and motorcycles were never used in
the fit.

---

## The three findings (these are what people remember)

### 1. "The drone's altitude was wrong, and the trajectories caught it"
- Telemetry says **70.5 m**. That is height above the *take-off point*, and the drone
  took off from a rooftop.
- If the median car must be 4 m long, the scale must be **×1.382**, which means an
  effective height of **97.4 m**.
- **Independent check:** motorcycles then come out at **1.85 m vs 1.99 m expected**
  (−7%). Motorcycles were not used in the fit, so this is a real test.
- Why it matters: every distance and speed scales with this factor. Uncorrected, they
  would all read about 28% low (1/1.382), and so would queue lengths and flow-density.

### 2. "The junction is signal-controlled; the corridor is not"
- A detector-free signal (occupancy vs motion from frame statistics) autocorrelates at
  **116 s** (r = 0.18) at the junction.
- The same method on the second video (a corridor) gives only r = 0.15 at the edge of the
  search band, which is noise.
- **So what:** retiming signals helps one site and would do nothing for the other, where
  congestion comes from side friction and a construction narrowing.

### 3. "Two-wheelers dominate the safety picture"
- **323 of 362** conflict-grade events (PET/TTC ≤ 3 s) involve a two-wheeler: **89%**,
  against 46% of road users.
- This held through three detector/tracker configurations, so it describes the traffic,
  not a model quirk.

---

## The engineering story: "how I made it better" (good for "tell me about a hard problem")

| | v0 (submitted) | v1 | v2 (current) |
|---|---|---|---|
| Detector | COCO YOLO11x | **VisDrone YOLO11s** | VisDrone YOLO11s |
| Tracker | ByteTrack (pixels) | tuned ByteTrack | **own metric-frame Kalman tracker** |
| Road users | 687 | 731 | 602 |
| Tracks crossing the whole junction | 30% | 33% | **51%** |
| Duplicate tracks removed | 80 | 402 | 180 |
| Two-wheelers labelled correctly | ✗ (called cars) | ✓ | ✓ |

**Three lessons to tell, one per revision:**

1. **Test the cheapest thing that could prove you wrong.** Before swapping detectors I ran
   a 20-frame bake-off (19 s of GPU). COCO labelled 8% of detections as motorcycles;
   VisDrone labelled 56%. The side-by-side images showed COCO's "cars" on rows of bikes.
   Only then did I pay for the full re-run.

2. **A fix can expose the next bottleneck.** Correct detections were denser, and the
   pixel-space tracker fragmented under them: duplicates went from 80 to 541. Tuning
   recovered only a quarter of that, so I wrote a tracker that works in ground metres,
   where a car's motion is simple and its size is known.

3. **Hold suspicious numbers for QA instead of publishing them.** The new tracker's
   first run showed **4.8× more critical conflicts**. I did not publish it. Independent
   QA showed **81.5%** of them occurred within 2 s of a track re-emerging from the tree
   occlusion, where the smoothed velocity is least reliable. The fix became a pipeline
   rule: those samples are excluded from PET/TTC for every tracker. Under that rule the
   new tracker gives 27 critical conflicts vs 29 for the old one. The safety signal is the
   same; the tracker is cleaner.

**The final polish pass** (this week, `IMPROVEMENTS.md`) found and fixed three smaller
issues, each with a test that fails on the old code:
- Stray per-frame labels leaked into tracks, creating a phantom `motorcycle` class and
  mis-weighting PCU.
- Anomalies were all stamped at the window start instead of when they happened.
- A README headline (310/362) was a partial sum. The pipeline now computes it (323/362).

---

## Live demo script (dashboard, ~4 min)

Open `demo/dashboard.html`. It is self-contained and works offline, except for the web
fonts.

1. **Top tiles and "Three findings".** Tell the story above in about 60 s.
2. **Network → "The road network, discovered from motion".** "Nobody drew these roads.
   Every line is one road user's path in metres. The legs are found from where tracks
   enter and leave."
3. **Conflicts → ranked table**, then **Evidence → play clip #1.** "Each row is a
   real pair at a known time and place. Here it is on video."
4. **Turning → the OD matrix.** "This is the manual turning-count survey, done
   automatically. SE→W carries 109 vehicles."
5. **Aggregate → back of queue in metres.** "A vehicle count cannot tell you whether a
   queue blocks the upstream junction. A distance can: the W leg reaches 68 m."
6. **Map → desire lines on real OSM geometry.** "Results are tied to named roads and
   link IDs, so they can join any asset register."
7. **Validation**, then **"How the numbers got better".** End on honesty and iteration.

---

## Likely questions, with answers

**"How do you know it's accurate without ground truth?"**
Physics and independent checks. 100% of samples are under 4 m/s² acceleration, none
exceed 100 km/h, and motorcycle length comes out within 7% although it was never fitted.
I also check determinism: two runs give 0 differing values. I would not quote a
precision/recall figure without labels.

**"Why write your own tracker instead of using DeepSORT or BoT-SORT?"**
I tried BoT-SORT, with and without ReID, and plain ByteTrack with four configurations.
None beat 402 duplicates. They all associate in *pixels*, where perspective and
occlusion make motion non-linear. In ground metres a vehicle is close to
constant-velocity, so a Kalman gate can bridge a 6 s occlusion. It is about 300 lines
and deterministic, and it cut duplicates from 402 to 110 (180 after the class gate).

**"What's PET vs TTC?"**
PET (post-encroachment time) is the time between one user leaving a spot and another
arriving at it; it suits crossing conflicts. TTC (time to collision) is how long until
contact if both keep their current velocity; it suits closing conflicts. Car-following is
excluded with a crossing-angle test (≥ 40°). Grades: under 1 s critical, under 1.5 s
serious, under 3 s conflict.

**"Why does 'road users' go *down* from 731 to 602 with a better tracker?"**
Fragments were glued back into whole journeys. The median track life went from 19 s to
33 s. Fewer IDs means fewer double-counted people, not less traffic.

**"Biggest weakness?"**
Three: (1) conflict counts are an upper bound, so the pattern is more robust than the
absolute rate; (2) 17% of samples fall outside the OSM road buffer, which is a
detection-side issue; (3) it is one 2-minute window, not a full day.

**"Licence plates / make and model?"**
Not achievable, and I can show why. A 65 mm plate character is about 2 px tall here
against about 16 px needed for OCR, a 7.4× shortfall. At −63° pitch the plates face away
from the camera anyway. This needs a different sensor tasking, not a better model.

**"How would you productionise it?"**
A single config-driven entry point (today `config.yaml` is documentation only). Batch
multiple windows for confidence intervals. Filter detections to the road surface before
map-matching. Add an appearance-ReID term; `detect.py --embed` already emits the vector.
Store trajectories as the primary product, since every insight is a query over that table.

**"Tech stack?"**
Python, PyTorch + Ultralytics YOLO11 (CUDA 12.8 on an RTX 5050), OpenCV, ffmpeg (NVDEC),
NumPy/SciPy (Kalman/RTS, Hungarian), pandas + Parquet, OSM for the map, and a
self-contained HTML/Canvas/SVG dashboard with no JS dependencies.

---

## Numbers cheat-sheet

| | |
|---|---|
| Window | 120 s at 10 fps (t = 240–360 s), 4K, −63° gimbal pitch |
| Road users / samples | 602 / 253,972 |
| Crossed the junction | 305 (51%) |
| Modal split (by track) | 2W 46.2 · car 29.1 · ped 19.1 · auto 3.7 · truck 1.5 · bus 0.5 |
| Modal split (by presence) | 2W 52.9 · car 23.9 · ped 17.8 · auto 3.4 · truck 1.6 · bus 0.3 |
| Conflicts | 27 critical · 40 serious · 295 conflict · 1,000 minor |
| Two-wheeler share of conflicts | 323/362 = 89% |
| Turning | 186 through · 51 left · 39 right · 29 u-turn |
| Signal cycle | 116 s (r = 0.18) |
| Max queue | 68 m (W leg) |
| Max flow | 1,556 veh/h/lane at 75 veh/km/lane |
| Scale correction | ×1.382 → 97.4 m effective height (telemetry: 70.5 m) |
| Motorcycle check | 1.85 m vs 1.99 m expected |
| OSM match | 82.6% |
| Tests | `tests/test_mtrack.py`, `tests/test_pipeline_fixes.py` |

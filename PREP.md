# Prep notes — qe-1 (drone traffic analysis)

Run 2026-09-15 by temp `worker-fh-prep3`. No detector re-run (`src/track.py` not touched,
no YOLO pass over the source 4K videos). Env: torch 2.11 cu128, RTX 5050, ultralytics
8.4.126, ffmpeg at `C:/ffmpeg/bin`.

Two earlier temps (`worker-fh-prep`, `worker-fh-prep2`) were dispatched for this same job
but left no notes (memory.md still empty in the index) — per god's memory.md they stalled
on a repo-trust prompt before doing any work. This run started clean; no PREP.md existed.

---

## Reproducibility

Backed up `out/report_intersection.json` to `.bak`, then ran, in order, from the existing
`out/detections_intersection.parquet` + `out/trajectories_intersection.parquet` (both
untouched — no `track.py`, no GPU inference):

| command | wall time | result |
|---|---|---|
| `python src/run_analysis.py --tag intersection --t0 240 --fps 10` | 31.4 s | 687 tracks, 113,620 rows, report regenerated |
| `python run_attrs.py` | 25.5 s | 687/687 attributes rebuilt |
| `python run_geo.py` | 46.5 s | 96.6% map-matched, 1.97 m residual, geojson rebuilt |
| `python src/build_dashboard.py intersection` | 3.3 s | dashboard.html rebuilt |

**`run_attrs.py` does not re-run YOLO.** Checked `src/attributes.py` first (no `YOLO`/
`.predict`/`.track` calls) — it only re-reads `out/window_intersection.mp4` (the already
-extracted 120 s window, not the raw 6.5 GB source) for pixel-level colour/size sampling
against the existing detections and trajectories. Confirmed safe to run per the job's own
instruction to check before running.

**Diff vs backup:** flattened both JSON files to leaf key/value pairs and compared —
**265 keys, 0 differences.** Every figure in the report (687 tracks, 45.1/31.4/19.5%
modal split, 20/24/243/846 conflicts, 96.6% map-match, 1.97 m residual, calibration
scale 1.5128, everything) reproduced byte-for-byte identical. No errors, no warnings, no
stochastic drift. Backup removed after the diff (redundant, identical to source).

**Verdict: fully reproducible.** The whole non-detection half of the pipeline (metric
projection → conflicts → attributes → aggregate → geo → dashboard) is deterministic
given the cached detections/trajectories.

---

## Package

`traffic-analysis-agent.zip` was built Aug 22 18:09 by `make_package.sh`, which already
listed `src/geo.py`, `src/aggregate.py`, `run_geo.py` and `demo/geojson/*` in its
contents — **contrary to god's memory note that these were missing.** The catch: that zip
predates `run_geo.py`'s last edit (18:16) and the dashboard's final rebuild (19:07) the
same day, so its bundled `demo/report.json` / `demo/dashboard.html` were likely one
revision stale even though the source files were present. Rebuilt anyway using the
freshly-regenerated `out/` files from the Reproducibility step above, so the new zip is
unambiguously current.

```
bash make_package.sh   →  "ZIP 30.8 MB"
```

- **Old zip:** 24,889,207 bytes (23.7 MB), Aug 22 18:09
- **New zip:** 32,289,597 bytes (30.8 MB), 40 entries (verified via
  `System.IO.Compression.ZipFile` listing)
- Contains `src/geo.py` ✓, `src/aggregate.py` ✓, `run_geo.py` ✓, `demo/geojson/`
  (4 files: `network.geojson`, `trajectories.geojson`, `desire_lines.geojson`,
  `queues.geojson`) ✓
- Excludes drone footage and model weights as designed (`README.md`, `WRITEUP.md`,
  `src/`, `demo/`, `models/README.md`, `notebooks/`, config/requirements/run scripts only)

---

## Aerial weights

Web research only — nothing downloaded (per boundaries). Two candidate pools per
`README.md` next-step #1: VisDrone-pretrained YOLO and DOTA-pretrained YOLO-OBB.

### VisDrone (fixes the class problem directly)

VisDrone's 10 classes — **pedestrian, people, bicycle, car, van, truck, tricycle,
awning-tricycle, bus, motor** — map onto this repo's classes as: pedestrian+people→
`person`, bicycle→`bicycle`, car→`car`, van→(no repo class today, would merge into
`car` or become new), truck→`truck`, bus→`bus`, **motor→`motorcycle`** (this is the fix:
COCO's "calls a two-wheeler a car from overhead" problem), and **tricycle +
awning-tricycle → `auto-rickshaw`** — a real detected class, replacing the current
size-inferred guess (WRITEUP.md §14 flags this explicitly).

Two ultralytics-format checkpoints found, both fine-tuned by the same author
(`dronefreak`) from Ultralytics' own YOLO base weights, both AGPL-3.0 (same licence
family as Ultralytics itself), both loadable directly via `ultralytics.YOLO(path)` —
they're standard `.pt` checkpoints, no custom loader needed, so ultralytics 8.4.126
should load them with no code change:

1. **[dronefreak/visdrone-yolov11s](https://huggingface.co/dronefreak/visdrone-yolov11s)**
   — YOLO11s fine-tuned on VisDrone2019-DET. `hf_hub_download(repo_id=
   "dronefreak/yolov11s-visdrone", filename="best.pt")` (note: the model-card slug and
   the `hf_hub_download` `repo_id` in its own example code differ by word order —
   verify the exact id before pulling). Same architecture generation as this repo's
   `yolo11x.pt`, but a smaller (`s`) variant — no mAP figure was shown on the card.
2. **[dronefreak/visdrone-yolov8x](https://huggingface.co/dronefreak/visdrone-yolov8x)**
   — YOLOv8x fine-tuned on the same VisDrone2019-DET set and same 10-class list,
   `best.pt`, quantified: mAP50 36.81%, mAP50-95 21.52%, precision 51.91%, recall
   39.78%. One architecture generation behind (v8 not v11) but size-matched to this
   repo's current `x`-variant model, and it's the only VisDrone weight found with a
   reported accuracy number.
   ([erbayat/yolov11n-visdrone](https://huggingface.co/erbayat/yolov11n-visdrone) also
   exists — YOLO11n, openrail licence — but it's the nano size and its card doesn't
   list classes or a base-model detail beyond "Ultralytics/YOLO11"; weaker pick than
   either above.)

### DOTA YOLO-OBB (does not fix the class problem — different problem)

Checked the actual DOTA v1.0 class list via [Ultralytics' own DOTA docs
page](https://docs.ultralytics.com/datasets/obb/dota-v2/): **plane, ship, storage tank,
baseball diamond, tennis court, basketball court, ground track field, harbor, bridge,
large vehicle, small vehicle, helicopter, roundabout, soccer ball field, swimming
pool.** There is no motorcycle/two-wheeler class and no pedestrian class at all — only
generic `large vehicle` / `small vehicle` buckets. So YOLO11-OBB pretrained on DOTA does
**not** solve next-step #1's stated class problem; it only gives oriented boxes, which
would help the *other* known limit ("vehicle width ~30% over-measured, ill-conditioned
half of the L/W solve") by removing axis-aligned-box inflation for car/truck-shaped
objects. Official `yolo11x-obb.pt` etc. auto-download "from the latest Ultralytics
release on first use" per [Ultralytics' OBB
docs](https://docs.ultralytics.com/tasks/obb/) — same mechanism as this repo's
`yolo11x.pt`, so the GitHub-release failure this repo already hit would likely repeat
(HF mirror fallback not confirmed to exist for the OBB weights specifically — not
checked). DOTA's dataset licence is **academic/non-commercial only**, per the same docs
page — a real constraint if this submission is ever used commercially.

### Ranking

1. **dronefreak/visdrone-yolov11s** — top pick. Directly targets the one class problem
   next-step #1 names (motor/tricycle confusion), same architecture family already in
   the repo, standard `.pt` load. Smaller model than the repo's current `x`-size is the
   only downside.
2. **dronefreak/visdrone-yolov8x** — second pick if size/accuracy matters more than
   architecture match; same class fix, quantified mAP, but one generation older.

DOTA-OBB is not ranked in the top 2 for solving the stated class problem — it's a
separate fix (box orientation / width measurement), worth revisiting once the class fix
is in, not before.

### Smallest experiment to prove a win

No need to re-cut video: `out/window_intersection.mp4` is already the extracted 120 s
intersection window used for the headline numbers. Run `YOLO('best.pt').predict()` (the
VisDrone weight, pick #1) over a handful of sampled frames from that file — a raw
detection pass, not full tracking — and compare the per-frame class-count share of
`motor` vs `car`/`van` against this repo's current report figures (two_wheeler 45.1%,
car 31.4%, from `out/report_intersection.json` → `counts`). A meaningful win looks like:
`motor` share rising above 45.1% and/or `car` share falling below 31.4%, i.e. objects
currently misclassified as `car` from directly overhead get correctly caught as
two-wheelers. This needs no tracking, no metric projection, no GPU minutes beyond a
single-pass inference on ~10–20 frames — cheapest possible test of the hypothesis before
committing to a full re-track.

Sources: [dronefreak/visdrone-yolov11s](https://huggingface.co/dronefreak/visdrone-yolov11s) ·
[dronefreak/visdrone-yolov8x](https://huggingface.co/dronefreak/visdrone-yolov8x) ·
[erbayat/yolov11n-visdrone](https://huggingface.co/erbayat/yolov11n-visdrone) ·
[VisDrone Detection Model Zoo collection](https://huggingface.co/collections/dronefreak/visdrone-detection-model-zoo) ·
[Ultralytics DOTA v1/v2 dataset docs](https://docs.ultralytics.com/datasets/obb/dota-v2/) ·
[Ultralytics OBB task docs](https://docs.ultralytics.com/tasks/obb/)

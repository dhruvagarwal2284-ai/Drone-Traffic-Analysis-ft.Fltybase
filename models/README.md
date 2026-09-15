# Models

No weights are committed. Fetch YOLO11x (114 MB) before running:

    curl -L -o yolo11x.pt "https://huggingface.co/Ultralytics/YOLO11/resolve/main/yolo11x.pt?download=true"

The GitHub release URL that Ultralytics auto-downloads from failed here with curl
error 35 (connection reset); the HuggingFace mirror above works.

## Default: VisDrone-pretrained (adopted, EXP1-3)

COCO is ground-level imagery. From directly overhead it routinely calls a
two-wheeler a car, which was the single largest source of class error in this
pipeline (see `EXP1.md`). The default detector is now an aerial-pretrained
VisDrone model:

    curl -L -o models/visdrone-yolov11s.pt "https://huggingface.co/dronefreak/visdrone-yolov11s/resolve/main/best.pt"

- **Weight**: `dronefreak/visdrone-yolov11s`, `best.pt` (18.3 MB), **licence
  AGPL-3.0**.
- Classes: `pedestrian`, `people`, `bicycle`, `car`, `van`, `truck`, `tricycle`,
  `awning-tricycle`, `bus`, `motor`, `others` (11; `others` dropped) — mapped
  to this repo's vocabulary in `src/track.py`'s `VISDRONE_MAP`.
- Not committed (like `yolo11x.pt`); fetch it before running `src/track.py`
  with `--weights models/visdrone-yolov11s.pt`.
- A larger variant, `dronefreak/visdrone-yolov8x` (130 MB, same licence), was
  also evaluated (`EXP1.md`) but the 11s pick was carried forward — accuracy
  was comparable and it is ~7x smaller/faster.

## COCO (still selectable)

The original ground-level-pretrained option remains available via
`--weights yolo11x.pt` (see `config.yaml`'s `detector:` block):

    curl -L -o yolo11x.pt "https://huggingface.co/Ultralytics/YOLO11/resolve/main/yolo11x.pt?download=true"

The GitHub release URL that Ultralytics auto-downloads from failed here with curl
error 35 (connection reset); the HuggingFace mirror above works.

## Alternative considered, not adopted

- **DOTA YOLO-OBB** — oriented boxes, which matter because vehicles sit at
  arbitrary angles and axis-aligned boxes inflate measured footprint size.
  Not adopted: DOTA's 15 classes have no motorcycle/pedestrian/bus-truck
  split, so it does not fix the two-wheeler-called-car problem this upgrade
  targets, and its dataset licence is academic/non-commercial only
  (see `PREP.md`).

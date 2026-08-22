# Models

No weights are committed. Fetch YOLO11x (114 MB) before running:

    curl -L -o yolo11x.pt "https://huggingface.co/Ultralytics/YOLO11/resolve/main/yolo11x.pt?download=true"

The GitHub release URL that Ultralytics auto-downloads from failed here with curl
error 35 (connection reset); the HuggingFace mirror above works.

## Recommended upgrade

COCO is ground-level imagery. From directly overhead it routinely calls a
two-wheeler a car, which is the single largest source of class error in this
pipeline. An aerial-pretrained model fixes it properly:

- **VisDrone** — has `pedestrian`, `motor`, `tricycle`, `awning-tricycle`
  (the last two are essentially auto-rickshaw, a class we could not recover).
- **DOTA YOLO-OBB** — oriented boxes, which matter because vehicles sit at
  arbitrary angles and axis-aligned boxes inflate measured footprint size.

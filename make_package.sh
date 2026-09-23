#!/usr/bin/env bash
# Build the submission ZIP. Excludes drone footage and model weights by design.
set -e
cd "$(dirname "$0")"

mkdir -p demo models notebooks

# Re-encode for the web instead of copying analysis-resolution originals raw
# (the main overlay is ~57 MB at native 4K; clips are ~1.2-1.8 MB each).
rm -f demo/conflict_*.mp4
ffmpeg -v error -i out/overlay_intersection.mp4 -vf scale=1280:-2 -crf 28 \
       -preset veryfast -pix_fmt yuv420p -y demo/example_output.mp4
for f in out/clips/*.mp4; do
  [ -e "$f" ] || continue
  ffmpeg -v error -i "$f" -vf scale=960:-2 -crf 30 \
         -preset veryfast -pix_fmt yuv420p -y "demo/$(basename "$f")"
done

python src/build_dashboard.py intersection --dest demo/dashboard.html --clip-base ""
cp out/report_intersection.json     demo/report.json
cp out/trajectories_intersection.parquet demo/trajectories.parquet
cp out/attributes_intersection.parquet   demo/attributes.parquet
cp out/objects_intersection.parquet      demo/objects.parquet

STAGE=$(mktemp -d)/traffic-analysis-agent
mkdir -p "$STAGE"
cp -r src demo notebooks trackers tests "$STAGE"/
rm -rf "$STAGE"/tests/__pycache__
mkdir -p "$STAGE"/models
cp models/README.md "$STAGE"/models/   # weights excluded by design (see models/README.md)

cp README.md WRITEUP.md NUMBERS-CHANGELOG.md INTERVIEW.md IMPROVEMENTS.md requirements.txt config.yaml run_attrs.py run_geo.py make_package.sh "$STAGE"/
mkdir -p "$STAGE"/demo/geojson && cp out/geojson/intersection/*.geojson "$STAGE"/demo/geojson/ 2>/dev/null || true
rm -rf "$STAGE"/src/__pycache__

powershell -NoProfile -Command \
  "Compress-Archive -Path '$(cygpath -w "$STAGE")' -DestinationPath '$(cygpath -w "$PWD")\traffic-analysis-agent.zip' -Force"
ls -la traffic-analysis-agent.zip | awk '{printf "ZIP %.1f MB\n", $5/1048576}'

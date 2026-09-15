#!/usr/bin/env bash
# Build the submission ZIP. Excludes drone footage and model weights by design.
set -e
cd "$(dirname "$0")"

mkdir -p demo models notebooks
cp out/overlay_intersection_web.mp4 demo/example_output.mp4
cp out/clips/*.mp4                  demo/ 2>/dev/null || true
cp out/dashboard_intersection.html  demo/dashboard.html
cp out/report_intersection.json     demo/report.json
cp out/trajectories_intersection.parquet demo/trajectories.parquet
cp out/attributes_intersection.parquet   demo/attributes.parquet
cp out/objects_intersection.parquet      demo/objects.parquet

STAGE=$(mktemp -d)/traffic-analysis-agent
mkdir -p "$STAGE"
cp -r src demo models notebooks "$STAGE"/

cp README.md WRITEUP.md requirements.txt config.yaml run_attrs.py run_geo.py make_package.sh "$STAGE"/
mkdir -p "$STAGE"/demo/geojson && cp out/geojson/intersection/*.geojson "$STAGE"/demo/geojson/ 2>/dev/null || true
rm -rf "$STAGE"/src/__pycache__

powershell -NoProfile -Command \
  "Compress-Archive -Path '$(cygpath -w "$STAGE")' -DestinationPath '$(cygpath -w "$PWD")\traffic-analysis-agent.zip' -Force"
ls -la traffic-analysis-agent.zip | awk '{printf "ZIP %.1f MB\n", $5/1048576}'

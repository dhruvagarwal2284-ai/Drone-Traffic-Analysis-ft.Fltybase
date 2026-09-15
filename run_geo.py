"""Level 4 -- map-native: georeference, map-match, and export on real geometry."""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, "src")
import geo  # noqa: E402
import insights as ins  # noqa: E402
import telemetry as tel  # noqa: E402
from geometry import GroundPlane  # noqa: E402

ROOT = Path(".")
TAG = sys.argv[1] if len(sys.argv) > 1 else "intersection"
VIDEO, T0 = "Intersection_Merged-002", 240.0

traj = pd.read_parquet(f"out/trajectories_{TAG}.parquet")
objs = pd.read_parquet(f"out/objects_{TAG}.parquet")
rep = json.load(open(f"out/report_{TAG}.json"))
tm = tel.load(Path(f"{VIDEO}.MP4"), Path("out/cache"))
row = tm.iloc[int(T0 * 30000 / 1001)]

gp = GroundPlane.from_telemetry(3840, 2160, row.focal_len, row.rel_alt,
                               row.gb_yaw, row.gb_pitch, row.gb_roll, zoom=row.dzoom)
gp.scale = rep["quality"]["calibration"]["scale"]
gp.set_origin(np.array([[1920.0, 1080.0]]))
frame = geo.Frame(row.lat, row.lon, gp.origin)

# Motion-derived approaches are joined to the map match, so each moment has
# both the OSM road identity and the vehicle's approach through the junction.
legs = ins.find_legs(traj)
moves = ins.delay(traj, ins.assign_movements(traj, legs))

# --- network, registered to the trajectories -------------------------------
allL = geo.load_network(Path("out/osm_raw.json"), frame, extent=200)
major = [L for L in allL if L["highway"] in ("primary", "secondary", "tertiary", "trunk")]
shift, align = geo.refine_alignment(traj, major, search_m=25, coarse=1.0, fine=0.25)
links = geo.apply_offset(allL, shift)          # apply to the FULL network
major_al = geo.apply_offset(major, shift)
print("alignment:", json.dumps(align))

# --- map-match -------------------------------------------------------------
print("map-matching ...", flush=True)
m = geo.match(traj, [L for L in links if L["highway"] != "service"],
              max_dist=20.0, sample_every=1)
m = geo.assign_lanes(m)
m = m.merge(moves[["track_id", "origin", "dest", "movement"]],
            on="track_id", how="left")
matched = m[m.link_idx >= 0]
print(f"  matched {len(matched):,}/{len(m):,} samples ({100*len(matched)/max(len(m),1):.1f}%)")
print("  by link class:", matched.link_class.value_counts().to_dict())
print("  named links  :", matched.link_name.value_counts().head(3).to_dict())
print("  lanes        :", matched.lane.value_counts().sort_index().to_dict())
print("  directions   :", matched.link_dir.value_counts().to_dict())
m.to_parquet(f"out/mapmatch_{TAG}.parquet", index=False)
print(f"  -> out/mapmatch_{TAG}.parquet ({len(m):,} 10-fps bindings)")

# per-lane metrics on real geometry
per_lane = (matched.groupby(["link_id", "link_name", "carriageway", "lane"])
            .agg(samples=("speed_kph", "size"),
                 vehicles=("track_id", "nunique"),
                 mean_kph=("speed_kph", "mean"),
                 p85_kph=("speed_kph", lambda s: s.quantile(.85)))
            .reset_index())
per_lane = per_lane[per_lane.vehicles >= 5].sort_values("vehicles", ascending=False)
print("\nper-lane metrics on real link geometry (top 8):")
print(per_lane.head(8).round(1).to_string(index=False))

# --- map-native exports ----------------------------------------------------
queues = rep["aggregate"]["queues"]

exports = {
    "network": geo.network_geojson(links),
    "trajectories": geo.trajectories_geojson(traj, frame, objs),
    "desire_lines": geo.desire_lines_geojson(moves, traj, frame, legs),
    "queues": geo.queues_geojson(queues, legs, frame),
}
Path("out/geojson").mkdir(exist_ok=True)
for k, v in exports.items():
    p = Path(f"out/geojson/{k}.geojson")
    p.write_text(json.dumps(v), encoding="utf-8")
    print(f"  {k:14s} {len(v['features']):4d} features -> {p} ({p.stat().st_size/1024:.0f} KB)")

# bundle for the dashboard map
lat0, lon0 = frame.to_wgs84(0.0, 0.0)
bundle = {
    "alignment": align,
    "binding": {"cadence_hz": 10.0,
                "fields": ["link", "approach", "direction", "lane"]},
    "origin": {"lat": float(lat0), "lon": float(lon0)},
    "network": [{"id": L["id"], "name": L["name"], "highway": L["highway"],
                 "x": [round(float(v), 1) for v in L["x"]],
                 "y": [round(float(v), 1) for v in L["y"]]}
                for L in links],
    "per_lane": per_lane.round(2).to_dict("records"),
    "match_rate": round(100 * len(matched) / max(len(m), 1), 1),
    "matched_by_class": {str(k): int(v) for k, v in matched.link_class.value_counts().items()},
    "desire": exports["desire_lines"],
    "queues_geo": exports["queues"],
}
Path(f"out/mapnative_{TAG}.json").write_text(json.dumps(bundle), encoding="utf-8")
np.save("out/geo_shift.npy", shift)
print("\n-> out/mapnative_%s.json" % TAG)

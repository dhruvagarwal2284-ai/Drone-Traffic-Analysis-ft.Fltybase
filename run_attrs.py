import sys, json; sys.path.insert(0,'src')
import numpy as np, pandas as pd
from pathlib import Path
import telemetry as tel, attributes as at
from geometry import GroundPlane

tag = sys.argv[1] if len(sys.argv) > 1 else "intersection"

det=pd.read_parquet(f'out/detections_{tag}.parquet')
traj=pd.read_parquet(f'out/trajectories_{tag}.parquet')
tm=tel.load(Path('Intersection_Merged-002.MP4'), Path('out/cache'))
row=tm.iloc[int(240*30000/1001)]
gp=GroundPlane.from_telemetry(3840,2160,row.focal_len,row.rel_alt,row.gb_yaw,row.gb_pitch,row.gb_roll,zoom=row.dzoom)
gp.scale=json.load(open(f'out/report_{tag}.json'))['quality']['calibration']['scale']
gp.set_origin(np.array([[1920.,1080.]]))
det=det[det.track_id.isin(traj.track_id.unique())]
a=at.build(f'out/window_{tag}.mp4', det, traj, gp, max_samples=18)
a.to_parquet(f'out/attributes_{tag}.parquet', index=False)
print('tracks with attributes:', len(a))
print('\n== colour ==');    print(a.groupby('colour').size().sort_values(ascending=False).to_string())
print('\n== body type =='); print(a.groupby('body_type').agg(n=('L_m','size'), L=('L_m','median'), W=('W_m','median')).round(2).to_string())
print('\n== size class =='); print(a.groupby('size_class').size().to_string())
print('\n== measured dims by detector class ==')
print(a.groupby('cls')[['L_m','W_m']].median().round(2).to_string())

from pathlib import Path
import os
import numpy as np,json
from scipy.optimize import least_squares
B=Path(os.environ['REALTOSIM_CASE_DIR']).resolve();p=json.loads((B/'assets/layout_fit.json').read_text());obs={'00':[430,326],'05':[592,195],'06':[520,433]}
def project(k,pt):
 c=p['cameras'][k];M=np.array(c['camera_to_world_blender']);v=np.diag([1,-1,-1])@M[:3,:3].T@(pt-M[:3,3]);u=np.array(c['intrinsics'])@v;return u[:2]/u[2]
r=least_squares(lambda pt:np.concatenate([project(k,pt)-v for k,v in obs.items()]),[-.6,-.2,.96]);h=np.array([-.508,p['table_dimensions_m'][1]/2+.045-.055,r.x[2]]);d=r.x-h;w=float(np.linalg.norm(d[:2]));a=float(np.rad2deg(np.arctan2(-d[1],-d[0])))
report={'observed_free_lower_corner_pixels':obs,'point_world_m':r.x.tolist(),'hinge_assumption_world_m':h.tolist(),'leaf_width_m':w,'yaw_degrees':a,'bottom_edge_z_m':float(r.x[2]),'reprojection_errors_px':{k:float(np.linalg.norm(project(k,r.x)-uv)) for k,uv in obs.items()},'basis':'same three fitted desk cameras; manually located sash corner, approximate shared state','unobserved_window_top':'estimated 2.22m'}
(B/'assets/window_fit.json').write_text(json.dumps(report,indent=2));print(report)

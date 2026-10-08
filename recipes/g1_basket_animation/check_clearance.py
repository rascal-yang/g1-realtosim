import os
import bpy,json
from pathlib import Path
from mathutils import Vector
A=Path(os.environ['G1_ANIMATION_DIR']);B=Path(os.environ['G1_TASK_DIR']);bpy.ops.wm.open_mainfile(filepath=str(A/'assets/g1_basket_authored.blend'));s=bpy.context.scene;mapping=json.loads(s['rig_mapping_json']);roots=[bpy.data.objects[v] for k,v in mapping.items() if '/' in k or any(x in k for x in ['elbow','wrist'])];meshes=set(o for r in roots for o in [r,*r.children_recursive] if o.type=='MESH' and not o.hide_render)
spec=json.loads((B/'motion_spec.json').read_text());low,high=spec['tabletop_aabb_m'];alerts=[];margin=999;tested=0
for f in range(1,s.frame_end+1):
 s.frame_set(f);bpy.context.view_layer.update()
 for o in meshes:
  pts=[o.matrix_world@Vector(v) for v in o.bound_box];lo=[min(p[i] for p in pts) for i in range(3)];hi=[max(p[i] for p in pts) for i in range(3)];tested+=1
  if hi[0]>low[0] and lo[0]<high[0] and hi[1]>low[1] and lo[1]<high[1]:
   if lo[2]>high[2]:margin=min(margin,lo[2]-high[2])
   elif hi[2]>low[2]:alerts.append({'frame':f,'mesh':o.name,'aabb_min':lo,'aabb_max':hi})
report={'fps_checked':s.render.fps,'frames':s.frame_end,'mesh_aabb_tests':tested,'aabb_table_overlap_count':len(alerts),'first_alerts':alerts[:10],'min_above_table_margin_m':margin,'method':'conservative whole visual mesh AABB vs designed112x60cm tabletop proxy z0.70..0.75 at every authored frame','limitations':'Geometric screen only; does not certify self-collision, basket penetration, continuous-time swept motion, support forces or balance'};(B/'reports/animation_clearance.json').write_text(json.dumps(report,indent=2));print(report);assert not alerts,'Hand/forearm tabletop overlap: reject geometry'

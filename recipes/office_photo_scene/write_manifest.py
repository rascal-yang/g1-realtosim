from pathlib import Path
import os
import json,numpy as np
from pxr import Usd,UsdGeom,Gf
B=Path(os.environ['REALTOSIM_CASE_DIR']).resolve();A=B/'assets';s=Usd.Stage.Open(str(A/'scene.usda'));cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'],useExtentsHint=False)
def bounds(path):
 b=cache.ComputeWorldBound(s.GetPrimAtPath(path)).ComputeAlignedRange();return {'min_m':list(b.GetMin()),'max_m':list(b.GetMax()),'size_m':list(b.GetSize())}
entries=[]
for name,path,route,reference,material,texture,placement,limits in [
('desk','/World/Scene/Desk','Blender + joint camera/layout fit',['00','05','06'],'laminated light wood; dark powder-coated steel','photo-extracted wood color, estimated normal/roughness','aligned to three manually annotated photo views','absolute metric scale assumes 0.75m height'),
('basket','/World/Scene/Basket','Blender procedural woven strands',['00','02','03','05','06'],'estimated orange-brown wicker','explicit weave, bound rim and handle geometry','photo-guided tabletop position','rigid proxy; detailed weave differs from individual real strands'),
('plush','/World/Plush','reuse Hunyuan3D-2 + photo-texture corrected asset',['01','00','02','05','06'],'estimated fabric PBR','original generated UV and photo-projected visible features','photo-guided lying orientation and scale','rigid proxy; limbs/face/hidden surface remain approximate'),
('window','/World/Scene/Window','Blender frame/sash + Isaac OmniGlass',['00','05','06'],'painted frame, metal hardware, dielectric glass','geometry and scalar material; exterior photo card','sill and free sash corner fitted across three views; camera/absolute scale remain approximate','unseen full window size inferred; outdoor is not reconstructed geometry'),
('room','/World/Scene/Architecture','Blender planes and tile grid',['00','02','05','06'],'warm painted plaster; rubber tile PBR','photographed blue-fleck floor, estimated normals','walls/table relationship from fitted cameras; column manually inferred','room extent and tile dimensions estimated; wall stains not individually reconstructed'),
('appliance','/World/Scene/Appliance','Blender approximate shell',['05'],'enamel, dark front panel, black components','scalar PBR; partly observed','estimated from photo 05','partially seen equipment, no internal structure/function modeled'),
('inferred_enclosure','/World/Scene/Inferred_enclosure','Blender enclosure completion',[],'painted wall','plain material','assumed unseen boundaries','not observed; separate group can be disabled')]:
 p=s.GetPrimAtPath(path);xf=UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default());actual=bounds(path)
 entries.append({'id':name,'prim_path':path,'route':route,'reference_photos':['photo_'+k+'.jpg' for k in reference],'size':{'status':'uncalibrated','basis':'photo proportions; table height assumed','exported_world_bounds':actual},'material':{'status':'estimated','description':material},'texture':{'status':'visually_reviewed','description':texture},'placement':{'status':'fitted' if name=='desk' else 'estimated','basis':placement,'world_transform_usd_row_vector':list(map(list,xf))},'limitations':limits})
report={'units':'meters','up_axis':'Z','scene_origin':'center of desk projected onto floor','objects':entries,'bottle':'bottle/manifest.json; optional scene_with_bottle.usda adds an explicitly designed placement','size_accuracy':'no measured absolute scale available','four_requirement_rule':'size, material, texture, placement recorded independently; visual acceptance does not establish real physical accuracy'}
(A/'asset_manifest.json').write_text(json.dumps(report,indent=2));print('MANIFEST',len(entries),'object groups')

from pathlib import Path
import os
import json,shutil,cv2,numpy as np
B=Path(os.environ['REALTOSIM_CASE_DIR']).resolve();T=B/'assets/textures';T.mkdir(exist_ok=True)
rng=np.random.default_rng(31)
# Rectify visible desk top, extract a clean patch and remove only broad illumination.
p=cv2.imread(str(B/'inputs/photo_05.jpg'))
H=cv2.getPerspectiveTransform(np.float32([[821,251],[1498,476],[1137,668],[522,340]]),np.float32([[0,0],[2048,0],[2048,1024],[0,1024]]))
r=cv2.warpPerspective(p,H,(2048,1024));cv2.imwrite(str(B/'reports/rectified_table.jpg'),r)
patch=r[200:850,1500:2000].astype(np.float32)
low=cv2.GaussianBlur(patch,(0,0),65);base=patch/np.maximum(low,1)*np.array([157,181,199])
base=np.clip(base,0,255).astype('uint8');base=np.concatenate([base,base[:,::-1]],1);base=np.concatenate([base,base[::-1]],0)
cv2.imwrite(str(T/'wood_basecolor.png'),base)
g=cv2.cvtColor(base,cv2.COLOR_BGR2GRAY).astype(float)/255
for name,gray,strength,rough in [('wood',g,.07,.38)]:
 gx=cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=3)*strength;gy=cv2.Sobel(gray,cv2.CV_64F,0,1,ksize=3)*strength;n=np.stack([-gx,-gy,np.ones_like(gx)],-1);n/=np.linalg.norm(n,axis=-1,keepdims=True)
 cv2.imwrite(str(T/f'{name}_normal.png'),((n[...,::-1]+1)*127.5).astype('uint8'));cv2.imwrite(str(T/f'{name}_roughness.png'),np.uint8(np.clip(rough+.08*(gray-gray.mean()),0,1)*255))
p=cv2.imread(str(B/'inputs/photo_02.jpg'));patch=p[700:1080,1110:1510].astype(np.float32)
low=cv2.GaussianBlur(patch,(0,0),45);base=np.clip(patch/np.maximum(low,1)*np.array([53,49,43]),0,255).astype('uint8')
base=np.concatenate([base,base[:,::-1]],1);base=np.concatenate([base,base[::-1]],0)
cv2.imwrite(str(T/'rubber_basecolor.png'),base)
g=cv2.cvtColor(base,cv2.COLOR_BGR2GRAY).astype(float)/255;gx=cv2.Sobel(g,cv2.CV_64F,1,0,ksize=3)*.22;gy=cv2.Sobel(g,cv2.CV_64F,0,1,ksize=3)*.22;n=np.stack([-gx,-gy,np.ones_like(gx)],-1);n/=np.linalg.norm(n,axis=-1,keepdims=True)
cv2.imwrite(str(T/'rubber_normal.png'),((n[...,::-1]+1)*127.5).astype('uint8'));cv2.imwrite(str(T/'rubber_roughness.png'),np.uint8(np.clip(.76+.25*(g-g.mean()),0,1)*255))
p=cv2.imread(str(B/'inputs/photo_00.jpg'));cv2.imwrite(str(T/'exterior_reference.jpg'),p[0:200,138:387])
# Original photographed bottle label: observed central face, inferred white surroundings.
p=cv2.imread(str(B/'inputs/photo_04.jpg'));crop=p[638:878,603:780]
label=np.full((512,1536,3),238,np.uint8);label[:,510:1022]=cv2.resize(crop,(512,512));cv2.imwrite(str(T/'bottle_label.png'),label)
src=Path(os.environ['REALTOSIM_PLUSH_ASSET']);dest=B/'assets/plush';dest.mkdir(exist_ok=True)
for n in ['plush.usda','collision_hulls.json','asset_manifest.json','generation.json']:shutil.copy2(src/n,dest/n)
shutil.copytree(src/'visuals',dest/'visuals',dirs_exist_ok=True)
# Preserve the actual original bytes in addition to normalized input images.
for i,f in enumerate(sorted(Path(os.environ['REALTOSIM_SOURCE_MEDIA']).iterdir())):
 if f.is_file():shutil.copy2(f,B/'inputs'/f'original_{i:02d}.bin')
plan={'task':'photo constrained office scene reconstruction','scale':'assumed 0.75m desk height; no true metric measurement','objects':[
{'id':'room','route':'procedural + fitted geometry','reason':'planar walls and photo layout','evidence':['photo_00','photo_05','photo_06'],'checks':['window opening/sill','wall return','column position']},
{'id':'desk','route':'Blender procedural + three-view fitted proportions + photo texture','reason':'precise box structure, edge bevels, separate steel frame','dimensions_source':'layout_fit.json'},
{'id':'basket','route':'Blender procedural woven geometry','reason':'real open interior and weave topology; close-up supports structure better than single fused generated mesh','reference':'photo_03','physics':'rigid approximation, uncalibrated'},
{'id':'plush','route':'reuse Hunyuan3D-2 reconstruction + placement/scale adjustment','reason':'same photographed toy; existing generated geometry and photo-corrected texture','physics':'rigid proxy, explicitly no soft response'},
{'id':'floor','route':'procedural tile geometry + real photographed granules','reason':'repeatable measured-relative seams and photo material'},
{'id':'window','route':'procedural multi-part frame, gasket, hinged leaf, glass','reason':'retain opening and correct occlusion rather than texture-only window'},
{'id':'appliance','route':'procedural approximate shell','reason':'partially observed secondary equipment; hidden structure unknown'},
{'id':'bottle','route':'procedural lathe + photo label','reason':'axisymmetric shell; single close-up, no observed room placement','placement':'separate optional asset; excluded from reference scene'}],
'validation':['three fitted photo cameras','target Isaac rendering','collider checks','rigid object settling','dependency resolution'],
'new_image_to_3d_calls':0,'reuse_note':'existing Hunyuan result reused; no new model call claimed'}
(B/'route_plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2))
print('Prepared textures and reusable assets')

from pathlib import Path
import os
import json,math,cv2,numpy as np
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Sdf,Gf
B=Path(os.environ['REALTOSIM_CASE_DIR']).resolve();A=B/'assets';O=A/'bottle';O.mkdir(exist_ok=True)
im=cv2.imread(str(B/'inputs/photo_04.jpg'));src=np.float32([[637,689],[889,678],[906,897],[642,906]]);H=cv2.getPerspectiveTransform(src,np.float32([[0,0],[599,0],[599,449],[0,449]]));patch=cv2.warpPerspective(im,H,(600,450));label=np.full((450,1500,3),235,np.uint8);label[:,450:1050]=patch;cv2.imwrite(str(O/'label_observed_face.png'),label)
s=Usd.Stage.CreateNew(str(O/'bottle.usda'));root=UsdGeom.Xform.Define(s,'/Bottle');s.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageMetersPerUnit(s,1);UsdGeom.SetStageUpAxis(s,'Z');UsdPhysics.RigidBodyAPI.Apply(root.GetPrim());UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(.25)
def mat(name,color,rough,glass=False,tex=False):
 p='/Bottle/Materials/'+name;m=UsdShade.Material.Define(s,p);sh=UsdShade.Shader.Define(s,p+'/Preview');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(color);sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
 if glass:
  md=UsdShade.Shader.Define(s,p+'/MDL');md.CreateImplementationSourceAttr('sourceAsset');md.SetSourceAsset(Sdf.AssetPath('OmniGlass.mdl'),'mdl');md.SetSourceAssetSubIdentifier('OmniGlass','mdl');md.CreateInput('glass_color',Sdf.ValueTypeNames.Color3f).Set(color);md.CreateInput('glass_ior',Sdf.ValueTypeNames.Float).Set(1.57);md.CreateInput('thin_walled',Sdf.ValueTypeNames.Bool).Set(True);md.CreateInput('frosting_roughness',Sdf.ValueTypeNames.Float).Set(rough);m.CreateSurfaceOutput('mdl').ConnectToSource(md.ConnectableAPI(),'out')
 if tex:
  uv=UsdShade.Shader.Define(s,p+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');tx=UsdShade.Shader.Define(s,p+'/Texture');tx.CreateIdAttr('UsdUVTexture');tx.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('label_observed_face.png'));tx.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tx.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(),'rgb')
 return m
pet=mat('Clear_PET',(.99,.999,1),.06,True);cap=mat('White_HDPE',(.82,.84,.83),.42);paper=mat('Photographed_label',(.85,.85,.85),.43,tex=True)
def revolve(name,zs,rs,material,waves=False):
 n=128;verts=[];uv=[];faces=[]
 for i,(z,r) in enumerate(zip(zs,rs)):
  for j in range(n+1):
   a=2*np.pi*j/n;rr=r
   if waves and .012<z<.119:rr+=.0011*np.sin(z*2*np.pi/.018+1.4*np.sin(5*a))
   verts.append([rr*np.cos(a),rr*np.sin(a),z]);uv.append([j/n,(z-zs[0])/max(zs[-1]-zs[0],1e-6)])
 for i in range(len(zs)-1):
  for j in range(n):a=i*(n+1)+j;faces.append([a,a+1,a+n+2,a+n+1])
 m=UsdGeom.Mesh.Define(s,'/Bottle/'+name);m.CreatePointsAttr(verts);m.CreateFaceVertexCountsAttr([4]*len(faces));m.CreateFaceVertexIndicesAttr(np.array(faces).ravel().tolist());m.CreateSubdivisionSchemeAttr('none');m.CreateDoubleSidedAttr(True);UsdGeom.PrimvarsAPI(m).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set(uv);UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material)
 m.GetPrim().CreateAttribute('semantic:Semantics:params:semanticType',Sdf.ValueTypeNames.String).Set('class');m.GetPrim().CreateAttribute('semantic:Semantics:params:semanticData',Sdf.ValueTypeNames.String).Set('water_bottle');m.GetPrim().AddAppliedSchema('SemanticsAPI:Semantics');return m
zs=np.r_[0,.003,np.linspace(.008,.17,90),.184,.195,.21,.22];rs=np.interp(zs,[0,.003,.009,.17,.184,.195,.21,.22],[.025,.029,.030,.030,.028,.022,.014,.014]);body=revolve('PET_shell',zs,rs,pet,True)
revolve('Label',np.array([.117,.171]),np.array([.0304,.0304]),paper)
revolve('Cap',np.array([.216,.217,.235,.237,.237]),np.array([.0145,.016,.016,.0145,0]),cap)
for i in range(64):
 a=2*np.pi*i/64;c=UsdGeom.Cylinder.Define(s,f'/Bottle/Cap_ridges/Ridge_{i:02d}');c.CreateRadiusAttr(.00042);c.CreateHeightAttr(.016);c.AddTranslateOp().Set(Gf.Vec3d(.016*np.cos(a),.016*np.sin(a),.227));UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(cap)
coll=UsdGeom.Cylinder.Define(s,'/Bottle/Collision');coll.CreateRadiusAttr(.030);coll.CreateHeightAttr(.235);coll.AddTranslateOp().Set(Gf.Vec3d(0,0,.1175));coll.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(coll.GetPrim())
phys=UsdShade.Material.Define(s,'/Bottle/Materials/EstimatedContact');p=UsdPhysics.MaterialAPI.Apply(phys.GetPrim());p.CreateStaticFrictionAttr(.6);p.CreateDynamicFrictionAttr(.5);p.CreateRestitutionAttr(.02);UsdShade.MaterialBindingAPI.Apply(coll.GetPrim()).Bind(phys,materialPurpose='physics')
s.GetRootLayer().Save();(O/'manifest.json').write_text(json.dumps({'source':'photo_04.jpg','dimensions_m':[.062,.062,.237],'dimensions_status':'estimated','mass_kg':.25,'mass_status':'estimated','label':'photographed observed face, unobserved remainder white/inferred','physics':'rigid PET proxy, no liquid simulation','placement':'not observed in room photos; standalone optional asset'},indent=2))
# Optional room variant, never presented as the observed bottle placement.
s=Usd.Stage.CreateNew(str(A/'scene_with_bottle.usda'));w=UsdGeom.Xform.Define(s,'/World');w.GetPrim().GetReferences().AddReference('scene.usda','/World');s.SetDefaultPrim(w.GetPrim());UsdGeom.SetStageMetersPerUnit(s,1);UsdGeom.SetStageUpAxis(s,'Z');o=UsdGeom.Xform.Define(s,'/World/Bottle');o.GetPrim().GetReferences().AddReference('bottle/bottle.usda','/Bottle');o.AddTranslateOp().Set(Gf.Vec3d(.36,-.15,.751));o.GetPrim().SetCustomDataByKey('placement_source','designed optional position; absent from room reference photos');s.GetRootLayer().Save();print('BOTTLE_READY')

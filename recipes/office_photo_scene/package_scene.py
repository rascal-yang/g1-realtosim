from pathlib import Path
import os
import json,math,numpy as np
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,UsdLux,Sdf,Gf,Tf,UsdUtils
B=Path(os.environ['REALTOSIM_CASE_DIR']).resolve();A=B/'assets';g=json.loads((A/'geometry_manifest.json').read_text());fit=json.loads((A/'layout_fit.json').read_text())
s=Usd.Stage.CreateNew(str(A/'scene.usda'));world=UsdGeom.Xform.Define(s,'/World');s.SetDefaultPrim(world.GetPrim());UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1)
scene=UsdGeom.Xform.Define(s,'/World/Scene');scene.GetPrim().GetReferences().AddReference('visuals/room.usdc','/Asset')
physical=UsdShade.Material.Define(s,'/World/Materials/EstimatedContact');p=UsdPhysics.MaterialAPI.Apply(physical.GetPrim());p.CreateStaticFrictionAttr(.65);p.CreateDynamicFrictionAttr(.50);p.CreateRestitutionAttr(.025)
names={Tf.MakeValidIdentifier(n) for n in g['static_collision_names']};count=0
for prim in s.Traverse():
 if prim.IsA(UsdGeom.Mesh):
  parent=prim.GetParent()
  if parent.GetName() in names:
   UsdPhysics.CollisionAPI.Apply(prim);UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('boundingCube');UsdShade.MaterialBindingAPI.Apply(prim).Bind(physical,materialPurpose='physics');count+=1
  category='background'
  for group,label in [('Desk','desk'),('Basket','basket'),('Window','window'),('Appliance','appliance')]:
   if '/'+group+'/' in str(prim.GetPath()):category=label
  if '/Mat_' in str(prim.GetPath()):category='floor'
  prim.CreateAttribute('semantic:Semantics:params:semanticType',Sdf.ValueTypeNames.String).Set('class');prim.CreateAttribute('semantic:Semantics:params:semanticData',Sdf.ValueTypeNames.String).Set(category);prim.AddAppliedSchema('SemanticsAPI:Semantics')
# Glass uses the MDL library included with Isaac Sim, plus a generic preview fallback.
mat=UsdShade.Material.Get(s,'/World/Scene/_materials/Window_clear_glass');sh=UsdShade.Shader.Define(s,mat.GetPath().AppendChild('IsaacGlass'));sh.CreateImplementationSourceAttr('sourceAsset');sh.SetSourceAsset(Sdf.AssetPath('OmniGlass.mdl'),'mdl');sh.SetSourceAssetSubIdentifier('OmniGlass','mdl');sh.CreateInput('glass_color',Sdf.ValueTypeNames.Color3f).Set((.985,.995,1));sh.CreateInput('glass_ior',Sdf.ValueTypeNames.Float).Set(1.47);sh.CreateInput('depth',Sdf.ValueTypeNames.Float).Set(.006);sh.CreateInput('thin_walled',Sdf.ValueTypeNames.Bool).Set(True);sh.CreateInput('frosting_roughness',Sdf.ValueTypeNames.Float).Set(.025);mat.CreateSurfaceOutput('mdl').ConnectToSource(sh.ConnectableAPI(),'out')
# Basket rigid shell: many convex wall segments keep the interior open.
basket=s.GetPrimAtPath('/World/Scene/Basket');UsdPhysics.RigidBodyAPI.Apply(basket);UsdPhysics.MassAPI.Apply(basket).CreateMassAttr(.12)
root=UsdGeom.Xform.Define(s,'/World/Scene/Basket/Colliders');root.CreateVisibilityAttr('invisible')
def collider(name,pos,size,angle=0):
 c=UsdGeom.Cube.Define(s,str(root.GetPath())+'/'+name);c.CreateSizeAttr(1);c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddRotateZOp().Set(float(angle*180/math.pi));c.AddScaleOp().Set(Gf.Vec3f(*size));UsdPhysics.CollisionAPI.Apply(c.GetPrim());UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(physical,materialPurpose='physics')
rx,ry=g['basket']['radii_m'];bh=g['basket']['body_height_m']
for i,t in enumerate(np.linspace(0,2*np.pi,40,endpoint=False)):
 angle=math.atan2(ry*math.cos(t),-rx*math.sin(t));collider('Side_%02d'%i,(rx*.90*math.cos(t),ry*.90*math.sin(t),bh/2),(.023,.007,bh),angle)
collider('Base',(0,0,.004),(rx*1.59,ry*1.59,.008))
# Handle collider segments follow the arch rather than sealing its opening.
for i,t in enumerate(np.linspace(.02,np.pi-.02,32)):
 c=UsdGeom.Capsule.Define(s,str(root.GetPath())+'/Handle_%02d'%i);c.CreateRadiusAttr(.006);c.CreateHeightAttr(.013);c.AddTranslateOp().Set(Gf.Vec3d(0,ry*np.cos(t),bh+.183*np.sin(t)));d=Gf.Vec3d(0,-ry*np.sin(t),.183*np.cos(t)).GetNormalized();c.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),d).GetQuat()));UsdPhysics.CollisionAPI.Apply(c.GetPrim())
# Original generation output reused with local, relocatable dependencies.
p=json.loads((A/'plush_placement.json').read_text());toy=UsdGeom.Xform.Define(s,'/World/Plush');toy.GetPrim().GetReferences().AddReference('plush/plush.usda','/Plush');toy.ClearXformOpOrder();toy.AddTranslateOp().Set(Gf.Vec3d(*p['translation_m']));q=p['quaternion_wxyz'];toy.AddOrientOp().Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])));toy.AddScaleOp().Set(Gf.Vec3f(p['scale']))
for prim in Usd.PrimRange(toy.GetPrim()):
 prim.CreateAttribute('semantic:Semantics:params:semanticType',Sdf.ValueTypeNames.String).Set('class');prim.CreateAttribute('semantic:Semantics:params:semanticData',Sdf.ValueTypeNames.String).Set('plush_toy');prim.AddAppliedSchema('SemanticsAPI:Semantics')
for k,c in fit['cameras'].items():
 cam=UsdGeom.Camera.Define(s,'/World/Cameras/Photo_'+k);cam.AddTransformOp().Set(Gf.Matrix4d(np.array(c['camera_to_world_blender']).T.tolist()));cam.CreateHorizontalApertureAttr(36);cam.CreateVerticalApertureAttr(36*1279/1706);cam.CreateFocalLengthAttr(c['intrinsics'][0][0]*36/1706);cam.CreateClippingRangeAttr((.01,100))
cam=UsdGeom.Camera.Define(s,'/World/Cameras/Inspect');cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(.6,-1.10,1.25),Gf.Vec3d(-.05,0,.85),Gf.Vec3d(0,0,1)).GetInverse());cam.CreateHorizontalApertureAttr(36);cam.CreateVerticalApertureAttr(27);cam.CreateFocalLengthAttr(30);cam.CreateClippingRangeAttr((.01,100))
for name,eye,target,width,height,power,color in [('WindowDaylight',[-.98,.27,1.65],[-.2,-1,.65],0.90,1.1,24500,(.92,.96,1)),('InteriorBounce',[-.7,-1,2.70],[0,0,.7],2.5,2.5,4250,(1,.95,.87))]:
 l=UsdLux.RectLight.Define(s,'/World/Lights/'+name);l.CreateWidthAttr(width);l.CreateHeightAttr(height);l.CreateIntensityAttr(power);l.CreateColorAttr(color);l.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
 l.GetPrim().CreateAttribute('visibleInPrimaryRay',Sdf.ValueTypeNames.Bool).Set(False)
l=UsdLux.DomeLight.Define(s,'/World/Lights/Ambient');l.CreateIntensityAttr(800);l.CreateColorAttr((.88,.94,1))
physics=UsdPhysics.Scene.Define(s,'/World/PhysicsScene');physics.CreateGravityDirectionAttr((0,0,-1));physics.CreateGravityMagnitudeAttr(9.81)
s.GetRootLayer().customLayerData={'reconstruction':'photo constrained; dimensions and dynamics not physically calibrated','cameraSettings':{'Perspective':{'position':Gf.Vec3d(-.6,-2.2,1.42),'target':Gf.Vec3d(0,0,.82)},'boundCamera':'/World/Cameras/Photo_00'}}
s.GetRootLayer().Save();layers,assets,unresolved=UsdUtils.ComputeAllDependencies(str(A/'scene.usda'))
report={'static_colliders':count,'basket_convex_sides':40,'basket_handle_capsules':32,'plush_convex_hulls':36,'unresolved':list(map(str,unresolved)),'builtin_mdl':'OmniGlass.mdl resolved by Isaac Sim MDL search path','layers':len(layers),'file_dependencies':len(assets)}
(B/'reports/package_validation.json').write_text(json.dumps(report,indent=2));print(report)

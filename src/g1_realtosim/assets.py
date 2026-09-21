"""USD scene assembly and dependency packaging. Run in the USD/Isaac environment."""
from pathlib import Path
import shutil
from .common import read_json, write_json, fresh_dir, resolve, sha256


def package(source, output):
    from pxr import Usd, UsdUtils, Sdf
    source = Path(source).resolve()
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(source))
    builtin = {'OmniGlass.mdl', 'OmniPBR.mdl', 'OmniSurface.mdl'}
    missing = [str(p) for p in unresolved if str(p) not in builtin]
    if missing:
        raise ValueError(f'Unresolved dependencies: {missing}')
    out = fresh_dir(output)
    dep = out/'dependencies'; dep.mkdir()
    mapping = {}
    for layer in layers:
        p = Path(layer.realPath).resolve()
        mapping[str(p)] = out/'scene.usda' if p == source else dep/(sha256(p)[:12]+'_'+p.name)
    for asset in assets:
        p = Path(str(asset)).resolve()
        if not p.is_file():
            raise ValueError(f'Nonlocal asset unsupported: {asset}')
        mapping[str(p)] = dep/(sha256(p)[:12]+'_'+p.name)
    for asset in assets:
        p = Path(str(asset)).resolve(); shutil.copy2(p, mapping[str(p)])
    import os
    for layer in layers:
        original = str(Path(layer.realPath).resolve()); destination = mapping[original]
        copy = Sdf.Layer.CreateAnonymous(); copy.TransferContent(layer)
        def rewrite(path):
            if not path or path in builtin:
                return path
            anchored = Sdf.ComputeAssetPathRelativeToLayer(layer, path)
            absolute = str(Path(anchored).resolve())
            if absolute not in mapping:
                raise ValueError(f'Cannot package {path} from {original}')
            return os.path.relpath(mapping[absolute], destination.parent)
        UsdUtils.ModifyAssetPaths(copy, rewrite)
        copy.Export(str(destination))
    stage = Usd.Stage.Open(str(out/'scene.usda'))
    if not stage:
        raise RuntimeError('Packaged USD cannot be reopened')
    _, _, remaining = UsdUtils.ComputeAllDependencies(str(out/'scene.usda'))
    if any(str(x) not in builtin for x in remaining):
        raise ValueError(f'Packaged unresolved dependencies: {remaining}')
    report = {'source':str(source),'entry':'scene.usda','files':[
        {'source':k,'path':str(v.relative_to(out)),'sha256':sha256(v)} for k,v in mapping.items()],
        'builtin_mdl':list(map(str, remaining)), 'requires':'Isaac Sim MDL search path for built-in shaders',
        'physics_validation':'not performed by packaging','visual_validation':'not performed by packaging'}
    write_json(out/'asset_manifest.json', report)
    return report


def build(spec_path, output):
    from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, UsdLux, Gf, Sdf
    spec_path = Path(spec_path).resolve(); spec = read_json(spec_path)
    if spec.get('units') != 'm' or spec.get('up_axis') != 'Z':
        raise ValueError('scene.json must explicitly use meters and Z up')
    objects = spec['objects']
    ids = [o['id'] for o in objects]
    if len(set(ids)) != len(ids):
        raise ValueError('Duplicate object IDs')
    # Validate before writing output; don't silently replace missing reconstruction with primitives.
    for o in objects:
        for key in ('size_m','position_m','quaternion_wxyz','material','texture','physics','evidence'):
            if key not in o: raise ValueError(f'{o["id"]} missing {key}')
        if o['kind'] not in ('box','sphere','cylinder','usd'):
            raise ValueError('Convert generated GLB/OBJ through Blender; supported kinds: box/sphere/cylinder/usd')
        if o['kind']=='usd' and not resolve(o['asset'], spec_path.parent).is_file():
            raise FileNotFoundError(o['asset'])
        if o['physics'].get('deformable'):
            raise ValueError('Deformable bodies require a dedicated validated asset; generic builder does not create them')
    out = fresh_dir(output); stage = Usd.Stage.CreateNew(str(out/'scene.usda'))
    UsdGeom.SetStageMetersPerUnit(stage,1); UsdGeom.SetStageUpAxis(stage,'Z')
    root=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(root.GetPrim())
    for o in objects:
        name=o['id']
        if not Sdf.Path.IsValidIdentifier(name):raise ValueError(f'Invalid USD object ID: {name}')
        path='/World/'+name;x=UsdGeom.Xform.Define(stage,path)
        if o['kind']=='usd':
            x.GetPrim().GetReferences().AddReference(str(resolve(o['asset'],spec_path.parent)),o.get('prim_path',''))
        q=o['quaternion_wxyz']; import math
        if abs(sum(float(v)**2 for v in q)-1)>1e-4: raise ValueError(f'{name}: quaternion must be normalized')
        x.ClearXformOpOrder();x.AddTranslateOp().Set(Gf.Vec3d(*o['position_m']))
        x.AddOrientOp().Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
        if o['kind']=='usd':
            x.AddScaleOp().Set(Gf.Vec3f(*o.get('scale_xyz',[1,1,1])))
        else:
            if any(float(v)<=0 for v in o['size_m']): raise ValueError('Dimensions must be positive')
            if o['kind']=='box':
                geom=UsdGeom.Cube.Define(stage,path+'/Visual');geom.CreateSizeAttr(1);scale=o['size_m']
            elif o['kind']=='sphere':
                geom=UsdGeom.Sphere.Define(stage,path+'/Visual');geom.CreateRadiusAttr(.5);scale=o['size_m']
            else:
                geom=UsdGeom.Cylinder.Define(stage,path+'/Visual');geom.CreateRadiusAttr(.5);geom.CreateHeightAttr(1);geom.CreateAxisAttr('Z');scale=o['size_m']
            geom.AddScaleOp().Set(Gf.Vec3f(*scale))
            m=o['material'];mat=UsdShade.Material.Define(stage,path+'/Material');sh=UsdShade.Shader.Define(stage,path+'/Material/Shader');sh.CreateIdAttr('UsdPreviewSurface')
            sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*m.get('base_color',[.5,.5,.5])))
            sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(m.get('roughness',.5));sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(m.get('metallic',0))
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)
            if o['texture'].get('path'):
                raise ValueError('Textured objects must use a USD with authored UV/materials; scalar primitive materials cannot consume a texture')
            physics=o['physics']
            if physics.get('collision',True):
                UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
                pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(physics.get('static_friction',.6));pm.CreateDynamicFrictionAttr(physics.get('dynamic_friction',.5));pm.CreateRestitutionAttr(physics.get('restitution',0.))
                UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat,materialPurpose='physics')
            if physics.get('rigid',False):
                if physics.get('mass_kg',0)<=0:raise ValueError('Rigid primitive requires positive mass_kg')
                UsdPhysics.RigidBodyAPI.Apply(x.GetPrim());UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(physics['mass_kg'])
        x.GetPrim().SetCustomDataByKey('realtosim:evidence',str(o['evidence']))
    light=UsdLux.DomeLight.Define(stage,'/World/Light');light.CreateIntensityAttr(spec.get('dome_intensity',800))
    UsdPhysics.Scene.Define(stage,'/World/PhysicsScene').CreateGravityMagnitudeAttr(9.81)
    stage.GetRootLayer().Save()
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']);results=[]
    for o in objects:
        bbox=cache.ComputeWorldBound(stage.GetPrimAtPath('/World/'+o['id'])).ComputeAlignedRange()
        results.append({**o,'world_aabb_size_m':list(bbox.GetSize()),'dimension_check':'world-axis AABB; rotated size is not local size'})
    write_json(out/'asset_manifest.json',{'objects':results,'source':str(spec_path),'units':'m','up_axis':'Z','physics_validation':'not run','metric_calibration':spec.get('scale_anchor','uncalibrated')})
    return str(out/'scene.usda')

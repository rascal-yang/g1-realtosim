"""Identical-state, head-camera-only rendering benchmark. Never advances physics."""
from pathlib import Path
import os,sys,json,time
BOOT=time.monotonic()
from .common import read_json, write_json, resolve, sha256
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--episode',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--profile',choices=['rt2','pt128'],default='rt2')
parser.add_argument('--start',type=int,default=0)
parser.add_argument('--count',type=int)
args=parser.parse_args()
manifest=args.episode.resolve(); config=read_json(manifest)
P={k:str(resolve(config[k],manifest.parent)) for k in ['source_raw','stage']}
route=args.profile;O=args.output.resolve()
if O.exists():raise FileExistsError(O)
import h5py
with h5py.File(P['source_raw']) as raw:
    N=len(raw['observation/timestamp']);dt=float(raw.attrs['dt'])
    stride=int(config.get('control_stride',2))
    if stride<1 or args.start<0 or args.start>=N or (args.count is not None and args.count<1):raise ValueError('Invalid frame selection')
    P['control_indices']=list(range(args.start,N,stride))[:args.count]
    P['warmup_control_indices']=[P['control_indices'][0]]*10
    calibration=json.loads(raw.attrs['camera_calibration'])['head']
    if (calibration['width'],calibration['height'])!=(640,480):raise ValueError('Current renderer contract is 640x480')
    P['realtime_updates_per_pose']=4
fps=1/(dt*stride)
O.mkdir(parents=True)
write_json(O/'protocol.json',{**P,'profile':route,'fps':fps,'episode':str(manifest),
    'source_raw_sha256':sha256(P['source_raw']),'stage_sha256':sha256(P['stage']),
    'camera_calibration':calibration,'physics_advanced':False})
if route=='rt2':
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RealTimePathTracing','extra_args':['--/persistent/rtx/modes/rt2/enabled=true','--/exts/omni.replicator.core/Orchestrator/enabled=false','--/app/renderer/waitIdle=true']})
else:
    from isaaclab.app import AppLauncher
    app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0').app
try:
    import omni.usd,omni.timeline,carb
    import omni.replicator.core as rep
    from pxr import UsdGeom,Gf
    from isaacsim.core.api import World
    import numpy as np,h5py,imageio.v2 as imageio
    from scipy.spatial.transform import Rotation
    omni.usd.get_context().open_stage(P['stage'])
    for _ in range(10):app.update()
    omni.timeline.get_timeline_interface().stop();stage=omni.usd.get_context().get_stage()
    for path in ['/Render','/Replicator']:
        if stage.GetPrimAtPath(path):stage.RemovePrim(path)
    viewport_report=[]
    try:
        from omni.kit.viewport.window import get_viewport_window_instances
        for window in get_viewport_window_instances():
            api=window.viewport_api;api.updates_enabled=False
            viewport_report.append({'updates_enabled':api.updates_enabled})
    except Exception as exc:viewport_report.append({'error':str(exc)})
    world=World(physics_dt=.005,rendering_dt=.04,stage_units_in_meters=1,physics_prim_path='/World/physicsScene')
    settings=carb.settings.get_settings();settings.set_float('/rtx/post/tonemap/exposureKey',.6)
    settings.set_bool('/rtx-transient/dlssg/enabled',False)
    cam=UsdGeom.Camera.Define(stage,config.get('camera_prim','/World/Capture/head'));cam.ClearXformOpOrder();camop=cam.AddTransformOp()
    cam.CreateHorizontalApertureAttr(36);cam.CreateVerticalApertureAttr(27)
    K=np.asarray(calibration['K']); assert np.allclose(K,[[320,0,320],[0,320,240],[0,0,1]])
    cam.CreateFocalLengthAttr(18);cam.CreateClippingRangeAttr((.015,40))
    product=rep.create.render_product(str(cam.GetPath()),(640,480));ann=rep.AnnotatorRegistry.get_annotator('rgb',device='cpu');ann.attach([product])
    settings.set_bool('/omni/replicator/asyncRendering',False)
    settings.set_bool('/app/asyncRendering',False)
    if route=='rt2': assert settings.get('/persistent/rtx/modes/rt2/enabled') is True
    # Preserve the mode selected by SimulationApp rather than calling the legacy realtime helper.
    P['realtime_updates_per_pose']=4
    rtpt_keys=['/rtx/rtpt/maxBounces','/rtx/rtpt/maxSpecularAndTransmissionBounces','/rtx/rtpt/translucency/virtualMotion/enabled','/rtx/post/aa/op','/rtx/post/dlss/execMode']
    (O/'rtpt_defaults.json').write_text(json.dumps({k:settings.get(k) for k in rtpt_keys},indent=2))
    if route=='rt2':
        settings.set_int('/rtx/post/aa/op',4)
        settings.set_int('/rtx/rtpt/maxBounces',4)
        settings.set_int('/rtx/rtpt/maxSpecularAndTransmissionBounces',6)
        settings.set_bool('/rtx/rtpt/translucency/virtualMotion/enabled',True)
    else:
        rep.settings.set_render_pathtraced(samples_per_pixel=128)
        settings.set_int('/rtx/pathtracing/clampSpp',128)
        settings.set_int('/rtx/pathtracing/spp',32)
        settings.set_bool('/rtx/pathtracing/optixDenoiser/enabled',True)
        settings.set_bool('/rtx/pathtracing/adaptiveSampling/enabled',False)
        settings.set_int('/rtx/pathtracing/maxBounces',4)
        settings.set_int('/rtx/pathtracing/maxSpecularAndTransmissionBounces',6)
    with h5py.File(P['source_raw']) as f,h5py.File(O/'frames.h5','w') as out:
        groups=[]
        for group,prefix,key in [('observation','Robot','body_names'),('brainco/right','BrainCo_right','brainco_right_body_names'),('brainco/left','BrainCo_left','brainco_left_body_names')]:
            if group not in f: continue
            pk='body_position_world' if group=='observation' else 'body_position'
            groups.append((prefix,json.loads(f.attrs[key]),f[group+'/'+pk][:],f[group+'/body_quaternion_wxyz'][:]))
        torso=groups[0][1].index(calibration['body_name']);bodyp=groups[0][2];bodyq=groups[0][3]
        entries=[]
        for prefix,names,ps,qs in groups:
            for j,name in enumerate(names):entries.append((config.get('env_prim','/World/envs/env_0')+'/'+prefix+'/'+name,ps[:,j],qs[:,j]))
        entries.append((config.get('object_prim','/World/envs/env_0/Object'),f['observation/object_position'][:],f['observation/object_quaternion_wxyz'][:]))
        for name in f['scene']:
            path=config['scene_prims'][name]
            entries.append((path,f['scene/'+name+'/position'][:],f['scene/'+name+'/quaternion_wxyz'][:]))
        # Cache authored world scales and immutable ancestors before any transforms change.
        original=UsdGeom.XformCache();paths={x[0] for x in entries};bindings=[];static_report={}
        for path,ps,qs in entries:
            prim=stage.GetPrimAtPath(path);assert prim,path
            scale=Gf.Matrix4d(1);scale.SetScale(Gf.Transform(original.GetLocalToWorldTransform(prim)).GetScale())
            parent=prim.GetParent();dynamic_parent=any(str(parent.GetPath())==p or str(parent.GetPath()).startswith(p+'/') for p in paths)
            parent_inv=None if dynamic_parent else original.GetLocalToWorldTransform(parent).GetInverse()
            xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();op=xf.AddTransformOp()
            static=bool(np.all(ps==ps[0]) and np.all(qs==qs[0]));static_report[path]=static
            bindings.append((path,prim,parent,op,scale,parent_inv,ps,qs,static))
        def pose(step,first=False):
            for path,prim,parent,op,scale,parent_inv,ps,qs,static in bindings:
                if static and not first:continue
                q=qs[step];m=Gf.Matrix4d(1);m.SetRotate(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))));m.SetTranslateOnly(Gf.Vec3d(*map(float,ps[step])))
                pi=UsdGeom.XformCache().GetLocalToWorldTransform(parent).GetInverse() if parent_inv is None else parent_inv
                op.Set(scale*m*pi)
            q=bodyq[step,torso];rot=Rotation.from_quat(q[[1,2,3,0]]);eye=bodyp[step,torso]+rot.apply(calibration['body_local_translation_m']);direction=rot.apply([np.cos(np.deg2rad(calibration['head_pitch_down_deg'])),0,-np.sin(np.deg2rad(calibration['head_pitch_down_deg']))]);up=rot.apply([0,0,1])
            m=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*(eye+direction)),Gf.Vec3d(*up)).GetInverse();camop.Set(m)
            return np.asarray(m).T@np.diag([1,-1,-1,1])
        def render(initial=False):
            updates=0;readbacks=0;stable=0;previous=None;render_s=0.;readback_s=0.
            if route=='rt2':
                for _ in range(20 if initial else P['realtime_updates_per_pose']):
                    t=time.monotonic();world.render();render_s+=time.monotonic()-t;updates+=1
                t=time.monotonic();rgb=np.asarray(ann.get_data())[:,:,:3].copy();readback_s+=time.monotonic()-t;readbacks=1
            else:
                while updates<40:
                    t=time.monotonic();world.render();render_s+=time.monotonic()-t;updates+=1
                    if updates<5:continue
                    t=time.monotonic();rgb=np.asarray(ann.get_data())[:,:,:3].copy();readback_s+=time.monotonic()-t;readbacks+=1
                    if previous is not None and np.array_equal(previous,rgb):stable+=1
                    else:stable=0
                    previous=rgb
                    if stable>=2:break
                if stable<2:raise RuntimeError('Accumulation did not settle')
            return rgb,dict(updates=updates,readbacks=readbacks,identical_final_updates=stable,render_s=render_s,readback_s=readback_s)
        startup=time.monotonic()-BOOT;warm=time.monotonic()
        for j,step in enumerate(P['warmup_control_indices']):pose(step,first=j==0);render(initial=j==0)
        warmup=time.monotonic()-warm
        # Validate every restored object's world position and orientation at the first measured state.
        pose(P['control_indices'][0]);cache=UsdGeom.XformCache();checks=[]
        for path,prim,parent,op,scale,pi,ps,qs,static in bindings:
            tr=Gf.Transform(cache.GetLocalToWorldTransform(prim));p=np.asarray(tr.GetTranslation());q=tr.GetRotation().GetQuat();q=np.r_[q.GetReal(),q.GetImaginary()];expected=qs[P['control_indices'][0]]
            checks.append({'path':path,'position_error_m':float(np.max(np.abs(p-ps[P['control_indices'][0]]))),'quaternion_error':float(min(np.max(np.abs(q-expected)),np.max(np.abs(q+expected))))})
        assert max(c['position_error_m'] for c in checks)<1e-6
        assert max(c['quaternion_error'] for c in checks)<1e-6
        (O/'state_restore_validation.json').write_text(json.dumps(checks,indent=2))
        skeys=rtpt_keys+['/persistent/rtx/modes/rt2/enabled','/omni/replicator/asyncRendering','/app/asyncRendering','/rtx/reflections/enabled','/rtx/translucency/enabled','/rtx/indirectDiffuse/enabled','/rtx/rendermode','/rtx/pathtracing/spp','/rtx/pathtracing/totalSpp','/rtx/pathtracing/clampSpp','/rtx/pathtracing/adaptiveSampling/enabled','/rtx/pathtracing/adaptiveSampling/targetError','/rtx/pathtracing/optixDenoiser/enabled','/rtx/pathtracing/maxBounces','/rtx/pathtracing/maxSpecularAndTransmissionBounces','/rtx/post/aa/op','/rtx/post/dlss/execMode','/rtx-transient/dlssg/enabled','/rtx/post/tonemap/exposureKey','/rtx/post/histogram/enabled']
        actual_settings={k:settings.get(k) for k in skeys}
        (O/'settings.json').write_text(json.dumps({'settings':actual_settings,'viewport':viewport_report,'render_product':str(product.path),'static_transform_skips':static_report},indent=2))
        steps=np.asarray(P['control_indices']);out['control_index']=steps;out['timestamp']=f['observation/timestamp'][:][steps]
        for name in ['observation','action','physics','next_observation','brainco','scene']:
            if name in f: out[name]=h5py.ExternalLink(P['source_raw'],'/'+name)
        out.attrs['source']=P['source_raw'];out.attrs['visual_realism_accepted']=False;out.attrs['physics_advanced']=False;out.attrs['route']=route;out.attrs['camera_calibration']=json.dumps(calibration);out.attrs['fps']=fps
        rgbout=out.create_dataset('rgb',(len(steps),480,640,3),dtype='u1',chunks=(1,480,640,3),compression='lzf');cvout=out.create_dataset('world_T_camera_cv',(len(steps),4,4),dtype='f8')
        writer=imageio.get_writer(O/'video.mp4',fps=fps,codec='libx264',quality=9,macro_block_size=1)
        start=time.monotonic();records=[]
        with (O/'timings.jsonl').open('w',buffering=1) as log:
            for i,step in enumerate(steps):
                frame_start=time.monotonic();t=time.monotonic();matrix=pose(step);pose_s=time.monotonic()-t
                rgb,stats=render();t=time.monotonic();rgbout[i]=rgb;cvout[i]=matrix;writer.append_data(rgb);write_s=time.monotonic()-t
                stats.update(frame=i,control_index=int(step),pose_s=pose_s,write_s=write_s,frame_s=time.monotonic()-frame_start);records.append(stats);log.write(json.dumps(stats)+'\n')
                if i%25==0 or i==len(steps)-1:
                    imageio.imwrite(O/f'frame_{i:05d}.png',rgb)
                    progress={'route':route,'frames':i+1,'elapsed_s':time.monotonic()-start};(O/'progress.json').write_text(json.dumps(progress));print(progress,flush=True)
        writer.close();out.flush();elapsed=time.monotonic()-start
    result={'route':route,'frames':len(steps),'fps':fps,'clip_duration_s':len(steps)/fps,'elapsed_s':elapsed,'startup_s':startup,'warmup_s':warmup,'frame_s_mean':float(np.mean([r['frame_s'] for r in records])),'frame_s_p95':float(np.percentile([r['frame_s'] for r in records],95)),'component_total_s':{k:sum(r[k] for r in records) for k in ['pose_s','render_s','readback_s','write_s']},'updates_minmax':[min(r['updates'] for r in records),max(r['updates'] for r in records)],'settings':actual_settings,'render_updates_per_pose':P['realtime_updates_per_pose'] if route=='rt2' else None,'physics_advanced':False,'visual_realism_accepted':False}
    write_json(O/'visual_realism_review.json',{'accepted':False,'temporal_review_status':'not_inspected','reviewed_frames':[],'physics_advanced':False,'note':'Inspect actual output using sim-visual-realism-review before accepting training RGB.'})
    (O/'complete.json').write_text(json.dumps(result,indent=2));print('COMPLETE',json.dumps(result),flush=True)
except Exception:
    import traceback;traceback.print_exc();raise
finally:
    app.close()

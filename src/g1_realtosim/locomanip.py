"""Fixed-base bimanual PhysX baseline. Run with the configured Isaac Python.

Object pose writes occur only at reset. Both hands follow one object-frame path.
This intentionally does not claim humanoid balance or a video reconstruction.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--attempts', type=int, default=1)
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--squeeze', type=float, default=0.0)
    parser.add_argument('--no-grip', action='store_true')
    parser.add_argument('--skill', type=Path, required=True)
    parser.add_argument('--source-seed', type=Path)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    skill=json.loads(args.skill.read_text())
    if skill.get('schema')!='bimanual_skill_v1' or skill['coupling']['independent_arm_warps_allowed']:
        raise ValueError('Expected a coupled bimanual_skill_v1 specification')
    if args.attempts<1:raise ValueError('attempts must be positive')
    if args.source_seed:
        seed=json.loads(args.source_seed.read_text())
        if not seed['physical_success']:raise ValueError('Cannot augment a failed seed')
        if seed.get('robot')!='two_fixed_Panda':raise ValueError('Seed embodiment mismatch')
        source=Path(seed['source_episode'])
        if hashlib.sha256(source.read_bytes()).hexdigest()!=seed['episode_sha256']:
            raise ValueError('Source episode is missing or has changed')
        if hashlib.sha256(Path(cfg['box_usd']).read_bytes()).hexdigest()!=seed['box_sha256']:
            raise ValueError('Only the same asset is supported')
        skill=seed['skill']
    cfg['skill']=skill
    cfg['source_seed']=str(args.source_seed.resolve()) if args.source_seed else None
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    boot = time.monotonic()
    from isaaclab.app import AppLauncher
    launcher = AppLauncher(headless=True, enable_cameras=False, device='cpu')
    app = launcher.app
    try:
        run(args, cfg, out, boot, app)
    finally:
        # Physics capture never starts a Replicator workflow. Waiting for its
        # default orchestrator while the timeline is playing can hang shutdown.
        app.close(wait_for_replicator=False, skip_cleanup=True)


def run(args, cfg, out, boot, app):
    import numpy as np
    import torch
    import h5py
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
    from isaaclab.sensors import ContactSensor, ContactSensorCfg
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
    from isaaclab.utils.math import subtract_frame_transforms
    from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
    from scipy.spatial.transform import Rotation
    from .common import write_json, sha256
    import shutil

    shutil.copy2(__file__, out/'locomanip.source.py')
    skill=cfg['skill']; control=skill['controller'];aug=skill['augmentation']
    write_json(out/'skill.json',skill)
    write_json(out/'config.json', dict(cfg, seed=args.seed, squeeze=args.squeeze,
                                     no_grip=args.no_grip, attempts=args.attempts))
    converter = UrdfConverter(UrdfConverterCfg(
        asset_path=cfg['panda_urdf'], usd_dir=cfg['robot_cache'],
        usd_file_name='panda.usd', fix_base=True, merge_fixed_joints=False,
        convert_mimic_joints_to_normal_joints=True, make_instanceable=False,
        self_collision=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=400., damping=40.))))
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
        dt=.005, device='cpu', gravity=(0,0,-9.81),
        physx=sim_utils.PhysxCfg(bounce_threshold_velocity=.2)))
    stage = sim_utils.get_current_stage()
    sim_utils.UsdFileCfg(usd_path=cfg['office_usd']).func('/World/Room', sim_utils.UsdFileCfg(usd_path=cfg['office_usd']))
    # Clear the tabletop for this designed task; preserve the desk and room assets.
    for path in ['/World/Room/Plush', '/World/Room/Scene/Basket',
                 '/World/Room/Cameras', '/World/Room/PhysicsScene',
                 '/World/Room/Scene/Inferred_enclosure']:
        if stage.GetPrimAtPath(path):
            stage.GetPrimAtPath(path).SetActive(False)
    sim_utils.DomeLightCfg(intensity=1800).func('/World/TaskLight',sim_utils.DomeLightCfg(intensity=1800))
    robots = []
    base_q = Rotation.from_euler('z',np.pi/2).as_quat()[[3,0,1,2]].tolist()
    for side, x in [('left',-.40),('right',.40)]:
        pedestal=sim_utils.CuboidCfg(size=(.22,.24,.748),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.12,.15,.18)))
        pedestal.func('/World/Mount_'+side,pedestal,translation=(x,-.62,.374))
        robot = Articulation(ArticulationCfg(
            prim_path='/World/'+side,
            spawn=sim_utils.UsdFileCfg(usd_path=converter.usd_path,
                activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,solver_position_iteration_count=32,solver_velocity_iteration_count=8)),
            init_state=ArticulationCfg.InitialStateCfg(pos=(x,-.62,.75),rot=tuple(base_q),
                joint_pos={'panda_joint1':0.,'panda_joint2':-.55,'panda_joint3':0.,
                           'panda_joint4':-2.,'panda_joint5':0.,'panda_joint6':1.5,
                           'panda_joint7':.7854,'panda_finger_joint.*':.0}),
            actuators={'arm':ImplicitActuatorCfg(joint_names_expr=['panda_joint[1-7]'],
                stiffness=800.,damping=60.,effort_limit_sim={'panda_joint[1-4]':87.,'panda_joint[5-7]':12.},velocity_limit_sim=2.),
                'hand':ImplicitActuatorCfg(joint_names_expr=['panda_finger_joint.*'],
                stiffness=300.,damping=15.,effort_limit_sim=20.,velocity_limit_sim=.2)}))
        robots.append(robot)
    box=RigidObject(RigidObjectCfg(prim_path='/World/Box',
        spawn=sim_utils.UsdFileCfg(usd_path=cfg['box_usd'],activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False,
                solver_position_iteration_count=32,solver_velocity_iteration_count=8,
                max_depenetration_velocity=.5),mass_props=sim_utils.MassPropertiesCfg(mass=.3)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0,0,1.0))))
    # Explicit estimated friction, identical for every trial, no attachment joints.
    mat_path='/World/ContactMaterial'
    sim_utils.RigidBodyMaterialCfg(static_friction=.8,dynamic_friction=.7,restitution=0.).func(
        mat_path,sim_utils.RigidBodyMaterialCfg(static_friction=.8,dynamic_friction=.7,restitution=0.))
    sim_utils.bind_physics_material('/World/Box',mat_path)
    for side in ['left','right']:
        # Imported geometry is instanced; uninstance to bind the explicit material.
        from pxr import Usd
        for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/'+side)):
            if prim.IsInstance():prim.SetInstanceable(False)
        sim_utils.bind_physics_material('/World/'+side,mat_path)
    sensors=[]
    for side in ['left','right']:
        for body in ['panda_hand','panda_leftfinger','panda_rightfinger']:
            sensors.append(ContactSensor(ContactSensorCfg(
                prim_path=f'/World/{side}/{body}', update_period=0.,
                filter_prim_paths_expr=['/World/Box'],history_length=1)))
    arm_sensors=[]
    arm_sensor_names=[]
    for side in ['left','right']:
        for body in [f'panda_link{i}' for i in range(8)]:
            arm_sensors.append(ContactSensor(ContactSensorCfg(
                prim_path=f'/World/{side}/{body}', update_period=0.,history_length=1)))
            arm_sensor_names.append(side+'/'+body)
    sim.reset()
    for r in robots:r.update(.005)
    box.update(.005)
    for sensor in sensors:sensor.update(.005)
    for sensor in arm_sensors:sensor.update(.005)
    print('ROBOT',robots[0].joint_names,robots[0].body_names,flush=True)
    drive_limits=[r.root_physx_view.get_dof_max_forces().cpu().numpy().tolist() for r in robots]
    write_json(out/'drive_limits_readback.json',{'joint_names':robots[0].joint_names,'max_force_by_robot':drive_limits})
    arms=[r.find_joints('panda_joint[1-7]')[0] for r in robots]
    handids=[r.body_names.index('panda_hand') for r in robots]
    ik=[DifferentialIKController(DifferentialIKControllerCfg(command_type='pose',
        ik_method='dls',ik_params={'lambda_val':.03}),num_envs=1,device='cpu') for _ in robots]
    bounds=UsdGeom.BBoxCache(0,['default','render']).ComputeLocalBound(stage.GetPrimAtPath('/World/Box')).ComputeAlignedRange()
    lo=np.array(bounds.GetMin());hi=np.array(bounds.GetMax())
    half=(hi-lo)/2
    collision_points=np.array(UsdGeom.Mesh(stage.GetPrimAtPath('/World/Box/Collision')).GetPointsAttr().Get())
    print('BOX_BOUNDS',lo,hi,flush=True)
    write_json(out/'provenance.json',{
        'simulator':'Isaac Sim 5.1 / PhysX CPU','robot':'two fixed-base Franka Panda arms',
        'humanoid_balance_validated':False,'source_kind':'user_authorized_designed_task',
        'video_source':None,'vlm_calls':0,'manual_annotation_seconds':0,
        'human_engineering_time_seconds':None,'physics_dt':.005,'control_dt':.02,
        'coordinate_frame':'right-handed world Z-up, meters radians seconds; quaternions wxyz',
        'assets':{k:{'path':cfg[k],'sha256':sha256(cfg[k])} for k in ['office_usd','box_usd','panda_urdf']},
        'box_half_extents_m':half.tolist(),'box_mass_kg':.3,
        'mass_friction_status':'engineering_assumptions_not_measured',
        'object_pose_writes':'initialization and episode reset only',
        'controller':'coupled object-frame waypoints, per-arm DLS IK, implicit joint PD',
        'self_collisions':'disabled in imported Panda articulations; external contacts monitored',
        'training_eligible':False})
    rng=np.random.default_rng(args.seed)
    all_results=[]
    for attempt in range(args.attempts):
        start=time.monotonic(); ep=out/f'episode_{attempt:03d}';ep.mkdir()
        nominal=attempt==0 and args.source_seed is None
        offset=np.zeros(2) if nominal else rng.uniform(-aug['initial_xy_half_range_m'],aug['initial_xy_half_range_m'],2)
        target=np.array([.0,.13])+ (np.zeros(2) if nominal else rng.uniform(-aug['goal_xy_half_range_m'],aug['goal_xy_half_range_m'],2))
        yaw=0. if nominal else float(rng.uniform(-aug['initial_yaw_half_range_rad'],aug['initial_yaw_half_range_rad']))
        target_yaw=yaw if nominal else float(rng.uniform(-aug['goal_yaw_half_range_rad'],aug['goal_yaw_half_range_rad']))
        table_z=.75
        initial=np.array([offset[0],offset[1]-.07,table_z+half[2]+.003])
        reset_position=initial.copy()
        settled_quaternion=Rotation.from_euler('z',yaw).as_quat()[[3,0,1,2]]
        destination=np.r_[target[0],target[1]-.07,table_z+half[2]]
        state=box.data.default_root_state.clone();state[0,:3]=torch.tensor(initial)
        state[0,3:7]=torch.tensor(Rotation.from_euler('z',yaw).as_quat()[[3,0,1,2]])
        state[0,7:]=0;box.write_root_state_to_sim(state);box.reset()
        for r in robots:
            r.write_joint_state_to_sim(r.data.default_joint_pos.clone(),torch.zeros_like(r.data.joint_vel));r.reset()
        for s in sensors:s.reset()
        for s in arm_sensors:s.reset()
        for r in robots:r.update(.005)
        home=[r.data.body_pos_w[0,h].numpy().copy() for r,h in zip(robots,handids)]
        home_q=[r.data.body_quat_w[0,h].numpy().copy() for r,h in zip(robots,handids)]
        stage.GetRootLayer().Export(str(ep/'initial_scene.usda'))
        # Hand z axes point down. Paired inner finger faces squeeze opposite box faces.
        # A shared center and yaw preserve the two-hand transform during transport.
        phase_defs=skill['phase_durations_s']
        records=[]; substeps=[]; current=0; reason=None; peak_lift=0.; grip_hold=0.
        clear_hold=0.;clear_run=0.;max_clear_run=0.;collision_event=None;max_unintended=0.
        desired=[r.data.joint_pos.clone() for r in robots]
        object0=None; phase_success={}; last_pose=None
        destination_hand=destination.copy()
        for phase,duration in phase_defs:
            n=int(duration/.02)
            # At contact boundaries hold the completed reference for up to 1 s
            # before evaluating measured speed/contact, rather than time alone.
            for step in range(n+(50 if phase in ['close','lower'] else 0)):
                u=min((step+1)/n,1.); blend=u*u*(3-2*u)
                center=initial.copy(); ang=yaw
                gap=control['open_gap_m']
                if phase in ['close','lift','transport','lower','release','retreat','verify']:
                    gap=control['open_gap_m']*(1-blend) if phase=='close' else 0.
                if phase=='lift':center[2]+=control['lift_m']*blend
                if phase=='transport':
                    center=(1-blend)*initial+blend*destination_hand;center[2]+=control['lift_m'];ang=yaw+(target_yaw-yaw)*blend
                if phase in ['lower','release','retreat','verify']:
                    center=destination_hand.copy();ang=target_yaw
                    if phase=='lower':center[2]+=control['lift_m']*(1-blend)
                    if phase=='release':gap=control['open_gap_m']*blend
                    if phase in ['retreat','verify']:gap=control['open_gap_m']
                if args.no_grip:gap=max(gap,.12)
                rz=Rotation.from_euler('z',ang)
                q=(rz*Rotation.from_euler('z',np.pi/2)*Rotation.from_euler('x',np.pi)).as_quat()[[3,0,1,2]]
                wrists=[]
                for i,(r,ids,h,controller) in enumerate(zip(robots,arms,handids,ik)):
                    sign=-1 if i==0 else 1
                    p=center+rz.apply([sign*(half[0]+control['hand_offset_outside_bbox_m']+gap-args.squeeze),0,control['hand_z_offset_m']])
                    if phase in ['retreat','verify']:p[2]+=.13*(blend if phase=='retreat' else 1)
                    qq=q
                    if phase=='settle':p=home[i];qq=home_q[i]
                    if phase=='approach':
                        p=home[i]+blend*(p-home[i]);qq=Rotation.from_quat(home_q[i][[1,2,3,0]])
                        from scipy.spatial.transform import Slerp
                        qq=Slerp([0,1],Rotation.from_quat(np.array([home_q[i],q])[:,[1,2,3,0]]))([blend]).as_quat()[0,[3,0,1,2]]
                    wrists.append(np.r_[p,qq])
                    # Jacobians are world-frame for fixed bases; rotate to base frame.
                    jp=r.root_physx_view.get_jacobians()[:,h-1,:,ids]
                    rootp=r.data.root_pos_w;rootq=r.data.root_quat_w
                    localp,localq=subtract_frame_transforms(rootp,rootq,torch.tensor(p[None],dtype=torch.float32),torch.tensor(np.asarray(qq)[None],dtype=torch.float32))
                    cp,cq=subtract_frame_transforms(rootp,rootq,r.data.body_pos_w[:,h],r.data.body_quat_w[:,h])
                    from isaaclab.utils.math import matrix_from_quat,quat_inv
                    rot=matrix_from_quat(quat_inv(rootq));jp[:,:3]=rot@jp[:,:3];jp[:,3:]=rot@jp[:,3:]
                    controller.set_command(torch.cat([localp,localq],dim=1))
                    solution=controller.compute(cp,cq,jp,r.data.joint_pos[:,ids])
                    limited=desired[i][:,ids]+torch.clamp(solution-desired[i][:,ids],-.02,.02)
                    limits=r.data.soft_joint_pos_limits[:,ids]
                    desired[i][:,ids]=torch.clamp(limited,limits[...,0],limits[...,1])
                    desired[i][:,r.find_joints('panda_finger_joint.*')[0]]=0.
                pre=read_state(robots,box)
                contacts=[]
                for sub in range(4):
                    for r,d in zip(robots,desired):r.set_joint_position_target(d);r.write_data_to_sim()
                    sim.step(render=False)
                    for r in robots:r.update(.005)
                    box.update(.005)
                    for s in sensors:s.update(.005)
                    for s in arm_sensors:s.update(.005)
                    force=np.array([s.data.force_matrix_w[0].numpy().reshape(-1,3).sum(0) for s in sensors])
                    net=np.array([s.data.net_forces_w[0].numpy().reshape(-1,3).sum(0) for s in sensors])
                    arm_forces=np.array([s.data.net_forces_w[0].numpy().reshape(-1,3).sum(0) for s in arm_sensors])
                    unintended=np.r_[np.linalg.norm(net-force,axis=1),np.linalg.norm(arm_forces,axis=1)]
                    max_unintended=max(max_unintended,float(unintended.max()))
                    if unintended.max()>5. and collision_event is None:
                        names=[f'{side}/{body}' for side in ['left','right'] for body in ['panda_hand','panda_leftfinger','panda_rightfinger']]+arm_sensor_names
                        collision_event={'time_s':current*.02+(sub+1)*.005,'link':names[int(unintended.argmax())],
                            'force_N':float(unintended.max()),'kind':'unexpected_external_contact'}
                    contacts.append(force)
                    substeps.append((current,sub,force,net,box.data.root_state_w[0].numpy().copy(),arm_forces,
                        np.concatenate([r.data.applied_torque[0].numpy().copy() for r in robots])))
                post=read_state(robots,box)
                actual=post['object_position']
                if phase=='settle':object0=actual.copy()
                clearance=actual[2]-(object0[2] if object0 is not None else initial[2])
                peak_lift=max(peak_lift,float(clearance))
                cf=np.linalg.norm(np.sum(contacts[-1][:3],axis=0)),np.linalg.norm(np.sum(contacts[-1][3:],axis=0))
                oq=post['object_quaternion_wxyz'];rot=Rotation.from_quat(oq[[1,2,3,0]])
                bottom=float((collision_points@rot.as_matrix().T)[:,2].min()+actual[2]-table_z)
                verified_clear=bottom>.03 and min(cf)>.1
                clear_run=clear_run+.02 if verified_clear else 0.
                max_clear_run=max(max_clear_run,clear_run)
                if verified_clear:clear_hold+=.02
                if clearance>.065 and min(cf)>.1:grip_hold+=.02
                records.append(dict(pre=pre,post=post,command=np.concatenate([d[0].numpy().copy() for d in desired]),
                    ee_target=np.array(wrists),phase=phase,t=current*.02,contact=contacts[-1],bottom_clearance=bottom))
                current+=1
                if actual[2]<table_z-.1 or not np.isfinite(actual).all():reason='object_fell';break
                if collision_event:reason='unexpected_external_contact';break
            print('PHASE',attempt,phase,'box',actual.tolist(),'forces',cf,'lift',peak_lift,flush=True)
            phase_success[phase]={'object_position':actual.tolist(),'contact_force_norms_N':list(map(float,cf))}
            if phase=='settle':
                destination[2]=actual[2]
                settled_quaternion=post['object_quaternion_wxyz'].copy()
                initial=actual.copy()
            if phase=='close':
                # Reanchor the shared pair to the achieved grasp, not the nominal box reset.
                grasp_offset=actual-initial
                destination_hand=destination-Rotation.from_euler('z',target_yaw-yaw).apply(grasp_offset)
                if min(cf)<.1 and not args.no_grip:reason='bilateral_contact_not_reached'
                if np.linalg.norm(post['object_velocity'][:3])>.08:reason='grasp_not_settled'
            if phase=='lift' and max_clear_run<.5:reason='no_verified_bimanual_lift'
            if phase=='lower' and (abs(actual[2]-destination[2])>.025 or np.linalg.norm(post['object_velocity'][:3])>.05):
                reason='not_ready_to_release'
            if reason:break
        final=box.data.root_state_w[0].numpy()
        error=float(np.linalg.norm(final[:2]-destination[:2]));speed=float(np.linalg.norm(final[7:10]))
        expected_rot=Rotation.from_euler('z',target_yaw-yaw)*Rotation.from_quat(settled_quaternion[[1,2,3,0]])
        rotation_error=float((expected_rot.inv()*Rotation.from_quat(final[[4,5,6,3]])).magnitude())
        stable_tail=[r for r in records if r['phase']=='verify'][-50:]
        released_stable=len(stable_tail)==50 and all(np.linalg.norm(r['post']['object_velocity'][:3])<.025 and np.max(np.linalg.norm(r['contact'],axis=-1))<.1 for r in stable_tail)
        success=reason is None and peak_lift>.065 and max_clear_run>.5 and error<.035 and abs(final[2]-destination[2])<.02 and released_stable and rotation_error<.2
        if not success and reason is None:
            reason='no_verified_bimanual_lift' if grip_hold<=.5 else 'placement_or_release_failed'
        result={'attempt':attempt,'success':bool(success),'failure_reason':reason,'seed':args.seed,
            'initial_position':initial.tolist(),'goal_position':destination.tolist(),'initial_yaw':yaw,'goal_yaw':target_yaw,
            'reset_position':reset_position.tolist(),'settled_position':None if object0 is None else object0.tolist(),
            'peak_lift_m':peak_lift,'bilateral_lift_contact_s':grip_hold,'final_xy_error_m':error,
            'whole_object_clear_bilateral_s':clear_hold,'longest_clear_bilateral_s':max_clear_run,
            'max_unintended_contact_N':max_unintended,'collision_event':collision_event,
            'released_stable_final_second':released_stable,'final_rotation_error_rad':rotation_error,
            'source_seed':cfg['source_seed'],
            'final_speed_m_s':speed,'rollout_wall_s':time.monotonic()-start,'simulation_s':current*.02,
            'phases':phase_success,'training_eligible':False,'humanoid_balance_validated':False}
        save_episode(ep,records,substeps,robots,result,cfg)
        write_json(ep/'result.json',result);all_results.append(result)
        if success:
            write_json(ep/'seed.json',{'schema':'verified_parametric_seed_v1','physical_success':True,
                'robot':'two_fixed_Panda','box_sha256':sha256(cfg['box_usd']),
                'episode_sha256':sha256(ep/'episode.h5'),'skill':skill,
                'source_episode':str(ep/'episode.h5'),
                'adaptation':'re-execute verified parametric skill with common object-frame transforms; reanchor grasp from measured state'})
        with (out/'attempts.jsonl').open('a') as f:f.write(json.dumps(result)+'\n')
        write_json(ep/'reflection.json',{'observation':result,'hypothesis':'Opposed finger contacts lift through friction; both wrists share an object-frame path.',
            'status':'observed','next_step':'repeat pose perturbations' if success else 'inspect actual contact and tracking before changing geometry or gains'})
        print('RESULT',json.dumps(result),flush=True)
    successes=sum(r['success'] for r in all_results)
    write_json(out/'summary.json',{'attempts':len(all_results),'successes':successes,
        'total_wall_s':time.monotonic()-boot,'rollout_wall_s':sum(r['rollout_wall_s'] for r in all_results),
        'wall_s_per_success_including_failures':(time.monotonic()-boot)/successes if successes else None,
        'video_to_seed_success_rate':None,'video_inputs':0,'full_locomotion_attempts':0,
        'full_locomotion_success_rate':None,'results':all_results})


def read_state(robots,box):
    import numpy as np
    return {'joint_position':np.concatenate([r.data.joint_pos[0].numpy().copy() for r in robots]),
        'joint_velocity':np.concatenate([r.data.joint_vel[0].numpy().copy() for r in robots]),
        'body_position_world':np.concatenate([r.data.body_pos_w[0].numpy().copy() for r in robots]),
        'body_quaternion_wxyz':np.concatenate([r.data.body_quat_w[0].numpy().copy() for r in robots]),
        'object_position':box.data.root_pos_w[0].numpy().copy(),
        'object_quaternion_wxyz':box.data.root_quat_w[0].numpy().copy(),
        'object_velocity':box.data.root_vel_w[0].numpy().copy()}


def save_episode(ep,records,substeps,robots,result,cfg):
    import numpy as np
    import h5py
    from .common import write_json
    with h5py.File(ep/'episode.h5','w') as f:
        f.attrs['schema']='bimanual_physx_v1';f.attrs['dt']=.02;f.attrs['physics_dt']=.005
        f.attrs['success']=result['success'];f.attrs['training_eligible']=False
        f.attrs['joint_names']=json.dumps([side+'/'+n for side,r in zip(['left','right'],robots) for n in r.joint_names])
        f.attrs['body_names']=json.dumps([side+'/'+n for side,r in zip(['left','right'],robots) for n in r.body_names])
        f.attrs['camera_calibration']=json.dumps({'head':{'width':640,'height':480,'K':[[320,0,320],[0,320,240],[0,0,1]]}})
        for group,key in [('observation','pre'),('next_observation','post')]:
            for name in records[0][key]:f.create_dataset(group+'/'+name,data=np.array([r[key][name] for r in records]),compression='gzip')
        f['observation/timestamp']=[r['t'] for r in records]
        f['action/joint_position_target']=np.array([r['command'] for r in records])
        f['action/ee_pose_target_world']=np.array([r['ee_target'] for r in records])
        f['phase']=np.array([r['phase'].encode() for r in records])
        f['physics/control_index']=[s[0] for s in substeps]
        f['physics/substep']=[s[1] for s in substeps]
        f['physics/hand_object_force_world']=np.array([s[2] for s in substeps])
        f['physics/hand_net_force_world']=np.array([s[3] for s in substeps])
        f['physics/object_state']=np.array([s[4] for s in substeps])
        f['physics/arm_net_force_world']=np.array([s[5] for s in substeps])
        f['physics/actuator_estimated_torque']=np.array([s[6] for s in substeps])
        f['physics/joint_position_target']=np.array([records[s[0]]['command'] for s in substeps])
        f['metrics/object_bottom_clearance']=np.array([r['bottom_clearance'] for r in records])
        f.attrs['actuator_gains']=json.dumps({'arm':{'kp':800,'kd':60,'max_effort':[87,87,87,87,12,12,12]},'fingers':{'kp':300,'kd':15,'max_effort':20}})
        f.attrs['joint_units']=json.dumps((['rad']*7+['m']*2)*2)
        f.attrs['quaternion_order']='wxyz';f.attrs['units']='m,s,rad,N,Nm'
    write_json(ep/'visual_realism_review.json',{'accepted':False,'temporal_review_status':'not_rendered','training_eligible':False})


if __name__=='__main__':
    main()

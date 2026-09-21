"""Explicit observation/action alignment for G1 + separate Revo 2 articulations."""
from pathlib import Path
import json
import shutil
import subprocess
import h5py
import numpy as np
from scipy.spatial.transform import Rotation
from .common import fresh_dir, write_json, read_json, sha256

BODY_FIELDS = ['joint_position_target','joint_velocity_target','joint_effort_command','joint_stiffness','joint_damping']


def verify(raw, frames):
    with h5py.File(raw) as f, h5py.File(frames) as g:
        idx=g['control_index'][:];t=f['observation/timestamp'][:];n=len(t)
        declared=g.attrs.get('source')
        if declared and Path(declared).resolve()!=Path(raw).resolve():
            if not Path(declared).is_file() or sha256(declared)!=sha256(raw):
                raise ValueError('RGB was rendered from a different raw trajectory')
        if idx.ndim!=1 or len(idx)<1 or idx.dtype.kind not in 'iu' or np.any(idx<0) or np.any(idx>=n) or np.any(np.diff(idx)<=0):
            raise ValueError('Invalid/nonmonotonic control indices')
        if len(g['rgb'])!=len(idx) or g['rgb'].shape[1:]!=(480,640,3):raise ValueError('Unexpected RGB shape')
        if not np.allclose(g['timestamp'][:],t[idx],rtol=0,atol=1e-9):raise ValueError('RGB timestamps do not match pre-action state')
        if not np.allclose(np.diff(t),float(f.attrs['dt']),rtol=0,atol=1e-8):raise ValueError('Nonuniform control time')
        if len(idx)>1 and not np.all(np.diff(idx)==np.diff(idx)[0]):raise ValueError('Nonuniform RGB time')
        matrix=g['world_T_camera_cv'][:]
        if matrix.shape!=(len(idx),4,4) or not np.isfinite(matrix).all():raise ValueError('Invalid camera transforms')
        if not np.allclose(f['next_observation/joint_position'][:-1],f['observation/joint_position'][1:],rtol=0,atol=1e-6):raise ValueError('Broken body transition chain')
        for key in BODY_FIELDS:
            if f['action/'+key].shape!=f['observation/joint_position'].shape:raise ValueError('Action shape mismatch')
            if not np.isfinite(f['action/'+key][:]).all():raise ValueError('Nonfinite action values')
        stride=int(np.diff(idx)[0]) if len(idx)>1 else 2
        return {'frames':len(idx),'control_steps':n,'stride':stride,'fps':1/(float(f.attrs['dt'])*stride),
                'index_timestamp_checks':True,'camera_matrices_finite':True,'body_transition_chain':True,
                'pixel_latency_validation':'not independently proven by metadata checks'}


def export(raw_path, frames_path, output, hand_contract=None):
    raw_path=Path(raw_path).resolve();frames_path=Path(frames_path).resolve()
    validation=verify(raw_path,frames_path)
    out=fresh_dir(output)
    with h5py.File(raw_path) as f,h5py.File(frames_path) as g,h5py.File(out/'episode.h5','w') as dst:
        n=len(f['observation/timestamp']);idx=g['control_index'][:];stride=validation['stride']
        body_names=json.loads(f.attrs['joint_names']);dt=float(f.attrs['dt']);pd=float(f.attrs['physics_dt'])
        decimation=round(dt/pd)
        if not np.isclose(decimation*pd,dt):raise ValueError('Nonintegral decimation')
        state_names=[f'g1/{name}/{field}' for field in ['position_rad','velocity_rad_s'] for name in body_names]
        states=[f['observation/joint_position'][:],f['observation/joint_velocity'][:]]
        hand_values=[];hand_names=[];contract=read_json(hand_contract) if hand_contract else {}
        for side in ['left','right']:
            if 'brainco/'+side not in f:continue
            names=json.loads(f.attrs['brainco_'+side+'_joint_names'])
            states += [f['brainco/'+side+'/joint_position'][:],f['brainco/'+side+'/joint_velocity'][:]]
            state_names += [f'revo2/{side}/{name}/{field}' for field in ['position_rad','velocity_rad_s'] for name in names]
            # Keep all recorded hand slots. Passive/mimic slots are declared in the contract and must not be driven.
            hand_values.append(f['brainco/'+side+'/position_target'][:])
            hand_names += [f'revo2/{side}/{name}/position_target' for name in names]
        rootq=f['observation/root_quaternion_wxyz'][:]
        gyro=Rotation.from_quat(rootq[:,[1,2,3,0]]).inv().apply(f['observation/root_angular_velocity_world'][:])
        states += [rootq,gyro]
        state_names += ['base/quaternion_'+c for c in 'wxyz']+['base/gyro_'+c+'_rad_s' for c in 'xyz']
        state=np.concatenate(states,axis=1).astype('float32')
        # Preserve EVERY body physics-substep command, including feedforward, stiffness and damping.
        physics_idx=f['physics/control_index'][:];sub=f['physics/substep_index'][:]
        if not np.array_equal(physics_idx,np.repeat(np.arange(n),decimation)) or not np.array_equal(sub,np.tile(np.arange(decimation),n)):
            raise ValueError('Physics substeps missing/reordered')
        physics=np.stack([f['physics/'+key][:] for key in BODY_FIELDS],axis=1).reshape(n,decimation,-1)
        commands=[physics.reshape(n,-1),*hand_values]
        per_control=np.concatenate(commands,axis=1).astype('float32')
        if not np.isfinite(state).all() or not np.isfinite(per_control).all():raise ValueError('Nonfinite state or physics commands')
        control_names=[f'substep_{substep}/{key}/{name}' for substep in range(decimation) for key in BODY_FIELDS for name in body_names]+hand_names
        action_names=[f'control_{offset}/{name}' for offset in range(stride) for name in control_names]
        action=np.zeros((len(idx),stride,per_control.shape[1]),dtype='float32');mask=np.zeros((len(idx),stride),dtype=bool)
        for j,i in enumerate(idx):
            count=min(stride,n-int(i));action[j,:count]=per_control[i:i+count];mask[j,:count]=True
        dst['observation/state']=state[idx];dst['action/commands']=action;dst['action/valid']=mask
        dst['control_index']=idx;dst['timestamp']=g['timestamp'][:];dst['camera/world_T_camera_cv']=g['world_T_camera_cv'][:]
        rgb=dst.create_dataset('observation/rgb',g['rgb'].shape,dtype='u1',chunks=(1,480,640,3),compression='lzf')
        for j in range(len(idx)):rgb[j]=g['rgb'][j]
        dst.attrs['schema']='g1_realtosim_aligned_v1';dst.attrs['task']=str(f.attrs['task'])
        dst.attrs['state_names']=json.dumps(state_names);dst.attrs['action_names']=json.dumps(action_names)
        dst.attrs['fps']=validation['fps'];dst.attrs['control_dt']=dt;dst.attrs['physics_dt']=pd
        dst.attrs['visual_realism_accepted']=bool(g.attrs.get('visual_realism_accepted',False))
        dst.attrs['episode_success']=bool(f.attrs.get('episode_success',False))
        dst.attrs['source_positive_behavior_cloning_eligible']=bool(f.attrs.get('positive_behavior_cloning_eligible',False))
        dst.attrs['source_raw_sha256']=sha256(raw_path);dst.attrs['source_rgb_sha256']=sha256(frames_path)
        dst.attrs['camera_calibration']=g.attrs.get('camera_calibration',f.attrs.get('camera_calibration','{}'))
        # Numeric training artifact is self-contained; original high-rate telemetry remains separately archived.
        provenance={'source_raw':str(raw_path),'source_rgb':str(frames_path),'source_raw_sha256':dst.attrs['source_raw_sha256'],
                    'source_rgb_sha256':dst.attrs['source_rgb_sha256'],'frames':len(idx),'fps':validation['fps'],
                    'state_dimension':state.shape[1],'action_shape':list(action.shape[1:]),'physics_substeps_per_control':decimation,
                    'state_names':state_names,'action_names':action_names,'hand_contract':contract,
                    'hand_contract_verified':False if hand_values else True,
                    'action_semantics':'For each 20ms control slot, body fields run at recorded physics_dt; hand targets are held for that control slot. Passive/mimic slots retained for audit, never independent motors.',
                    'last_partial_chunk':'Zero padded only where action/valid=false; LeRobot export drops incomplete chunks.',
                    'visual_realism_accepted':bool(dst.attrs['visual_realism_accepted']),
                    'source_positive_behavior_cloning_eligible':bool(dst.attrs['source_positive_behavior_cloning_eligible']),
                    'training_eligible':False,
                    'remaining_review':'Visual temporal acceptance and controller/hand contract review required; structural export does not grant training eligibility.',
                    'episode_success':bool(dst.attrs['episode_success'])}
        write_json(out/'provenance.json',provenance)
        write_json(out/'validation.json',validation)
    return {k:provenance[k] for k in ['frames','state_dimension','action_shape','training_eligible']}


def lerobot_export(aligned, output, allow_unreviewed=False):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    aligned=Path(aligned).resolve();output=Path(output).resolve()
    if output.exists():raise FileExistsError(output)
    provenance=read_json(aligned.parent/'provenance.json')
    if not provenance.get('training_eligible') and not allow_unreviewed:
        raise ValueError('Dataset not accepted for training; use --allow-unreviewed for a clearly labelled development export')
    with h5py.File(aligned) as f:
        mask=f['action/valid'][:];valid=np.flatnonzero(mask.all(axis=1))
        if len(valid)==0:raise ValueError('No complete action chunks')
        if not np.array_equal(valid,np.arange(len(valid))):raise ValueError('Interior partial chunks cannot be dropped without breaking FPS')
        ns=f['observation/state'].shape[1];na=int(np.prod(f['action/commands'].shape[1:]))
        features={'observation.state':{'dtype':'float32','shape':(ns,),'names':json.loads(f.attrs['state_names'])},
                  'action':{'dtype':'float32','shape':(na,),'names':json.loads(f.attrs['action_names'])},
                  'observation.images.head':{'dtype':'video','shape':(480,640,3),'names':['height','width','channels']},
                  'next.done':{'dtype':'bool','shape':(1,),'names':['done']}}
        ds=LeRobotDataset.create(repo_id='local/g1_realtosim',root=output,fps=int(round(f.attrs['fps'])),features=features,
                                robot_type='g1_revo2_simulation',use_videos=True,image_writer_threads=2,video_backend='pyav',vcodec='h264')
        for j in valid:
            ds.add_frame({'observation.state':f['observation/state'][j],'action':f['action/commands'][j].reshape(-1),
                          'observation.images.head':f['observation/rgb'][j],'next.done':np.array([j==valid[-1]]),'task':str(f.attrs['task'])})
        ds.save_episode();ds.finalize()
        write_json(output/'realtosim_provenance.json',{**provenance,'exported_frames':len(valid),
                   'dropped_incomplete_frames':len(mask)-len(valid),'development_export':not provenance.get('training_eligible',False),
                   'data_contract':'Structured commands were flattened, preserving physics substeps. Do not execute as a simple vector of joint angles.'})
    return str(output)

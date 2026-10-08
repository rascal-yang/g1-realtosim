"""Export self-contained Panda RGB/state/actual-action samples, without G1 schema assumptions."""
import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from .common import sha256


def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--frames',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    with h5py.File(args.episode) as raw,h5py.File(args.frames) as frames:
        if raw.attrs['schema']!='bimanual_physx_v1':raise ValueError('Wrong embodiment/schema')
        if sha256(args.episode)!=json.loads((args.frames.parent/'protocol.json').read_text())['source_raw_sha256']:
            raise ValueError('RGB source does not match this physical episode')
        indices=frames['control_index'][:].astype(int);n=len(raw['observation/timestamp'])
        if np.any(np.diff(indices)!=2):raise ValueError('Expected consecutive 25 FPS samples from 50 Hz control')
        if not np.allclose(frames['timestamp'][:],raw['observation/timestamp'][:][indices]):
            raise ValueError('RGB timestamp mismatch')
        with h5py.File(args.output,'w') as out:
            for k,v in raw.attrs.items():out.attrs[k]=v
            out.attrs['schema']='panda_rgb_action_v1';out.attrs['training_eligible']=False
            out.attrs['source_episode_sha256']=sha256(args.episode)
            out.attrs['source_frames_sha256']=sha256(args.frames)
            out.attrs['rgb_quality']='RT2_preview_temporal_review_incomplete'
            out.attrs['action_interface']='implicit PD joint position targets; two consecutive 50 Hz commands per RGB'
            out.require_group('observation');out.require_group('camera')
            frames.copy('rgb',out,name='observation/rgb')
            frames.copy('world_T_camera_cv',out,name='camera/world_T_camera_cv')
            out['timestamp']=frames['timestamp'][:];out['control_index']=indices
            out['observation/state']=np.concatenate([raw['observation/joint_position'][:][indices],raw['observation/joint_velocity'][:][indices]],axis=-1)
            commands=np.zeros((len(indices),2,18),np.float32);valid=np.zeros((len(indices),2),bool)
            for i,index in enumerate(indices):
                count=min(2,n-index);commands[i,:count]=raw['action/joint_position_target'][index:index+count];valid[i,:count]=True
            out['action/position_target']=commands;out['action/valid']=valid
            out['privileged/object_position_world']=raw['observation/object_position'][:][indices]
            out['privileged/object_quaternion_wxyz']=raw['observation/object_quaternion_wxyz'][:][indices]
            out['phase']=raw['phase'][:][indices]
        # Read back actual bytes and action chunks, not just output shapes.
        with h5py.File(args.output) as check:
            for i,index in enumerate(indices):
                assert np.array_equal(check['observation/rgb'][i],frames['rgb'][i])
                for j in range(2):
                    if check['action/valid'][i,j]:
                        assert np.array_equal(check['action/position_target'][i,j],raw['action/joint_position_target'][index+j])
            assert check['observation/state'].shape==(len(indices),36)
    print(json.dumps({'output':str(args.output),'rgb_frames':len(indices),'state_dim':36,'action_shape':[2,18],
                      'self_contained':True,'readback_verified':True,'training_eligible':False}))


if __name__=='__main__':main()

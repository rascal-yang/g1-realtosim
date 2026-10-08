"""Independently audit saved transitions and make render manifests / batch metrics."""
import argparse
import json
from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

from .common import write_json, sha256


def audit_episode(ep):
    result=json.loads((ep/'result.json').read_text())
    with h5py.File(ep/'episode.h5') as f:
        n=len(f['observation/timestamp']);dt=float(f.attrs['dt'])
        assert np.allclose(f['observation/timestamp'][:],np.arange(n)*dt)
        assert len(f['physics/object_state'])==n*4
        assert np.array_equal(f['physics/control_index'][:],np.repeat(np.arange(n),4))
        assert np.array_equal(f['physics/substep'][:],np.tile(np.arange(4),n))
        for key in f['next_observation']:
            assert np.isfinite(f['next_observation/'+key][:]).all(),key
            assert np.allclose(f['observation/'+key][1:],f['next_observation/'+key][:-1],atol=1e-6),key
        if 'physics/joint_position_target' in f:
            assert np.array_equal(f['physics/joint_position_target'][:],np.repeat(f['action/joint_position_target'][:],4,axis=0))
        assert np.allclose(f['physics/object_state'][3::4,:3],f['next_observation/object_position'][:],atol=1e-6)
        final=f['next_observation/object_position'][-1]
        error=float(np.linalg.norm(final[:2]-np.array(result['goal_position'])[:2]))
        phases=f['phase'][:]
        force=f['physics/hand_object_force_world'][:].reshape(n,4,6,3)
        paired=np.minimum(np.linalg.norm(force[:,:,0:3].sum(axis=2),axis=-1),np.linalg.norm(force[:,:,3:6].sum(axis=2),axis=-1))
        clearance=f['metrics/object_bottom_clearance'][:] if 'metrics/object_bottom_clearance' in f else np.zeros(n)
        run=best=0
        for ok in (clearance>.03)&(paired.min(axis=1)>.1):
            run=run+1 if ok else 0;best=max(run,best)
        # Recompute whole-mesh clearance at 200 Hz from the actual source mesh,
        # independent of the controller's last-substep clearance metric.
        from pxr import Usd,UsdGeom
        from scipy.spatial import ConvexHull
        config=json.loads((ep.parent/'config.json').read_text())
        mesh_stage=Usd.Stage.Open(config['box_usd'])
        points=np.array(UsdGeom.Mesh(mesh_stage.GetPrimAtPath('/Object/Collision')).GetPointsAttr().Get())
        points=points[ConvexHull(points).vertices]
        states=f['physics/object_state'][:]
        rotation=Rotation.from_quat(states[:,[4,5,6,3]]).as_matrix()
        bottom=(rotation[:,2,:]@points.T).min(axis=1)+states[:,2]-.75
        flat_force=force.reshape(-1,6,3);left=flat_force[:,:3].sum(axis=1);right=flat_force[:,3:].sum(axis=1)
        norms=np.linalg.norm(left,axis=1)*np.linalg.norm(right,axis=1)
        cosine=np.sum(left*right,axis=1)/np.maximum(norms,1e-12)
        run200=best200=0
        for ok in (bottom>.03)&(paired.ravel()>.1)&(cosine<-.8):
            run200=run200+1 if ok else 0;best200=max(best200,run200)
        verify=np.flatnonzero(phases==b'verify')[-50:]
        released=len(verify)==50 and np.max(np.linalg.norm(force[verify],axis=-1))<.1 and np.max(np.linalg.norm(f['next_observation/object_velocity'][:][verify,:3],axis=-1))<.025
        success=best200*.005>.5 and released and error<.035 and result.get('collision_event') is None and result.get('final_rotation_error_rad',0)<.2 and abs(final[2]-result['goal_position'][2])<.02
        audited={'success_recomputed':bool(success),'transition_alignment':True,'physics_substeps':n*4,
            'control_steps':n,'last_state_audited':True,'longest_all_substeps_bilateral_clear_s':best*dt,
            'longest_200hz_opposed_grasp_clear_s':best200*.005,'max_whole_mesh_clearance_m':float(bottom.max()),
            'min_whole_mesh_clearance_m':float(bottom.min()),'opposition_cosine_threshold':-.8,
            'final_xy_error_m':error,'released_stable_final_second':bool(released),
            'source_sha256':sha256(ep/'episode.h5'),'training_eligible':False}
        # Strict audit applies to the final schema; early development files lack new metrics.
        if 'metrics/object_bottom_clearance' in f:
            assert bool(success)==bool(result['success']),(str(ep),audited,result['success'])
        write_json(ep/'audit.json',audited)
        camera={'eye':[1.,-1.35,1.7],'target':[0,-.02,1.07]}
        render={'source_raw':str((ep/'episode.h5').resolve()),'stage':str((ep/'initial_scene.usda').resolve()),
            'body_prim_prefix':'/World','object_prim':'/World/Box','fixed_camera':camera,'control_stride':2}
        write_json(ep/'render.json',render)
        # Explicit diagnostic terminal snapshot; never presented as an executed episode.
        terminal=ep/'terminal_state.h5'
        with h5py.File(terminal,'w') as t:
            for k,v in f.attrs.items():t.attrs[k]=v
            t.attrs['schema']='terminal_render_diagnostic';t.attrs['has_actions']=False
            for key in f['next_observation']:t['observation/'+key]=f['next_observation/'+key][-1:]
            t['observation/timestamp']=[n*dt]
        write_json(ep/'render_terminal.json',dict(render,source_raw=str(terminal.resolve()),control_stride=1))
    return dict(result,episode=str(ep),audit=audited)


def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument('--runs',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    all_results=[];wall=0.
    for run in args.runs:
        summary=json.loads((run/'summary.json').read_text());wall+=summary['total_wall_s']
        all_results.extend(audit_episode(ep) for ep in sorted(run.glob('episode_*')) if (ep/'result.json').exists())
    successes=sum(r['success'] for r in all_results)
    report={'attempts':len(all_results),'successes':successes,'success_rate':successes/len(all_results),
        'wall_s_including_startup_and_failures':wall,
        'wall_s_per_success_including_startup_and_failures':wall/successes if successes else None,
        'rollout_wall_s':sum(r['rollout_wall_s'] for r in all_results),
        'video_inputs':0,'video_to_valid_seed_success_rate':None,'human_annotation_time_s':0,
        'human_engineering_time_s':None,'full_locomotion_attempts':0,'full_locomotion_success_rate':None,
        'results':all_results,'cost_exclusions':['rendering is reported separately','historical development runs unless supplied in --runs','unmeasured human engineering time','shutdown after summary write'],
        'training_eligible':False}
    write_json(args.output,report)
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))


if __name__=='__main__':main()

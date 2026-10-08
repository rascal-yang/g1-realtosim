"""Configurable runner for the validated fixed-standing right-handle basket recipe.

Reuses prior robot/assets/controller implementation. Generates NEW geometry;
never grants physical success from the source experiment or from an animation.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time

EVENTS = ['initial', 'settled', 'raise_complete', 'approach_complete', 'closed',
          'lift_complete', 'hold_complete', 'transfer_complete', 'supported',
          'released', 'withdrawn']
FAMILY = 'g1_revo2_right_handle_basket_place_v1'
REPO = Path(__file__).resolve().parents[2]


def validate_spec(s):
    if s.get('skill_family') != FAMILY:
        raise ValueError('Unsupported skill family: add an explicit strategy/controller adapter')
    if [e['event'] for e in s['events']] != EVENTS:
        raise ValueError('Missing, reordered or unsupported contact phases')
    times = [float(e['time_s']) for e in s['events']]
    if not all(math.isfinite(t) for t in times) or times[0] != 0 or any(b <= a for a,b in zip(times,times[1:])):
        raise ValueError('Event times must be finite and strictly increasing from zero')
    if times[-1] != s['duration_s'] or not 0 < s['duration_s'] <= 120:
        raise ValueError('Duration must match final event and be at most120s')
    if type(s['fps']) is not int or not 1 <= s['fps'] <= 120:
        raise ValueError('fps must be integer1..120')
    if any(abs(t*s['fps']-round(t*s['fps'])) > 1e-6 for t in times):
        raise ValueError('Each semantic event must coincide with a sample')
    for key,n in [('root_position_m',3),('basket_initial_position_m',3),('basket_goal_xy_m',2),('grasp_offset_object_m',3),('wrist_euler_xyz',3),('raise_target_m',3),('withdraw_delta_m',3)]:
        if len(s[key]) != n or not all(math.isfinite(float(v)) for v in s[key]):
            raise ValueError('Invalid finite vector: '+key)
    if not .65 < s['closure'] < .8 or not 0 < s['lift_height_m'] <= .2:
        raise ValueError('Closure/lift outside current controller envelope')
    if not math.isfinite(s['root_yaw_rad']) or abs(s['root_yaw_rad']-math.pi/2) > 1e-6:
        raise ValueError('Current physical adapter only supports root yaw pi/2')
    if s['tabletop_aabb_m'] != [[-.56,-.3,.7],[.56,.3,.75]]:
        raise ValueError('This recipe requires the original office desk; new geometry needs an adapter')
    if abs(s['basket_initial_position_m'][2]-.75) > 1e-6:
        raise ValueError('Initial basket bottom must be on the configured desk')


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def generate(spec_path, local_path, output, render=False):
    spec_path=Path(spec_path).resolve();local_path=Path(local_path).resolve();out=Path(output).resolve()
    spec=json.loads(spec_path.read_text());validate_spec(spec)
    local=json.loads(local_path.read_text())
    for k in ['python','blender','sonic_root','template_experiment','imported_scene']:
        if not Path(local[k]).exists():raise FileNotFoundError(k+': '+local[k])
    # Legacy physical collector resolves the workspace repository via this layout.
    if out.parent != REPO.parent/'experiments':
        raise ValueError('Output must be a direct child of workspace/experiments for the reused collector')
    if out.exists():raise FileExistsError('Preserving existing experiment: '+str(out))
    source=Path(local['template_experiment']);recipe=REPO/'recipes/g1_basket_animation'
    started=time.monotonic();out.mkdir();timings=[]
    for d in ['trajectory','reports','runs','renders','datasets','animation/assets','animation/reports','animation/renders']:
        (out/d).mkdir(parents=True,exist_ok=True)
    shutil.copytree(source/'code',out/'code',ignore=shutil.ignore_patterns('__pycache__'))
    (out/'scripts').mkdir()
    for name in ['launch.py','collect.py','pack_capture.py','validate_capture.py','validate_brainco.py','validate_contacts.py','audit_scene.py','postprocess.py','verify_compact.py','audit.py','supervise.py']:
        shutil.copy2(source/'scripts'/name,out/'scripts'/name)
    for p in recipe.glob('*.py'):shutil.copy2(p,out/'scripts'/p.name)
    for name in ['rooms.json','asset_manifest.json']:shutil.copy2(source/name,out/name)
    shutil.copy2(source/'config.json',out/'controller_defaults.json')
    (out/'assets').symlink_to((source/'assets').resolve(),target_is_directory=True)
    (out/'animation/assets/imported_scene.blend').symlink_to(Path(local['imported_scene']).resolve())
    (out/'motion_spec.json').write_text(json.dumps(spec,indent=2))
    (out/'reports/input_provenance.json').write_text(json.dumps({'spec_sha256':digest(spec_path),'source_template':str(source),'imported_scene':local['imported_scene'],'imported_scene_sha256':digest(Path(local['imported_scene'])),'controller_defaults_sha256':digest(out/'controller_defaults.json'),'recipe_sha256':{p.name:digest(p) for p in recipe.glob('*.py')},'runner_sha256':digest(Path(__file__)),'reused':'assets, kinematics and contact controller; not historical time series','physical_success':None},indent=2))
    env=dict(os.environ,**local.get('env',{}),G1_TASK_DIR=str(out),G1_ANIMATION_DIR=str(out/'animation'),G1_SONIC_ROOT=local['sonic_root']);env.pop('DISPLAY',None)
    steps=[('ik',local['python'],'author_motion.py'),('build',local['blender'],'build_animation.py'),('export',local['blender'],'export_animation.py'),('clearance',local['blender'],'check_clearance.py'),('compile',local['python'],'compile_execution.py')]
    if render:steps.append(('render',local['blender'],'render_animation.py'));env['ANIMATION_FULL_RENDER']='1'
    try:
        for name,exe,script in steps:
            cmd=[exe,*(['-b','--python-exit-code','1','--python'] if exe==local['blender'] else []),str(out/'scripts'/script)]
            t=time.monotonic()
            with (out/'reports'/f'{name}.log').open('w') as log:
                result=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
            timings.append({'stage':name,'wall_seconds':time.monotonic()-t,'returncode':result.returncode,'command':cmd})
            print(name,round(timings[-1]['wall_seconds'],3),'seconds',flush=True)
            if result.returncode:raise RuntimeError(name+' failed; preserved '+str(out/'reports'/f'{name}.log'))
        report={'geometry_export_passed':True,'physical_rollouts':0,'physical_success':None,'training_eligible':False,'rendered_animation':render,'stages':timings}
        (out/'reports/generation_result.json').write_text(json.dumps(report,indent=2))
        # Existing audited physical tools are retained, without inventing an automatic review.
        py=local['python'];commands={'launch':[py,str(out/'scripts/launch.py'),'trial_001'], 'supervise_in_second_terminal':[py,str(out/'scripts/supervise.py'),str(out/'runs/trial_001'),'--review-initial'], 'validate_after_completion':[py,str(out/'scripts/postprocess.py'),'trial_001'], 'render_after_success':[py,str(out/'scripts/postprocess.py'),'trial_001','--render','--skip-validation']}
        (out/'physical_commands.json').write_text(json.dumps({'commands':commands,'initial_review':'Inspect actual observation_000.png before creating reviewed_000.json; stop collector if rejected. Never manufacture review.','scope':'Inherited physical commands; generation success is not a physical success result.'},indent=2))
        print('GEOMETRY EXPORTED; physical_success=null; next:',out/'physical_commands.json')
    finally:
        (out/'reports/pipeline_timing.json').write_text(json.dumps({'wall_seconds':time.monotonic()-started,'includes_setup':True,'stages':timings,'GPT_authoring_seconds':None},indent=2))
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--spec',required=True);p.add_argument('--local',required=True);p.add_argument('--output',required=True);p.add_argument('--render',action='store_true');a=p.parse_args()
    generate(a.spec,a.local,a.output,a.render)

if __name__=='__main__':main()

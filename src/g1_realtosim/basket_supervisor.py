"""Deterministic feedback protocol; optional session image review checkpoints."""
from pathlib import Path
import json,time,argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--review',action='store_true');p.add_argument('--review-initial',action='store_true');a=p.parse_args();r=a.run;cfg=json.loads((r/'config.json').read_text());stage='initial';start=0;hold=0

def decision(s):
 global stage,start,hold
 step=s['step'];m=s['grasp_information']['metrics'];palm=np.array(s['palm_positions']['right']);obj=np.array(s['object_pos']);oq=R.from_quat(np.array(s['object_quat_wxyz'])[[1,2,3,0]])
 def act(action='continue',steps=25,goal=None,reason='',next_stage=None):
  global stage,start
  if next_stage:stage=next_stage;start=step
  return dict(action=action,steps=steps,goal=goal or {},reason=reason,stage=stage)
 def review(label):
  if not a.review and not (a.review_initial and stage=='initial'):return
  i=s['observation_id'];marker=r/f'reviewed_{i:03d}.json';(r/f'review_request_{i:03d}.json').write_text(json.dumps({'label':label,'step':step}))
  print('REVIEW',i,label,flush=True)
  while not marker.exists():time.sleep(.4)
 def reach(t,duration,next_stage,action='reach'):
  return act(action,25,{'hand':'right','target':list(t),'wrist_euler_xyz':(R.from_euler('z',cfg['object_yaw_rad'])*R.from_euler('xyz',cfg['wrist_euler_xyz'])).as_euler('xyz').tolist(),'duration_steps':duration,'arm_speed_limit_rad_s':.65,'lift_m':cfg['lift_m']},'Smooth task-space transition with actual-state arm IK',next_stage)
 def converged():
  rows=[json.loads(l) for l in (r/'trajectory.jsonl').read_text().splitlines()[-30:]];ps=np.array([v['palm_positions']['right'] for v in rows]);speed=np.linalg.norm(np.diff(ps,axis=0),axis=1)/.02
  q=np.array(s['wrist_quat_wxyz']['right']);desired=R.from_euler('xyz',s['active_goal']['wrist_euler_xyz']);angle=(desired.inv()*R.from_quat(q[[1,2,3,0]])).magnitude()
  check={'step':step,'position_error_m':s['goal_error_m'],'speed_max_m_s':float(max(speed,default=99)),'orientation_error_rad':float(angle)};check['passed']=bool(s['goal_error_m']<.025 and max(speed,default=99)<.035 and angle<.20)
  with (r/'convergence.jsonl').open('a') as f:f.write(json.dumps(check)+'\n')
  return check['passed']
 if s['contact_watch']['alert']:return act('stop',1,reason='Unexpected hand/forearm table or room contact',next_stage='failed_collision')
 if s['robot_root_pos'][2]<.45:return act('stop',1,reason='Low pelvis guard',next_stage='failed_balance')
 if stage=='initial':
  review('initial bilateral hand and dynamic basket setup');return act(steps=75,reason='Settle physical robot and basket',next_stage='settle')
 if stage=='settle':return reach(cfg['raise_target_m'],cfg['raise_steps'],'raise')
 if stage=='raise':
  if step-start<cfg['raise_steps']+30:return act(reason='Complete side raise outside desk edge')
  review('whole hand above table before forward approach')
  if s['goal_error_m']>.04:return act('stop',1,reason='Side raise tracking failure',next_stage='failed_tracking')
  target=obj+oq.apply(cfg['palm_offset_object_m']);return reach(target,cfg['approach_steps'],'approach')
 if stage=='approach':
  if np.linalg.norm(obj[:2]-np.array(cfg['object_position_m'])[:2])>.06:return act('stop',1,reason='Basket moved; old contact target invalid',next_stage='failed_displacement')
  if step-start<cfg['approach_steps']+30:return act(reason='Slow handle approach')
  if not converged():
   if step-start>cfg['approach_steps']+250:return act('stop',1,reason='Preclose actual wrist convergence timeout',next_stage='failed_tracking')
   return act(steps=10,reason='Bounded wait for actual wrist convergence')
  review('preclose handle alignment')
  return act('close_hand',10,{'closure':cfg['closure'],'closure_steps':90,'finger_stiffness':4},'Close physical Revo2 fingers on handle',next_stage='close')
 if stage=='close':
  if step-start<110:return act(steps=10,reason='Observe contact formation')
  review('handle contacts before proof lift')
  if m['object_hand_normal_force_N']<.2:return act('stop',1,reason='No handle contact after closure',next_stage='failed_grasp')
  t=np.array(s['active_goal']['target']);t[2]+=cfg['lift_m'];return reach(t,180,'lift',action='lift')
 if stage=='lift':
  if step-start<210:return act(reason='Physical proof lift')
  review('whole basket clearance')
  if not m['strict_lift_now']:return act('stop',1,reason='Basket did not lift clear of table',next_stage='failed_lift')
  hold=step;return act(reason='Hold basket while SONIC balances',next_stage='hold')
 if stage=='hold':
  if not m['strict_lift_now']:return act('stop',1,reason='Lost basket clearance/contact during hold',next_stage='failed_hold')
  if step-hold<int(cfg['hold_s']/.02):return act(reason='Continuous final hold')
  review('final hold');return act('stop',1,reason='Completed proof lift and hold; offline audit authoritative',next_stage='complete')
 raise RuntimeError(stage)

while not (r/'result.json').exists():
 files=sorted(r.glob('observation_[0-9][0-9][0-9].json'))
 if files:
  s=json.loads(files[-1].read_text());i=s['observation_id'];out=r/f'decision_{i:03d}.json'
  if not out.exists():
   d=decision(s);d.update(observation_id=i,wall_time=time.time(),author='Deterministic asset-relative basket protocol, no VLM API');tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(d,indent=2));tmp.replace(out);print(step if (step:=s['step']) else 0,stage,d['action'],flush=True)
   (r/'supervisor_state.json').write_text(json.dumps({'stage':stage,'stage_start':start,'step':s['step']},indent=2))
   if d['action']=='stop':break
 time.sleep(.2)

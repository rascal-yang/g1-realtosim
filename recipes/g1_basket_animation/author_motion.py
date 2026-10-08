import os
"""Compile GPT-authored task-space keyframes using existing calibrated robot FK/IK."""
from pathlib import Path
import sys,json,time,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as Rot
B=Path(os.environ['G1_TASK_DIR']);sys.path.insert(0,str(B/'code'))
from online_goals import Kinematics
from calibration_config import ROBOT_SCALE
S=Path(os.environ['G1_SONIC_ROOT']);spec=json.loads((B/'motion_spec.json').read_text());kin=Kinematics(S);t0=time.time()
q=kin.default.copy()
for i,n in enumerate(kin.names):
 if n.endswith('shoulder_pitch_joint'):q[i]=.8
 if n.endswith('elbow_joint'):q[i]=.2
root=np.array(spec['root_position_m']);rq=Rot.from_euler('z',spec['root_yaw_rad']).as_quat()[[3,0,1,2]];kin.fk(q,root,rq);initial_palm=kin.palm('right').copy();wristid=kin.model.body('right_wrist_yaw_link').id
initial_rot=Rot.from_matrix(kin.data.xmat[wristid].reshape(3,3));wrist_rot=Rot.from_euler('xyz',spec['wrist_euler_xyz']);turn=(initial_rot.inv()*wrist_rot).as_rotvec()
obj0=np.array(spec['basket_initial_position_m']);off=np.array(spec['grasp_offset_object_m']);grasp=obj0+off;lift=grasp+[0,0,spec['lift_height_m']];goal_obj=np.r_[spec['basket_goal_xy_m'],obj0[2]];carry=goal_obj+off+[0,0,spec['lift_height_m']];place=goal_obj+off;withdraw=place+np.array(spec['withdraw_delta_m'])
kt=np.array([e['time_s'] for e in spec['events']],dtype=float);kp=np.array([initial_palm,initial_palm,spec['raise_target_m'],grasp,grasp,lift,lift,carry,place,place,withdraw]);ko=np.array([obj0,obj0,obj0,obj0,obj0,obj0+[0,0,spec['lift_height_m']],obj0+[0,0,spec['lift_height_m']],goal_obj+[0,0,spec['lift_height_m']],goal_obj,goal_obj,goal_obj]);kc=np.array([0,0,0,0,*([spec['closure']]*5),0,0])
# URDF hand FK with correct bilateral mount and mimic joints.
hands={}
for side in ['left','right']:
 rt=ET.parse(B/'assets/brainco_candidate'/f'revo2_{side}_clean.urdf').getroot();js=[]
 for j in rt.findall('joint'):
  o=j.find('origin');lim=j.find('limit');mi=j.find('mimic');ax=j.find('axis')
  js.append({'name':j.get('name'),'parent':j.find('parent').get('link'),'child':j.find('child').get('link'),'type':j.get('type'),'pos':np.fromstring(o.get('xyz','0 0 0'),sep=' ')*ROBOT_SCALE,'rot':Rot.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix(),'axis':np.fromstring(ax.get('xyz','1 0 0'),sep=' ') if ax is not None else np.array([1.,0,0]),'closed':max([float(lim.get('lower')),float(lim.get('upper'))],key=abs) if lim is not None else 0,'mimic':mi.attrib if mi is not None else None})
 links=[l.get('name') for l in rt.findall('link')];base=next(n for n in links if n not in [j['child'] for j in js]);hands[side]=(js,links,base)
mount=Rot.from_matrix([[0,0,1],[1,0,0],[0,1,0]]);mounts={'right':mount,'left':Rot.from_euler('x',np.pi)*mount}
body_names=[kin.model.body(i).name for i in range(1,kin.model.nbody)];times=np.arange(round(spec['duration_s']*spec['fps'])+1)/spec['fps'];qs=[];bps=[];bqs=[];ops=[];cls=[];errors=[];palm_targets=[];hp={s:[] for s in hands};hq={s:[] for s in hands};hj={s:[] for s in hands}
for t in times:
 j=min(max(np.searchsorted(kt,t,side='right')-1,0),len(kt)-2);v=np.clip((t-kt[j])/(kt[j+1]-kt[j]),0,1);u=v**3*(10-15*v+6*v*v);target=kp[j]+u*(kp[j+1]-kp[j]);op=ko[j]+u*(ko[j+1]-ko[j]);cl=kc[j]+u*(kc[j+1]-kc[j]);rot=initial_rot*Rot.from_rotvec(turn*(0 if t<=kt[1] else min(1,((np.clip((t-kt[1])/(kt[2]-kt[1]),0,1))**3*(10-15*np.clip((t-kt[1])/(kt[2]-kt[1]),0,1)+6*np.clip((t-kt[1])/(kt[2]-kt[1]),0,1)**2)))))
 if t>kt[1]:q,rep=kin.solve(q,root,rq,'right',target,rot.as_euler('xyz'),allow_waist=False)
 kin.fk(q,root,rq);errors.append(np.linalg.norm(kin.palm('right')-target));qs.append(q.copy());bps.append(kin.data.xpos[1:].copy());bqs.append(kin.data.xquat[1:].copy());ops.append(op);cls.append(cl);palm_targets.append(target)
 for side,(js,links,base) in hands.items():
  i=kin.model.body(side+'_wrist_yaw_link').id;wr=Rot.from_matrix(kin.data.xmat[i].reshape(3,3));trans={base:(kin.data.xpos[i]+wr.apply(np.array([.025,0,0])*ROBOT_SCALE),(wr*mounts[side]).as_matrix())};values={};pending=js.copy()
  while pending:
   progressed=False
   for joint in pending[:]:
    if joint['parent'] not in trans:continue
    mi=joint['mimic']
    if mi and mi['joint'] not in values:continue
    fraction=cl if side=='right' else 0.
    if 'thumb_metacarpal' in joint['name']:fraction=(.75 if side=='right' else .15)*min(t/kt[1],1)
    value=joint['closed']*fraction if not mi else values[mi['joint']]*float(mi.get('multiplier',1))+float(mi.get('offset',0));values[joint['name']]=value
    pp,pr=trans[joint['parent']];trans[joint['child']]=(pp+pr@joint['pos'],pr@joint['rot']@Rot.from_rotvec(joint['axis']*value).as_matrix());pending.remove(joint);progressed=True
   assert progressed
  hp[side].append([trans[n][0] for n in links]);hq[side].append([Rot.from_matrix(trans[n][1]).as_quat()[[3,0,1,2]] for n in links]);hj[side].append([values[j['name']] for j in js if j['type']!='fixed'])
qs=np.array(qs);np.savez_compressed(B/'trajectory/authored_reference.npz',timestamp=times,joint_position=qs,root_position=np.tile(root,(len(times),1)),root_quaternion_wxyz=np.tile(rq,(len(times),1)),body_position=np.array(bps),body_quaternion_wxyz=np.array(bqs),object_position=np.array(ops),object_quaternion_wxyz=np.tile([1.,0,0,0],(len(times),1)),palm_target=np.array(palm_targets),closure=np.array(cls),**{f'{s}_{k}':np.array(v) for s in hands for k,v in [('body_position',hp[s]),('body_quaternion_wxyz',hq[s]),('joint_position',hj[s])]})
meta={'joint_names':kin.names,'body_names':body_names,'joint_body_names':[kin.model.body(int(kin.model.jnt_bodyid[i])).name for i in range(1,kin.model.njnt)],'body_parent_names':[kin.model.body(int(kin.model.body_parentid[i])).name for i in range(1,kin.model.nbody)],'body_rest_quaternion_wxyz':kin.model.body_quat[1:].tolist(),'joint_axes':kin.axes.tolist(),'joint_limits':kin.model.jnt_range[1:].tolist(),'hand_body_names':{s:hands[s][1] for s in hands},'hand_joint_names':{s:[j['name'] for j in hands[s][0] if j['type']!='fixed'] for s in hands},'robot_scale':ROBOT_SCALE,'source':'GPT-authored keyframes plus calibrated MJCF IK, no measured trajectory read','fps':spec['fps']};(B/'trajectory/kinematic_metadata.json').write_text(json.dumps(meta,indent=2))
report={'frames':len(times),'duration_s':times[-1],'max_palm_IK_error_m':float(max(errors)),'joint_limits_passed':bool(np.all(qs>=kin.model.jnt_range[1:,0]-1e-6) and np.all(qs<=kin.model.jnt_range[1:,1]+1e-6)),'max_joint_speed_rad_s':float(np.max(np.abs(np.gradient(qs,1/spec['fps'],axis=0)))),'feet_motion_span_m':{n:float(np.max(np.ptp(np.array(bps)[:,body_names.index(n)],axis=0))) for n in ['left_ankle_roll_link','right_ankle_roll_link']},'wall_seconds':time.time()-t0,'physics_success':None,'collision_check':'not yet performed; FK/IK only'};(B/'reports/author_geometry.json').write_text(json.dumps(report,indent=2));print(report);assert max(errors)<.005 and report['joint_limits_passed']

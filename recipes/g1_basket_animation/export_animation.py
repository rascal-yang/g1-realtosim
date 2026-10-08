import os
"""Reopen .blend and derive joint angles from evaluated link transforms, not source arrays."""
from pathlib import Path
import bpy,json,numpy as np,xml.etree.ElementTree as ET,math
from mathutils import Quaternion,Vector,Euler
A=Path(os.environ['G1_ANIMATION_DIR']);B=Path(os.environ['G1_TASK_DIR']);bpy.ops.wm.open_mainfile(filepath=str(A/'assets/g1_basket_authored.blend'));s=bpy.context.scene;m=json.loads(s['kinematic_metadata_json']);mapping=json.loads(s['rig_mapping_json']);spec=json.loads(s['motion_spec_json']);names=m['body_names'];fps=s.render.fps
hand_joints={}
for side in ['left','right']:
 tree=ET.parse(B/'assets/brainco_candidate'/f'revo2_{side}_clean.urdf').getroot();hand_joints[side]=[j for j in tree.findall('joint') if j.get('type')!='fixed']
def matrix(name):return bpy.data.objects[mapping[name]].matrix_world.copy()
def hinge(rot,axis):
 rot.normalize();
 if rot.w<0:rot.negate()
 v=Vector(rot[1:]);return 2*math.atan2(v.dot(Vector(axis)),rot.w)
rows=[];bodypos=[];bodyquat=[];palms=[]
for frame in range(s.frame_start,s.frame_end+1):
 s.frame_set(frame);bpy.context.view_layer.update();root=matrix('pelvis');rr=root.to_quaternion();angles=[]
 for i,bn in enumerate(m['joint_body_names']):
  bi=names.index(bn);pn=m['body_parent_names'][bi];rq=matrix(pn).to_quaternion().inverted()@matrix(bn).to_quaternion();delta=Quaternion(m['body_rest_quaternion_wxyz'][bi]).inverted()@rq;angles.append(hinge(delta,m['joint_axes'][i]))
 hands={}
 for side,js in hand_joints.items():
  values=[]
  for j in js:
   pr=matrix(side+'/'+j.find('parent').get('link')).to_quaternion();cr=matrix(side+'/'+j.find('child').get('link')).to_quaternion();rpy=[float(x) for x in j.find('origin').get('rpy','0 0 0').split()];rest=Euler(rpy,'XYZ').to_quaternion();delta=rest.inverted()@pr.inverted()@cr;axis=[float(x) for x in j.find('axis').get('xyz').split()];values.append(hinge(delta,axis))
  hands[side]=values
 op=matrix('basket');oq=op.to_quaternion();wp=matrix('right_wrist_yaw_link');palm=wp.translation+wp.to_quaternion()@Vector((.07*m['robot_scale'],0,0));palms.append(list(palm));rows.append([(frame-1)/fps,*root.translation,*rr,*angles,*hands['left'],*hands['right'],*op.translation,*oq]);bodypos.append([list(matrix(n).translation) for n in names if n in mapping]);bodyquat.append([list(matrix(n).to_quaternion()) for n in names if n in mapping])
v=np.array(rows);q=np.unwrap(v[:,8:37],axis=0);l=np.unwrap(v[:,37:48],axis=0);r=np.unwrap(v[:,48:59],axis=0);v[:,8:37]=q;v[:,37:48]=l;v[:,48:59]=r
cols=['time_s','root_x','root_y','root_z','root_qw','root_qx','root_qy','root_qz']+m['joint_names']+m['hand_joint_names']['left']+m['hand_joint_names']['right']+['basket_x','basket_y','basket_z','basket_qw','basket_qx','basket_qy','basket_qz']
np.savetxt(B/'trajectory/g1_from_blender.csv',v,delimiter=',',header=','.join(cols),comments='')
np.savez_compressed(B/'trajectory/g1_from_blender.npz',timestamp=v[:,0],root_position=v[:,1:4],root_quaternion_wxyz=v[:,4:8],joint_position=q,joint_velocity=np.gradient(q,1/fps,axis=0),left_joint_position=l,right_joint_position=r,object_position=v[:,59:62],object_quaternion_wxyz=v[:,62:66],palm_position=np.array(palms),body_position=np.array(bodypos),body_quaternion_wxyz=np.array(bodyquat))
# This comparison verifies export; source arrays were not used to derive the export.
source=np.load(B/'trajectory/authored_reference.npz');err={'joint_rad':float(np.max(np.abs(q-source['joint_position']))),'left_hand_joint_rad':float(np.max(np.abs(l-source['left_joint_position']))),'right_hand_joint_rad':float(np.max(np.abs(r-source['right_joint_position']))),'object_position_m':float(np.max(np.abs(v[:,59:62]-source['object_position'])))}
report={'source_blend':str(A/'assets/g1_basket_authored.blend'),'frames':len(v),'fps':fps,'columns':cols,'roundtrip_error':err,'readback':'evaluated Blender matrices -> parent/rest-relative signed hinge angles','semantics':'kinematic reference, no torques/contact truth/control actions','physics_success':None};(B/'reports/blender_export.json').write_text(json.dumps(report,indent=2));print(report);assert max(err.values())<1e-4

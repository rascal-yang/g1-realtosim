import os
"""Create an editable Blender animation from newly authored G1 FK/IK."""
from pathlib import Path
import bpy,json,numpy as np
from mathutils import Matrix,Quaternion,Vector
A=Path(os.environ['G1_ANIMATION_DIR']);B=Path(os.environ['G1_TASK_DIR'])
bpy.ops.wm.open_mainfile(filepath=str((A/'assets/imported_scene.blend').resolve()))
d=np.load(B/'trajectory/authored_reference.npz');m=json.loads((B/'trajectory/kinematic_metadata.json').read_text());spec=json.loads((B/'motion_spec.json').read_text());scene=bpy.context.scene
# Preserve fixed accessories at their original link-relative transforms.
fixed={'pelvis_contour_link':'pelvis','imu_in_pelvis':'pelvis','head_link':'torso_link','logo_link':'torso_link','imu_in_torso':'torso_link','d435_link':'torso_link','mid360_link':'torso_link'}
for name,parent in fixed.items():
 o=bpy.data.objects.get(name);p=bpy.data.objects.get(parent)
 if o and p:
  local=p.matrix_world.inverted()@o.matrix_world;o.parent=p;o.matrix_parent_inverse=Matrix.Identity(4);o.matrix_basis=local
rig=[];mapping={}
robot=bpy.data.objects['Robot']
for i,name in enumerate(m['body_names']):
 o=next((x for x in robot.children if x.name==name),None)
 if not o:continue
 rig.append((o,d['body_position'][:,i],d['body_quaternion_wxyz'][:,i]));mapping[name]=o.name
for side in ['left','right']:
 parent=bpy.data.objects['BrainCo_'+side]
 for i,name in enumerate(m['hand_body_names'][side]):
  o=next((x for x in parent.children if x.name.split('.')[0].lower()==name.lower()),None)
  if o is None:
   o=next((x for x in parent.children_recursive if x.name.split('.')[0].lower()==name.lower() and x.parent and not x.parent.name.startswith('visual')),None)
  assert o,(side,name)
  rig.append((o,d[side+'_body_position'][:,i],d[side+'_body_quaternion_wxyz'][:,i]));mapping[side+'/'+name]=o.name
obj=bpy.data.objects['Object'];rig.append((obj,d['object_position'],d['object_quaternion_wxyz']));mapping['basket']=obj.name
for o,p,q in rig:
 scale=o.matrix_world.to_scale().copy();o.parent=None;o.matrix_parent_inverse=Matrix.Identity(4);o.rotation_mode='QUATERNION';o.scale=scale;o['animation_semantics']='Authored kinematics, not a physical state/action record'
 prev=None
 for f,(pos,quat) in enumerate(zip(p,q),1):
  quat=np.asarray(quat).copy()
  if prev is not None and np.dot(quat,prev)<0:quat=-quat
  o.location=pos.tolist();o.rotation_quaternion=quat.tolist();o.keyframe_insert('location',frame=f);o.keyframe_insert('rotation_quaternion',frame=f);prev=quat
 for fc in o.animation_data.action.fcurves:
  for key in fc.keyframe_points:key.interpolation='LINEAR'
# Add explicit semantic event markers; objects in grasp phase are kinematic constraints only.
scene.render.fps=spec['fps'];scene.frame_start=1;scene.frame_end=len(d['timestamp'])
for event in spec['events']:scene.timeline_markers.new(event['event'],frame=round(event['time_s']*spec['fps'])+1)
scene['motion_spec_json']=json.dumps(spec);scene['rig_mapping_json']=json.dumps(mapping);scene['kinematic_metadata_json']=json.dumps(m);scene['physics_success']=False
# Single fixed camera; entire G1 and table are visible.
for o in list(bpy.data.objects):
 if o.type in ['CAMERA','LIGHT']:bpy.data.objects.remove(o,do_unlink=True)
cam_data=bpy.data.cameras.new('AnimationCamera');cam=bpy.data.objects.new('AnimationCamera',cam_data);scene.collection.objects.link(cam);cam.location=(1.30,-1.65,1.35);direction=Vector((.24,-.25,.74))-cam.location;cam.rotation_euler=direction.to_track_quat('-Z','Y').to_euler();cam.data.lens=28;scene.camera=cam
for name,loc,power,size in [('Key',(1,-1.5,2.6),210,3),('Fill',(-.9,-.5,2),105,2)]:
 l=bpy.data.lights.new(name,'AREA');l.energy=power;l.shape='DISK';l.size=size;o=bpy.data.objects.new(name,l);scene.collection.objects.link(o);o.location=loc;o.rotation_euler=(Vector((.2,-.1,.7))-o.location).to_track_quat('-Z','Y').to_euler()
scene.world=bpy.data.worlds.new('StudioWorld');scene.world.use_nodes=True;scene.world.node_tree.nodes['Background'].inputs['Color'].default_value=(.18,.18,.18,1);scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.3
scene.render.engine='BLENDER_EEVEE_NEXT';scene.eevee.taa_render_samples=16;scene.view_settings.exposure=-1.2;scene.render.resolution_x=960;scene.render.resolution_y=720;scene.render.resolution_percentage=100
scene.view_settings.view_transform='AgX';scene.render.image_settings.file_format='PNG';scene.render.film_transparent=False
bpy.ops.file.pack_all();scene.frame_set(1);bpy.ops.wm.save_as_mainfile(filepath=str(A/'assets/g1_basket_authored.blend'))
(A/'reports/rig_mapping.json').write_text(json.dumps(mapping,indent=2));(A/'reports/build.json').write_text(json.dumps({'animated_link_count':len(rig),'frames':scene.frame_end,'fps':scene.render.fps,'asset_geometry_source':'prior initial USD snapshot, not recorded motion','keyframes_source':str(B/'motion_spec.json'),'joint_trajectory_copied_from_success':False,'physical_success':None},indent=2))
for frame in ([1,round(scene.frame_end/2),scene.frame_end] if os.environ.get('G1_BUILD_PREVIEWS')=='1' else []):
 scene.frame_set(frame);scene.render.filepath=str(A/'renders'/f'preview_{frame:04d}.png');bpy.ops.render.render(write_still=True)
print('BLEND_ANIMATION_COMPLETE',flush=True)

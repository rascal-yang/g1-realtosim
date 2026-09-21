"""Photo-constrained scene, editable geometry and portable textures. Run with Blender."""
from pathlib import Path
import os
import bpy, math, json, numpy as np
from mathutils import Matrix,Vector
B=Path(os.environ['REALTOSIM_CASE_DIR']).resolve();A=B/'assets';T=A/'textures';V=A/'visuals';V.mkdir(exist_ok=True)
fit=json.loads((A/'layout_fit.json').read_text());W,D,H=fit['table_dimensions_m'];winfit=json.loads((A/'window_fit.json').read_text())
rng=np.random.default_rng(31);bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene;scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1
manifest=[];collisions=[]
def group(name,loc=(0,0,0)):
 o=bpy.data.objects.new(name,None);scene.collection.objects.link(o);o.location=loc;return o
room=group('Architecture');desk=group('Desk');basket=group('Basket',(.10,-.05,.752));details=group('Details')
def material(name,color,rough=.5,metal=0,tex=None,normal=None,roughtex=None,transmission=0):
 m=bpy.data.materials.new(name);m.use_nodes=True;n=m.node_tree.nodes;links=m.node_tree.links;p=n.get('Principled BSDF');p.inputs['Base Color'].default_value=(*color,1);p.inputs['Roughness'].default_value=rough;p.inputs['Metallic'].default_value=metal
 p.inputs['Transmission Weight'].default_value=transmission;p.inputs['IOR'].default_value=1.47
 for typ,file in [('Base Color',tex),('Normal',normal),('Roughness',roughtex)]:
  if file:
   node=n.new('ShaderNodeTexImage');node.image=bpy.data.images.load(str(T/file),check_existing=True)
   if typ!='Base Color':node.image.colorspace_settings.name='Non-Color'
   if typ=='Normal':
    nm=n.new('ShaderNodeNormalMap');nm.inputs['Strength'].default_value=.5;links.new(node.outputs['Color'],nm.inputs['Color']);links.new(nm.outputs['Normal'],p.inputs['Normal'])
   else:links.new(node.outputs['Color'],p.inputs[typ])
 return m
wall=material('Warm_offwhite_plaster',(.64,.62,.57),.92);trim=material('Painted_skirting',(.58,.59,.55),.64)
wood=material('Photo_light_wood',(.6,.48,.3),.4,tex='wood_basecolor.png',normal='wood_normal.png',roughtex='wood_roughness.png')
steel=material('Dark_blue_powdercoat',(.022,.037,.043),.44,.25);black=material('Black_rubber',(.006,.009,.010),.76)
rubber=material('Photo_blue_fleck_rubber',(.03,.04,.045),.78,tex='rubber_basecolor.png',normal='rubber_normal.png',roughtex='rubber_roughness.png')
pvc=material('Warm_white_window_frame',(.67,.7,.70),.32);aluminium=material('Brushed_aluminium',(.42,.45,.46),.27,.82)
glass=material('Window_clear_glass',(.92,.98,1),.035,transmission=1)
willow=[material('Willow_%02d'%i,(.37+i*.018,.139+i*.009,.040+i*.004),.45) for i in range(7)]
cream=material('Appliance_enamel',(.62,.64,.62),.34,.1);apglass=material('Appliance_dark_window',(.009,.021,.029),.19,.2)
ext=material('Exterior_photo_unmeasured',(.7,.7,.7),1,tex='exterior_reference.jpg')
node=next(n for n in ext.node_tree.nodes if n.type=='TEX_IMAGE');bs=ext.node_tree.nodes.get('Principled BSDF');ext.node_tree.links.new(node.outputs['Color'],bs.inputs['Emission Color']);bs.inputs['Emission Strength'].default_value=2.5
def uvbox(o,size):
 uv=o.data.uv_layers.active or o.data.uv_layers.new()
 for p in o.data.polygons:
  normal=np.array(p.normal);ids=[i for i in range(3) if i!=int(np.argmax(abs(normal)))]
  for li in p.loop_indices:
   v=o.data.vertices[o.data.loops[li].vertex_index].co;uv.data[li].uv=(v[ids[0]]/max(size[ids[0]],1e-6)+.5,v[ids[1]]/max(size[ids[1]],1e-6)+.5)
def cube(name,loc,size,mat,parent=room,bevel=.001,collision=False,rot=0):
 bpy.ops.mesh.primitive_cube_add(size=1,location=loc);o=bpy.context.object;o.name=name;o.dimensions=size;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);uvbox(o,size);o.rotation_euler.z=rot;o.parent=parent;o.data.materials.append(mat)
 if name.startswith('Wall_'):bevel=0
 if bevel:
  mod=o.modifiers.new('Manufactured_edge_bevel','BEVEL');mod.width=bevel;mod.segments=3
  mod=o.modifiers.new('Weighted_corner_normals','WEIGHTED_NORMAL');mod.keep_sharp=True
 if collision:collisions.append(name)
 o['semantic_class']=parent.name.lower();o['evidence']='photo constrained; metric scale uncalibrated'
 manifest.append({'name':name,'group':parent.name,'local_position_m':list(loc),'size_m':list(size),'yaw_rad':rot,'material':mat.name,'collision':collision})
 return o
def mesh(name,verts,faces,mat,parent):
 me=bpy.data.meshes.new(name);me.from_pydata(verts,[],faces);me.update();o=bpy.data.objects.new(name,me);scene.collection.objects.link(o);o.parent=parent;me.materials.append(mat)
 for p in me.polygons:p.use_smooth=True
 o['semantic_class']=parent.name.lower();return o
def tubes(name,paths,radius,mat,parent,sides=6):
 vs=[];fs=[]
 for path in paths:
  p=np.asarray(path);t=np.gradient(p,axis=0);t/=np.maximum(np.linalg.norm(t,axis=1,keepdims=True),1e-12);g=np.tile([0.,0,1],(len(p),1));g[abs(t[:,2])>.9]=[0,1,0];u=np.cross(t,g);u/=np.maximum(np.linalg.norm(u,axis=1,keepdims=True),1e-12);v=np.cross(t,u);angle=np.arange(sides)*2*np.pi/sides
  rings=p[:,None,:]+radius*(np.cos(angle)[None,:,None]*u[:,None,:]+np.sin(angle)[None,:,None]*v[:,None,:]);off=len(vs);vs.extend(rings.reshape(-1,3).tolist())
  for i in range(len(p)-1):
   for j in range(sides):a=off+i*sides+j;b=off+i*sides+(j+1)%sides;fs.append((a,b,b+sides,a+sides))
 return mesh(name,vs,fs,mat,parent)
# Floor of individually beveled estimated 1m rubber mats, with real photographed granules.
cube('Floor_foundation',(-.9,-1.6,-.025),(6.2,4,.05),black,collision=True,bevel=0)
for i,x in enumerate(np.arange(-3.6,2,1.0)):
 for j,y in enumerate(np.arange(-3.60,.9,1.0)):
  o=cube(f'Mat_{i:02d}_{j:02d}',(x,y,-.006),(.998,.998,.012),rubber,bevel=.0007)
  # UV rotations prevent repeating the same grain orientation on every tile.
  for u in o.data.uv_layers.active.data:
   a,b=u.uv[:];u.uv=(2*b,2*(1-a)) if (i+j)%2 else (2*a,2*b)
# Back wall opening, anchored to inferred sill height, not the old oversized window.
Y=D/2+.045;xl=-3.9;xr=1.90;wx1=-.46;wx0=wx1-2*(winfit['leaf_width_m']+.068);zb=.92;zt=2.22
cube('Wall_below_window',((xl+xr)/2,Y,zb/2),(xr-xl,.12,zb),wall,collision=True)
cube('Wall_above_window',((xl+xr)/2,Y,(zt+2.85)/2),(xr-xl,.12,2.85-zt),wall,collision=True)
cube('Wall_left_window',((xl+wx0)/2,Y,(zb+zt)/2),(wx0-xl,.12,zt-zb),wall,collision=True)
cube('Wall_right_window',((wx1+xr)/2,Y,(zb+zt)/2),(xr-wx1,.12,zt-zb),wall,collision=True)
cube('Right_return_wall',(xr,-1.55,1.425),(.12,3.9,2.85),wall,collision=True)
cube('Column',(xr-.105,-1.58,1.425),(.25,.68,2.85),wall,collision=True)
cube('Skirting_back',((xl+xr)/2,Y-.069,.027),(xr-xl,.017,.054),trim)
cube('Skirting_right',(xr-.069,-1.55,.027),(.017,3.9,.054),trim)
cube('Column_skirting_front',(xr-.24,-1.58,.027),(.016,.71,.054),trim)
for yy in [-1.927,-1.233]:cube('Column_skirting_side_'+str(yy),(xr-.115,yy,.027),(.24,.017,.054),trim)
# Unseen enclosure retained with explicit inferred classification.
inferred=group('Inferred_enclosure')
cube('Ceiling',(-1,-1.55,2.88),(6,3.9,.06),wall,parent=inferred,collision=True)
cube('Unobserved_left_wall',(-3.96,-1.55,1.425),(.12,3.9,2.85),wall,parent=inferred,collision=True)
cube('Unobserved_front_wall',(-1,-3.53,1.425),(6,.12,2.85),wall,parent=inferred,collision=True)
# Desk top, separate edged laminate, steel square tube frame and rubber end caps.
cube('Tabletop',(0,0,.7375),(W,D,.025),wood,parent=desk,bevel=.0015,collision=True)
for x in [-W/2+.035,W/2-.035]:
 for y in [-D/2+.035,D/2-.035]:
  suffix=f'{x:.3f}_{y:.3f}';cube('Leg_'+suffix,(x,y,.36),(.043,.043,.72),steel,desk,.0013,True)
  cube('Foot_'+suffix,(x,y,.005),(.047,.047,.011),black,desk,.002)
for y in [-D/2+.035,D/2-.035]:cube('Long_apron_'+str(y),(0,y,.693),(W-.045,.041,.050),steel,desk,.001,True)
for x in [-W/2+.035,W/2-.035]:cube('Short_apron_'+str(x),(x,0,.693),(.041,D-.045,.05),steel,desk,.001,True)
# Recessed cable hatch with frame, brush and hinged lid.
cube('Cable_hatch_frame',(-.22,.198,.752),(.275,.111,.004),steel,desk,.002)
cube('Cable_hatch_recess',(-.22,.198,.754),(.247,.086,.002),black,desk,.001)
cube('Cable_hatch_lid',(-.22,.217,.755),(.239,.045,.003),steel,desk,.001)
for i in range(65):cube('Cable_brush_%02d'%i,(-.336+i*.0036,.167,.756),(.0011,.014,.001),black,desk,0)
# Window: white multi-layer frame, dark seals, open right sash and real glass panes.
window=group('Window')
for name,loc,size in [('Sill',((wx0+wx1)/2,Y-.058,zb-.012),(wx1-wx0+.09,.29,.029)),('Bottom',((wx0+wx1)/2,Y-.009,zb+.029),(wx1-wx0,.092,.058)),('Top',((wx0+wx1)/2,Y-.009,zt-.029),(wx1-wx0,.092,.058)),('Left',(wx0+.029,Y-.009,(zb+zt)/2),(.058,.092,zt-zb)),('Right',(wx1-.029,Y-.009,(zb+zt)/2),(.058,.092,zt-zb))]:cube('Window_'+name,loc,size,pvc,window,.0015,True)
mid=(wx0+wx1)/2
cube('Window_mullion',(mid,Y-.009,(zb+zt)/2),(.058,.095,zt-zb),pvc,window,.0015,True)
cube('Window_fixed_glass',((wx0+mid)/2,Y,(zb+zt)/2),(mid-wx0-.115,.006,zt-zb-.12),glass,window,.0003)
for xx in [wx0+.060,mid-.060]:cube('Fixed_black_seal_'+str(xx),(xx,Y-.053,(zb+zt)/2),(.008,.009,zt-zb-.10),black,window,0)
leaf=group('Open_sash',(wx1-.048,Y-.055,0));leaf.parent=window;leaf.rotation_euler.z=math.radians(winfit['yaw_degrees'])
lw=winfit['leaf_width_m'];bottom=winfit['bottom_edge_z_m'];top=zt-.025;lh=top-bottom;middle=(top+bottom)/2;lc=-lw/2
for name,loc,size in [('Bottom',(lc,0,bottom+.027),(lw,.065,.054)),('Top',(lc,0,top-.027),(lw,.065,.054)),('Hinge',(0,0,middle),(.054,.065,lh)),('Latch',(-lw,0,middle),(.054,.065,lh))]:cube('Sash_'+name,loc,size,pvc,leaf,.0014,True)
cube('Sash_clear_glass',(lc,.008,middle),(lw-.059,.005,lh-.058),glass,leaf,.0002)
for xx in [-lw+.03,-.03]:cube('Sash_gasket_'+str(xx),(xx,-.035,middle),(.009,.009,lh-.045),black,leaf,.0005)
cube('Sash_handle',(-lw+.055,-.055,1.56),(.023,.034,.135),aluminium,leaf,.006)
for zz in [zb+.18,zt-.18]:cube('Sash_hinge_'+str(zz),(.025,.005,zz),(.024,.055,.09),aluminium,leaf,.005)
# A distant exterior photograph is a visual background; no outdoor geometry is claimed.
cube('Exterior_photo_card',(-1.50,2.5,1.75),(4,.005,3.2),ext,window,0)
t=np.linspace(0,2*np.pi,140);tubes('Blind_pull_loop',[np.c_[np.full_like(t,wx1+.075),Y-.11+.012*np.cos(t),1.80+.30*np.sin(t)]],.0012,pvc,window,5)
# Wicker basket: alternating over/under strands, vertical stakes, densely wrapped rim.
rx,ry,bh=.145,.103,.130;theta=np.linspace(0,2*np.pi,321)
for m in range(7):
 paths=[]
 for row in range(32):
  if row%7!=m:continue
  z=.007+row*.00345;sc=.79+.21*(z/bh)**.6;wave=.0017*np.cos(48*theta+row*np.pi)
  paths.append(np.c_[(rx*sc+wave)*np.cos(theta),(ry*sc+wave)*np.sin(theta),z+.00035*np.sin(theta*5+row)])
 tubes('Basket_weft_%d'%m,paths,.00138,willow[m],basket,7)
paths=[]
for j,a in enumerate(np.linspace(0,2*np.pi,96,endpoint=False)):
 z=np.linspace(.003,bh,65);sc=.79+.21*(z/bh)**.6;v=.0011*np.cos(z/.00345*np.pi+j*np.pi);paths.append(np.c_[(rx*sc+v)*np.cos(a),(ry*sc+v)*np.sin(a),z])
tubes('Basket_stakes',paths,.0010,willow[3],basket,6)
# Broad top binding actually consists of close, flattened loops around the rim band.
paths=[]
for a in np.linspace(0,2*np.pi,410,endpoint=False):
 b=np.linspace(0,2*np.pi,17);rad=.0038*np.cos(b);z=bh-.002+.014*np.sin(b);paths.append(np.c_[(rx+rad)*np.cos(a),(ry+rad)*np.sin(a),z])
tubes('Basket_bound_rim',paths,.00092,willow[4],basket,6)
paths=[]
for x in np.arange(-rx*.79,rx*.79,.0032):
 yy=ry*.79*np.sqrt(max(0,1-(x/(rx*.79))**2));ys=np.linspace(-yy,yy,70);paths.append(np.c_[np.full_like(ys,x),ys,.003+.0005*np.sin(ys*900)])
for y in np.arange(-ry*.79,ry*.79,.0032):
 xx=rx*.79*np.sqrt(max(0,1-(y/(ry*.79))**2));xs=np.linspace(-xx,xx,70);paths.append(np.c_[xs,np.full_like(xs,y),.004+.0005*np.sin(xs*900)])
tubes('Basket_woven_base',paths,.0011,willow[2],basket,6)
# Flat wrapped handle across the short axis. Individually modeled wraps catch highlights.
t=np.linspace(0,np.pi,180);path=np.c_[np.zeros_like(t),ry*np.cos(t),bh+.183*np.sin(t)];tubes('Basket_handle_core',[path],.0045,willow[3],basket,10)
paths=[]
for a in np.linspace(0,np.pi,245):
 p=np.array([0,ry*np.cos(a),bh+.183*np.sin(a)]);tang=np.array([0,-ry*np.sin(a),.183*np.cos(a)]);tang/=np.linalg.norm(tang);u=np.array([1,0,0]);v=np.cross(tang,u);phi=np.linspace(0,2*np.pi,15);paths.append(p+.010*np.cos(phi)[:,None]*u+.0042*np.sin(phi)[:,None]*v)
tubes('Basket_handle_wraps',paths,.00072,willow[5],basket,6)
# Partially observed equipment and loose floor mats.
appliance=group('Appliance',(-3.18,-.05,0))
cube('Equipment_body',(0,0,.405),(.49,.5,.80),cream,appliance,.019,True)
cube('Equipment_front_window',(0,-.259,.43),(.378,.014,.41),apglass,appliance,.028)
for z in [.60,.63,.66]:cube('Equipment_vent_'+str(z),(0,-.27,z),(.34,.012,.008),aluminium,appliance,.002)
cube('Equipment_top_tray',(0,.02,.825),(.38,.34,.03),black,appliance,.006)
cube('Equipment_top_unit',(0,.04,.91),(.30,.25,.14),apglass,appliance,.025)
cube('Equipment_tray_back',(0,.15,1.035),(.33,.015,.16),black,appliance,.004)
cube('Wall_outlet',(-2.59,Y-.072,.72),(.071,.025,.107),pvc,details,.004)
cube('Plug',(-2.59,Y-.103,.709),(.029,.042,.047),black,details,.009)
from mathutils.geometry import interpolate_bezier
ctrl=[[-2.59,Y-.135,.70],[-2.65,.08,.36],[-2.56,-.06,.07],[-2.97,-.15,.017]]
pts=[]
for j in range(120):
 t=j/119;pts.append((1-t)**3*np.array(ctrl[0])+3*(1-t)**2*t*np.array(ctrl[1])+3*(1-t)*t*t*np.array(ctrl[2])+t**3*np.array(ctrl[3]))
tubes('Power_cable',[pts],.0025,black,details,7)
for i in range(2):cube('Loose_mat_'+str(i),(-2.6+i*.09,-.65+i*.04,.014+i*.022),(.43,.28,.022),rubber,details,.002,True,rot=.19+i*.17)
cube('Paper_scrap',(.45,-1.11,.0015),(.023,.013,.002),pvc,details,.001,rot=.5)
# Export architecture before adding the reusable plush, cameras and lights.
bpy.ops.object.select_all(action='SELECT')
bpy.ops.wm.usd_export(filepath=str(V/'room.usdc'),selected_objects_only=True,root_prim_path='/Asset',export_textures=True,overwrite_textures=True,relative_paths=True,export_materials=True,generate_preview_surface=True,export_custom_properties=True,export_lights=False,export_cameras=False,triangulate_meshes=True)
# Editable blend includes the same toy pose; the final USD uses its original physics asset.
bpy.ops.object.select_all(action='DESELECT');bpy.ops.import_scene.gltf(filepath=str(A/'plush/source_visual.glb'))
plushobjs=list(bpy.context.selected_objects);rot=Matrix.Rotation(math.pi/2,4,'Z')@Matrix.Rotation(-math.pi/2,4,'X');scale=.90
coords=[]
for o in plushobjs:
 if o.type=='MESH':coords.extend([rot.to_3x3()@(v.co*scale) for v in o.data.vertices])
coords=np.array(coords);offset=np.array([-.17,-.14,.755-coords[:,2].min()]);# center based on rotated mesh bounds in xy
mid=(coords.min(0)+coords.max(0))/2;offset[:2]-=mid[:2]
for o in plushobjs:o.matrix_world=Matrix.Translation(Vector(offset))@rot@Matrix.Diagonal((scale,scale,scale,1))
(A/'plush_placement.json').write_text(json.dumps({'translation_m':offset.tolist(),'quaternion_wxyz':[.5,-.5,-.5,.5],'scale':scale,'physics':'rigid approximate plush; softness not reconstructed'},indent=2))
for k,c in fit['cameras'].items():
 bpy.ops.object.camera_add();o=bpy.context.object;o.name='PhotoCamera_'+k;o.matrix_world=Matrix(c['camera_to_world_blender']);o.data.sensor_width=36;o.data.lens=c['intrinsics'][0][0]*36/1706
 if k=='00':scene.camera=o
for name,loc,energy,size,color,target in [('Window_daylight',(-1.35,.18,1.7),220,1.5,(.88,.93,1),(-.2,-1,.7)),('Ceiling_bounce',(-.7,-.8,2.65),95,2.2,(1,.94,.83),(0,0,.6))]:
 bpy.ops.object.light_add(type='AREA',location=loc);o=bpy.context.object;o.name=name;o.data.energy=energy;o.data.shape='DISK';o.data.size=size;o.data.color=color;o.rotation_euler=(Vector(target)-o.location).to_track_quat('-Z','Y').to_euler()
scene.render.engine='CYCLES';scene.cycles.samples=64;scene.view_settings.view_transform='AgX';scene.world.color=(.10,.12,.16);scene.render.resolution_x=1706;scene.render.resolution_y=1279;scene.render.resolution_percentage=100
bpy.ops.file.pack_all();bpy.ops.wm.save_as_mainfile(filepath=str(A/'scene.blend'))
(A/'geometry_manifest.json').write_text(json.dumps({'components':manifest,'static_collision_names':collisions,'table_dimensions_m':[W,D,H],'basket':{'center':list(basket.location),'radii_m':[rx,ry],'body_height_m':bh,'handle_rise_m':.183},'wall_y':Y,'window_opening':[wx0,wx1,zb,zt],'plush_translation_m':offset.tolist(),'parameter_source':'photo proportions and fitted desk cameras; absolute dimensions and physical parameters estimated'},indent=2))
print('BLENDER_SCENE_COMPLETE',len(manifest),'components',flush=True)

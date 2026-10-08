import os
import bpy,json,gpu,os
from pathlib import Path
A=Path(os.environ['G1_ANIMATION_DIR']);bpy.ops.wm.open_mainfile(filepath=str(A/'assets/g1_basket_authored.blend'));s=bpy.context.scene
s.camera.data.lens=28;s.render.resolution_x=640;s.render.resolution_y=480;s.eevee.taa_render_samples=16;s.view_settings.exposure=-1.2
for o in bpy.data.objects:
 if o.type=='LIGHT':o.data.energy={'Key':210,'Fill':105}.get(o.name,100)
s.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.3
s.frame_set(1);s.render.filepath=str(A/'renders/final_preview.png');bpy.ops.render.render(write_still=True)
(A/'reports/blender_renderer.json').write_text(json.dumps({'engine':s.render.engine,'samples':s.eevee.taa_render_samples,'GPU_vendor':gpu.platform.vendor_get(),'GPU_renderer':gpu.platform.renderer_get(),'GPU_version':gpu.platform.version_get(),'resolution':[640,480],'fps':s.render.fps},indent=2))
bpy.ops.wm.save_as_mainfile(filepath=str(A/'assets/g1_basket_authored.blend'))
if os.environ.get('ANIMATION_FULL_RENDER')=='1':
 s.render.image_settings.file_format='FFMPEG';s.render.ffmpeg.format='MPEG4';s.render.ffmpeg.codec='H264';s.render.ffmpeg.constant_rate_factor='MEDIUM';s.render.filepath=str(A/'renders/g1_basket_authored.mp4');bpy.ops.render.render(animation=True)

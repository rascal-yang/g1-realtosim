"""Generate a mesh and UV base-color atlas from the isolated, original toy photograph."""
import argparse
import gc
import json
import os
from pathlib import Path
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument('--input', type=Path, required=True, help='Isolated RGBA reference image')
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--source-root', type=Path, required=True, help='Hunyuan3D-2 checkout')
parser.add_argument('--models', type=Path, required=True, help='Local model cache')
parser.add_argument('--stage', choices=['shape','paint','all'], default='all')
parser.add_argument('--seed', type=int, default=42)
args = parser.parse_args()
OUT = args.output.resolve()
if args.stage in ('shape','all'):
    OUT.mkdir(parents=True, exist_ok=False)
elif not (OUT/'shape.glb').exists():
    raise FileNotFoundError('paint requires existing output/shape.glb')
sys.path.insert(0, str(args.source_root.resolve()))
os.environ.setdefault('HF_HUB_DISABLE_XET', '1')
import torch
import trimesh
from PIL import Image
models = args.models.resolve()
source = Image.open(args.input).convert('RGBA')
source.save(OUT/'input_rgba.png')
metadata_path = OUT/'generation.json'
metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
metadata['input_source'] = str(args.input.resolve())
torch.set_num_threads(8)

if args.stage in ('shape', 'all'):
    from hy3dgen.shapegen import Hunyuan3DDiTFlowMatchingPipeline, FaceReducer, FloaterRemover, DegenerateFaceRemover
    start = time.time()
    pipe = Hunyuan3DDiTFlowMatchingPipeline.from_pretrained(str(models), use_safetensors=True, variant='fp16')
    mesh = pipe(image=source, num_inference_steps=40, guidance_scale=5.0, octree_resolution=384,
                num_chunks=8000, generator=torch.Generator(device='cuda').manual_seed(args.seed))[0]
    mesh.export(OUT / 'shape_raw.glb')
    mesh = FloaterRemover()(mesh)
    mesh = DegenerateFaceRemover()(mesh)
    mesh = FaceReducer()(mesh, max_facenum=60000)
    mesh.export(OUT / 'shape.glb')
    metadata['shape'] = {'model': 'tencent/Hunyuan3D-2/hunyuan3d-dit-v2-0', 'seed': args.seed,
        'steps': 40, 'octree_resolution': 384, 'guidance_scale': 5.0,
        'vertices': len(mesh.vertices), 'faces': len(mesh.faces), 'seconds': time.time() - start,
        'bounds_native': mesh.bounds.tolist()}
    metadata_path.write_text(json.dumps(metadata, indent=2))
    print('SHAPE_DONE', metadata['shape'], flush=True)
    del pipe
    gc.collect()
    torch.cuda.empty_cache()

if args.stage in ('paint', 'all'):
    from hy3dgen.texgen import Hunyuan3DPaintPipeline
    start = time.time()
    # The upstream custom loader requires .bin even with official safetensors.
    unet_folder = models / 'hunyuan3d-paint-v2-0-turbo/unet'
    if not (unet_folder / 'diffusion_pytorch_model.bin').exists():
        raise RuntimeError('This pinned Hunyuan paint loader requires a compatible local .bin cache. See docs/ENVIRONMENT.md; do not convert shared weights automatically.')
    mesh = trimesh.load(OUT / 'shape.glb', force='mesh')
    pipe = Hunyuan3DPaintPipeline.from_pretrained(str(models), subfolder='hunyuan3d-paint-v2-0-turbo')
    delight = pipe.models['delight_model']
    delight_call = type(delight).__call__
    def record_delight(self, *a, **kw):
        result = delight_call(self, *a, **kw)
        result.save(OUT / 'delighted_reference.png')
        return result
    type(delight).__call__ = record_delight
    # Save the generated views as provenance instead of treating them as measurements.
    model = pipe.models['multiview_model']
    original_call = type(model).__call__
    def record_views(self, *a, **kw):
        views = original_call(self, *a, **kw)
        folder = OUT / 'inferred_views'
        folder.mkdir(exist_ok=True)
        for index, im in enumerate(views):
            im.save(folder / f'view_{index:02d}.png')
        return views
    type(model).__call__ = record_views
    textured = pipe(mesh, image=source)
    textured.export(OUT / 'textured_raw.glb')
    texture = getattr(textured.visual.material, 'image', None)
    if texture is None:
        texture = textured.visual.material.baseColorTexture
    texture.save(OUT / 'generated_basecolor.png')
    metadata['paint'] = {'model': 'tencent/Hunyuan3D-2/hunyuan3d-paint-v2-0-turbo',
        'texture_size': list(texture.size), 'seconds': time.time() - start,
        'note': 'Single-photo-conditioned multi-view inference. Hidden colors and shape are inferred.'}
    metadata_path.write_text(json.dumps(metadata, indent=2))
    print('PAINT_DONE', metadata['paint'], flush=True)

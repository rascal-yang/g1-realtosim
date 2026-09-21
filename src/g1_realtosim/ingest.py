"""Inventory photos/videos and extract bounded, timestamped video references."""
import subprocess
from pathlib import Path
from PIL import Image, ImageOps
from .common import fresh_dir, sha256, write_json


def ingest(source, output, interval=2.0, max_frames=60):
    if interval <= 0 or max_frames < 1:
        raise ValueError('interval and max_frames must be positive')
    source = Path(source).resolve()
    paths = sorted(source.iterdir()) if source.is_dir() else [source]
    out = fresh_dir(output)
    (out/'references').mkdir()
    records = []
    for path in paths:
        if not path.is_file():
            continue
        record = {'source': str(path), 'sha256': sha256(path), 'references': []}
        try:
            with Image.open(path) as im:
                rgb = ImageOps.exif_transpose(im).convert('RGB')
                name = f'{len(records):03d}_{path.stem}.png'
                rgb.save(out/'references'/name)
                record.update(kind='image', size=list(rgb.size))
                record['references'].append('references/'+name)
        except (OSError, ValueError):
            if path.suffix.lower() not in {'.mp4', '.mov', '.avi', '.mkv', '.m4v'}:
                continue
            record['kind'] = 'video'
            pattern = f'{len(records):03d}_video_%05d.png'
            subprocess.run(['ffmpeg','-v','error','-i',str(path),'-vf',f'fps=1/{interval}:round=up',
                            '-frames:v',str(max_frames),str(out/'references'/pattern)], check=True)
            frames = sorted((out/'references').glob(f'{len(records):03d}_video_*.png'))
            if not frames:
                raise ValueError(f'No video reference frames decoded from {path}')
            record['references'] = [str(p.relative_to(out)) for p in frames]
            record['sampling'] = {'interval_s': interval, 'max_frames': max_frames,
                                  'note': 'ffmpeg fps filter reference samples; not original camera calibration or training timestamps'}
        records.append(record)
    if not records:
        raise ValueError('No supported images or videos found; inspect preserved output directory')
    write_json(out/'inputs.json', {'schema':'realtosim_inputs_v1','media':records})
    write_json(out/'reconstruction_plan.json', {
        'status':'awaiting_agent_reconstruction', 'units':'m','up_axis':'Z',
        'scale_anchor':None,'objects':[],
        'required_per_object':['size_m','pose','material','texture','physics','evidence','uncertainty'],
        'next':'Use skills/scene-asset-router. Inspect references, record route decisions, produce scene.json plus generated assets. This ingestion step does not reconstruct geometry.'})
    return str(out)

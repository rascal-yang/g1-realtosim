"""CPU-only command front end; Isaac and LeRobot run in separate environments."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from .common import read_json, resolve


def interpreter(value, base):
    # Preserve venv symlink paths: resolving the link bypasses pyvenv.cfg.
    return os.path.abspath(os.path.join(str(base), os.path.expanduser(os.path.expandvars(value))))


def main():
    parser=argparse.ArgumentParser(description='Agent-assisted scene assets and aligned G1 training data')
    parser.add_argument('--runtime',type=Path,help='Optional JSON: isaac_python, export_python, env; paths relative to this JSON')
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('doctor',help='Inspect tools and environment without installing or changing them')
    p=sub.add_parser('ingest',help='Inventory scene photos/videos for agent reconstruction')
    p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--interval',type=float,default=2);p.add_argument('--max-frames',type=int,default=60)
    p=sub.add_parser('build-scene',help='Assemble an agent-authored metric scene.json into USD')
    p.add_argument('spec',type=Path);p.add_argument('output',type=Path)
    p=sub.add_parser('package-assets',help='Collect USD and texture dependencies into a relocatable directory')
    p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p=sub.add_parser('render',help='Render existing g1_capture_v1 states without advancing physics')
    p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--profile',choices=['rt2','pt128'],default='rt2');p.add_argument('--start',type=int,default=0);p.add_argument('--count',type=int)
    for name in ['verify','export']:
        p=sub.add_parser(name);p.add_argument('--raw',type=Path,required=True);p.add_argument('--frames',type=Path,required=True)
        if name=='export':
            p.add_argument('--output',type=Path,required=True);p.add_argument('--hand-contract',type=Path)
    p=sub.add_parser('export-lerobot');p.add_argument('--aligned',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--allow-unreviewed',action='store_true')
    a=parser.parse_args();cfg=read_json(a.runtime) if a.runtime else {};base=a.runtime.resolve().parent if a.runtime else Path.cwd()
    env=os.environ.copy();env.update({k:os.path.expandvars(v) for k,v in cfg.get('env',{}).items()})
    env['PYTHONPATH']=str(Path(__file__).resolve().parents[1])+os.pathsep+env.get('PYTHONPATH','')
    if a.command=='doctor':
        result={'python':sys.executable,'python_version':sys.version,'tools':{x:shutil.which(x) for x in ['ffmpeg','ffprobe','blender','git']},
                'modules':{x:importlib.util.find_spec(x) is not None for x in ['numpy','h5py','pxr','isaacsim','isaaclab','lerobot']},
                'configured_interpreters':{k:interpreter(cfg[k],base) for k in ['isaac_python','export_python'] if k in cfg},
                'note':'Presence check only; render smoke test is needed to verify GPU/driver compatibility.'}
    elif a.command=='render':
        command=[interpreter(cfg['isaac_python'],base) if 'isaac_python' in cfg else sys.executable,'-m','g1_realtosim.render',
                 '--episode',str(a.episode.resolve()),'--output',str(a.output.resolve()),'--profile',a.profile,'--start',str(a.start)]
        if a.count is not None:command+=['--count',str(a.count)]
        log=a.output.resolve().with_suffix('.render.log')
        if a.output.exists():raise FileExistsError(a.output)
        log.parent.mkdir(parents=True,exist_ok=True)
        print(f'Rendering {a.profile}; log: {log}',flush=True)
        with log.open('x') as stream:
            result=subprocess.run(command,env=env,stdout=stream,stderr=subprocess.STDOUT)
        if result.returncode:raise RuntimeError(f'Render failed; inspect {log}')
        print(json.dumps(read_json(a.output/'complete.json'),indent=2));return
    elif a.command=='export-lerobot' and 'export_python' in cfg and not os.environ.get('REALTOSIM_EXPORT_CHILD'):
        env['REALTOSIM_EXPORT_CHILD']='1'
        command=[interpreter(cfg['export_python'],base),'-m','g1_realtosim','export-lerobot','--aligned',str(a.aligned.resolve()),'--output',str(a.output.resolve())]
        if a.allow_unreviewed:command+=['--allow-unreviewed']
        subprocess.run(command,env=env,check=True);return
    elif a.command=='ingest':
        from .ingest import ingest
        result=ingest(a.source,a.output,a.interval,a.max_frames)
    elif a.command in ['build-scene','package-assets']:
        from . import assets
        result=assets.build(a.spec,a.output) if a.command=='build-scene' else assets.package(a.source,a.output)
    elif a.command=='verify':
        from .dataset import verify
        result=verify(a.raw,a.frames)
    elif a.command=='export':
        from .dataset import export
        result=export(a.raw,a.frames,a.output,a.hand_contract)
    elif a.command=='export-lerobot':
        from .dataset import lerobot_export
        result=lerobot_export(a.aligned,a.output,a.allow_unreviewed)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()

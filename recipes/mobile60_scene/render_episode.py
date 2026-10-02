#!/usr/bin/env python3
"""Replay one packaged mobile60 episode and record its Isaac Sim camera video.

This uses the scene bundle's recorded motion tokens and advances simulation
physics. It is not a fixed-pose, physics-free re-render.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUNDLE = REPO_ROOT.parent / "exports/g1_revo2_mobile60_scene_20261002"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--runtime", type=Path, required=True,
                        help="Prepared runtime directory for the scene bundle")
    parser.add_argument("--output", type=Path, required=True,
                        help="New output directory outside this source repository")
    parser.add_argument("--episode", type=int, default=0, choices=range(60))
    parser.add_argument("--isaac-python", type=Path, default=Path(sys.executable))
    parser.add_argument("--runtime-config", type=Path,
                        help="Optional g1-realtosim local runtime JSON; applies its env settings")
    parser.add_argument("--ort-path", type=Path,
                        help="Optional directory containing onnxruntime/")
    parser.add_argument("--max-steps", type=int,
                        help="Optional short startup check; never a full visual review")
    args = parser.parse_args()

    bundle = args.bundle.resolve()
    runtime = args.runtime.resolve()
    output = args.output.resolve()
    isaac_python = args.isaac_python.resolve()
    ort_path = args.ort_path.resolve() if args.ort_path else None
    env = os.environ.copy()
    if args.runtime_config:
        runtime_config = json.loads(args.runtime_config.resolve().read_text())
        env.update({key: os.path.expandvars(value)
                    for key, value in runtime_config.get("env", {}).items()})

    launcher = bundle / "launch.py"
    episode_dir = bundle / "payload/episodes" / f"{args.episode:06d}"
    if not launcher.is_file() or not episode_dir.is_dir():
        parser.error(f"not a mobile60 scene bundle or episode missing: {bundle}")
    if not runtime.is_dir():
        parser.error(f"prepared runtime directory does not exist: {runtime}")
    if not isaac_python.is_file():
        parser.error(f"Isaac Python interpreter does not exist: {isaac_python}")
    if output.exists():
        parser.error(f"output must be new; preserving existing run: {output}")
    if output == REPO_ROOT or REPO_ROOT in output.parents:
        parser.error("render outputs must stay outside the g1-realtosim source repository")
    if "robot" in str(runtime) or "robot" in str(output):
        parser.error("bundle runtime and output paths cannot contain the substring 'robot'")
    if args.max_steps is not None and args.max_steps < 1:
        parser.error("--max-steps must be positive")

    command = [
        str(isaac_python), str(launcher), "--runtime", str(runtime),
        "--episode", str(args.episode), "--output", str(output), "--record-video",
    ]
    if ort_path:
        command += ["--ort-path", str(ort_path)]
    if args.max_steps is not None:
        command += ["--max-steps", str(args.max_steps)]

    log_path = output.with_name(output.name + ".render.log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    print(json.dumps({"status": "starting", "episode": args.episode,
                      "output": str(output), "log": str(log_path),
                      "mode": "recorded-token simulation replay; physics advances"}),
          flush=True)
    with log_path.open("x", encoding="utf-8") as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                 env=env)
        last_report = 0.0
        while child.poll() is None:
            time.sleep(10)
            now = time.time()
            if now - last_report >= 30:
                print(json.dumps({"status": "running", "elapsed_s": round(now-started, 1),
                                  "log_bytes": log_path.stat().st_size}), flush=True)
                last_report = now
        return_code = child.returncode

    summary = {
        "episode": args.episode,
        "mode": "recorded-token simulation replay; physics advances",
        "bundle": str(bundle),
        "runtime": str(runtime),
        "runtime_config": str(args.runtime_config.resolve()) if args.runtime_config else None,
        "output": str(output),
        "command": command,
        "return_code": return_code,
        "elapsed_s": round(time.time() - started, 3),
        "shortened_smoke_test": args.max_steps is not None,
        "visual_review": "pending",
        "training_rgb_accepted": False,
        "controller_or_asset_changes": [],
        "matched_prior_lesson": "Preserved the bundled upright-bottle reference, hand mounts, and contact guards; this run changes no controller, physics, or asset parameters.",
        "log": str(log_path),
    }
    if output.is_dir() and (output / "bundle_result.json").is_file():
        summary["bundle_result"] = json.loads((output / "bundle_result.json").read_text())
    if output.is_dir():
        (output / "rerender_provenance.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())

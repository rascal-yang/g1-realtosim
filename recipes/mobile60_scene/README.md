# Packaged mobile60 scene replay

This recipe uses the prepared scene bundle to run one of its 60 recorded-token
episodes in Isaac Sim and save a new head-camera `head.mp4`. It preserves the
bundle and every earlier run. The replay advances physics; it is not an offline
render of saved per-frame rigid-body poses.

Outputs belong outside this repository. Choose a new directory for every run.
Isaac Sim, GPU drivers, the prepared bundle runtime, and the optional ONNX
Runtime dependency must already be available.

```bash
python recipes/mobile60_scene/render_episode.py \
  --runtime /tmp/g1_mobile60_portability_v3 \
  --isaac-python /home/kinetra-4090-01/miniconda3/envs/sonic/bin/python \
  --runtime-config configs/local.runtime.json \
  --ort-path /home/kinetra-4090-01/code/zijian/g1_closed_loop/.deps \
  --episode 0 \
  --output ../experiments/20261002_mobile60_bundle/rerender_004
```

The new run records a sibling `<output>.render.log` plus `bundle_result.json`,
`head.mp4`, and `rerender_provenance.json` in the output directory. A complete
task replay does not itself establish visual acceptance. Review the video at
native frame rate and keep its visual review status separate from grasp/contact
results.

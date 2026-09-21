---
name: sim-visual-realism-review
description: Inspect and correct visual realism after robot simulation experiments and before accepting first-person training videos, especially missing fingers, transparency, temporal ghosts, contact penetration, or mismatched materials.
---

# Simulation visual realism review

For this user's simulation experiments, review realism after every run, including failed runs. Preserve the original capture and create a corrected version or a new run; never silently overwrite failure evidence.

## Inspect actual output

- Decode the entire video. Inspect first-person and external views at initialization, approach, contact, maximum wrist rotation, slip/drop, recovery, and final hold. Also inspect a uniform sample through the whole episode; a final successful frame is insufficient.
- Inspect the final `next_observation` and all final physics substeps too: a last pre-action image can miss a collision occurring during the final control step. Keep that collision in the report.
- Inspect original RGB frames, camera transforms, actual body/joint poses, object poses, and contact data together. Review both hands and all digits, especially the thumb. Save timestamps and image evidence of artifacts.
- Opaque metal, plastic and rubber must remain opaque. Distinguish seeing a finger physically through a transparent bottle from a finger disappearing in free space. Check material bindings including face subsets, opacity/transmission inputs and alpha textures, visibility, face winding/backface culling, near clipping, mesh penetration, and temporal denoising/antialiasing history.
- Check whether RGB, depth and segmentation describe the same pose. Do not hide a missing digit with camera changes, make a real transparent object opaque to hide an artifact, or repair training RGB with generative inpainting.

## Diagnose and correct

Re-render a suspect saved pose without advancing physics. Change one relevant rendering/material variable at a time and compare against a converged path-traced reference when useful. Saved-state visual replay is permitted for diagnosis, but is not a new physical success or a substitute for running a changed controller.

Preserve robot geometry, mounting, mass, friction and collision properties during a visual fix. If physics or geometry must change, disclose it and repeat the physical experiment. Avoid inferring a cause solely from a screenshot: label hypotheses until supported by material inspection or rendering comparison.

After the fix, inspect the moving approach/contact sequence again. A static reference alone does not validate temporal artifacts. Record renderer mode, sample count, denoiser/AA settings, camera calibration, asset hashes, and before/after evidence.

## Temporal quality

- Montages cannot establish absence of flicker. Inspect consecutive frames at native playback rate in stationary, moving, contact and recovery segments; preserve short clips. Separate actual camera/body motion from render noise, exposure pumping, texture shimmer and temporal ghosts. Use fixed-state or identical saved-state A/B rendering when attributing causes.
- In the 20260921 visual recovery runs, 4 path-tracing samples/update, 8 total samples and OptiX denoising removed the missing-thumb defect but produced user-reported flicker. Do not accept that preview profile for training. A 128-sample short replay improved detail; this is a candidate profile, not proof of full-video acceptance. Check actual installed renderer settings rather than assuming latest documentation applies.
- Record `temporal_review_status` separately. Unresolved flicker, or temporal quality not inspected, keeps overall `accepted=false` even if the thumb check passes. After a later defect report, preserve the previous review and revoke the stale visual acceptance in JSON/HDF5/report metadata.
- Prefer recording physics once and rendering matching states with sufficient samples for dataset export. Restore all dynamic objects, articulated bodies and cameras; a replay restoring only robot/bottle/camera is diagnostic only. Preserve frame/action correspondence; do not smooth video independently of state to conceal physical shaking.
- Preserve authored scales when restoring world poses, including scaled scene props. Keep preview RGB and final RGB explicitly distinguished in files and training metadata; a valid physical episode does not make its low-sample preview training-ready. Record the final image-to-control-index mapping and identify the exact RGB source for export.
- A privileged camera's ray score needs a positive check against a known relevant robot/object collider. A floor-only probe or repeated no-hit rays does not establish visibility of the actual hand and object. Preserve invalid scores as original inputs, annotate their limits, and inspect the rendered view without hiding geometry.

- Verify actual per-pose accumulation, not just `totalSpp=128`. In the local IsaacSim build, a pose change invalidated the first32-sample update: four updates differed from converged output, while updates5through8matched. Warm up scene loading and require settled output after sufficient accumulation; preserve the per-frame audit. Do not silently label earlier partial renders as128-complete.

## Acceptance

Write `visual_realism_review.json` and a short report in each experiment folder with reviewed frame indices, findings, corrective changes, remaining approximations, and pass/fail/uncertain. Keep visual quality separate from task success and collision quality. A video with unresolved disappearing opaque fingers or inconsistent image/state alignment must not be labeled training-ready. Do not claim physically calibrated materials from an uncalibrated photograph.

If post-rendering saved states for cleaner video, retain original capture and frame-to-state mapping and label the new video as re-rendered. Replace a training RGB stream only with exact corresponding states and revalidate all alignment metadata.

---
name: g1-grasp-runtime-review
description: Review and run G1 grasp experiments with Revo 2 hands in Isaac Sim, checking bilateral hand installation, continuous obstacle contacts, slow reference execution, and grasp quality. Use when changing hands, approaching a desk, or reviewing a grasp video.
---

# G1 grasp runtime review

Use images, actual robot/hand state, and privileged collision/contact data together. A final successful lift does not imply the approach was collision-free. Keep each experiment and its report in a separate folder outside the original project, preserving old assets and runs.

## Before running

- For this user's G1/Revo 2 bottle experiments, read [bottle experience](references/bottle_experience.json) and referenced motion metadata before choosing a first attempt. Record matched lessons, changed parameters and rejected hypotheses in the new experiment. Retrieve by upright/fallen pose, hand mount, dimensions and physics; never treat a failed reclosure as a successful lift.

- Verify each hand's URDF separately: wrist-to-base rotation and translation, thumb side, palm normal, finger-forward direction, signed closing limits, mimic joints, and initial open state. A physical mount-position check alone cannot detect a hand installed with a 180-degree twist. Check mount quaternion error as well as an independent anatomical mirror check and rendered views of both hands.
- Left/right URDFs may already be mirrored. Do not copy the same mount rotation blindly, and do not mirror meshes as an ad hoc correction. For this local Revo 2 model, the old common mount incorrectly pointed the left thumb downward; correcting the left mount requires a half-turn around G1 wrist longitudinal X while retaining finger-forward X. Confirm with FK and rendering before accepting.
- Check *whole hand* geometry against the desk edge, not just the wrist or palm point. A hand below tabletop must rise outside the front edge before moving forward; reserve space for fingertips and thumb during rotation. Historical success trajectories are hints, especially after a hand-model change.
- Re-check the inactive arm rest pose after correcting a hand mount: the changed fingertip volume can now hit the table even if the wrist reference is unchanged. Park it behind the edge with margin and continue monitoring it. Changing one arm can alter full-body settling and move the other; verify that both open hands have initial clearance.
- Inspect initial images before executing. Clear stale finger overrides when opening. Record any intentional preshape separately from an initialization fault.

## During physics execution

- For walk-to-grasp transitions, compare actual pelvis and foot-center arrival, then check how both move during standing-reference handoff. A fixed nominal leg pose can move already settled feet. Consider preserving the measured stable support pose while transitioning arms; verify it physically before promoting it. Read [the bottle multistart findings](references/bottle_multistart_20260921.md) when revising this local pipeline.
- Before closing, distinguish nominal pad alignment, actual wrist tracking, and actual opposed contacts. A geometrically feasible adjustment can unload an already tilted object and let it fall. Reobserve object motion during open-hand corrections; keep planning feasibility separate from physical stability.
- Verify the execution frequency of every target limiter. A bound per `apply_actions` call may run at physics frequency, not the policy frequency. Set increments from the actual timestep, and retain all physics-substep targets if an interpolator changes commands within a control interval.
- Record both hands and forearms against the table/obstacles at physics frequency. Use actual simulator contacts and poses, not requested waypoints. A point-to-AABB or nearest-vertex distance is only an approximation; label it accordingly.
- Add an event latch that requests an immediate feedback checkpoint on unexpected table contact, and retain the timestamp, affected links, peak force, and image. Do not continue a multi-second waypoint just because the next scheduled GPT checkpoint has not arrived. Pausing is a simulation inspection mechanism, not a claim of a real robot emergency stop.
- Choose force/clearance thresholds for this scene and record them. Distinguish intended toy contact from table impact. After an alert, inspect and retreat/raise outside the obstacle or end the attempt; do not simply dismiss the event and continue pushing. Preserve grip if carrying the object.
- Near the desk/grasp, shorten feedback intervals and re-check both hands, including the inactive arm. Monitor actual wrist angular change, palm speed, object displacement, grip opposition, and table contact—not only successful lift height.
- Keep continuous position and quaternion reference transitions. Hold the existing arm reference during finger-only commands. Do not alter wrist orientation in contact merely because a tracking error exists. If changing orientation is necessary, plan and inspect the swept hand volume.
- Maintain the user-selected GRAIL/SONIC architecture. IK reference changes are not the same as direct joint-actuator overrides. Log any controller change and avoid inserting a lagging output filter into a learned feedback controller without evidence it remains stable.

## Accepting and reporting a run

- Report task success and collision quality separately: stable whole-object clearance, hold duration and opposed grip; table contact count/duration/peak, affected links, and whether alerts interrupted execution.
- Validate frame/action/state alignment, bilateral mount rotation/translation, mimic constraints, and videos by full decode plus image inspection. Inspect approach and lift transitions, not only a successful final frame.
- Preserve failed diagnostics, source snapshots, exact decisions, assets used, and independent reports. Save the successful adapted trajectory as hints, not an unconditional replay controller.
- State whether GPT means this session supplying decisions at paused checkpoints or an external API running continuously. Do not describe a single selected success as a success rate or attribute improvement to one component when several changed.

## Concrete prior failure

`${HOME}/code/zijian/scene_revo2_stationary_hint_20260919/runs/trial_002` succeeded at lifting, but right-hand table contact began at 2.60 s and peaked near 5.10 s at about 16 N. The same rotation had been assigned to both hands. The old hand validator checked mounting position and mimic joints, but not orientation/anatomical direction; the final lift validator ignored hand-table contact. Do not reuse those validators alone as evidence of a clean grasp.

For the corrected implementation, reports, and physics contact monitor, inspect `${HOME}/code/zijian/scene_revo2_guarded_20260920`.

## Visual realism acceptance

After every experiment, apply the local `sim-visual-realism-review` skill before accepting first-person video or marking data training-ready. Inspect thumb visibility through approach and contact; task success alone does not validate rendering.

## Recovery turn clearance

A palm behind the table edge does not establish safe yaw rotation. In run walk_right_recovery_006, a SONIC turn after an elevated withdrawal swept the right pinky into the tabletop. Before turning near furniture, inspect the swept volume of both hands/forearms across the planned yaw and reference transition. Back away into clear floor space before a large turn when a bounded swept-volume check is unavailable. Preserve and report collision stops.

## Learn from recovery and preserve motion continuity

- Run walk_right_recovery_007 at step 1460 removed arm IK when entering SONIC walking: wrist pitch target jumped 1.115 rad and actual velocity reached 13.26 rad/s. Do not copy that transition. Prepare a shared, clearance-checked arm posture and continuous reference before transferring control; inspect actual joint velocities as well as targets. Avoid slowing the entire SONIC feedback output with a generic filter.
- Prefer successful measured posture/relative-pose segments for noncontact transport. Seed IK near that posture, preserve quaternion continuity, and adapt contact targets to the current object. Check both arms' swept volume and joint limits. A reference is a prior, not evidence that blindly replaying commands is safe after a walk or drop.
- Attempt reachable local recovery before a large stance detour. Reposition only when current reach/clearance makes it necessary. Keep the user's 30 simulated second recovery budget global; reserve time for closure, a short proof lift and the required hold. If they cannot fit, stop and label recovery incomplete.
- Save failure evidence, proposed correction and verification status separately. A shallow-contact hypothesis can motivate bounded deeper approach after opening/unloading contact, but slip alone does not justify increasing force/depth/friction. Promote a correction only after a physical stable lift and collision review; preserve unsuccessful attempts and compare matched conditions before claiming improvement.

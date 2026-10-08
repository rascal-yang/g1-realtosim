"""Independent full-episode physical pick/place/release acceptance."""
import argparse
import hashlib
import shutil
import json
from pathlib import Path
import h5py
import numpy as np
from scipy.spatial.transform import Rotation

p = argparse.ArgumentParser()
p.add_argument('run', type=Path)
a = p.parse_args()
r = a.run
c = json.loads((r / 'config.json').read_text())
decisions = [json.loads(p.read_text()) for p in sorted(r.glob('decision_[0-9][0-9][0-9].json'))]
with h5py.File(r / 'episode_raw.h5') as f:
    n = len(f['next_observation/timestamp'])
    if not n:
        raise RuntimeError('No physical steps; preserve initialization failure separately')
    dt = float(f.attrs['dt'])
    state = f['next_observation']
    m = state['metrics']
    root = state['root_position'][:]
    rq = state['root_quaternion_wxyz'][:]
    root_up = Rotation.from_quat(rq[:, [1, 2, 3, 0]]).apply(np.tile([0, 0, 1], (n, 1)))
    tilt = np.arccos(np.clip(root_up[:, 2], -1, 1))
    obj = state['object_position'][:]
    oq = state['object_quaternion_wxyz'][:]
    rotations = Rotation.from_quat(oq[:, [1, 2, 3, 0]])
    obj_tilt = np.arccos(np.clip(rotations.apply(np.tile([0, 0, 1], (n, 1)))[:, 2], -1, 1))
    yaw = rotations.as_euler('xyz')[:, 2]
    yaw_error = np.abs(np.arctan2(np.sin(yaw-c['object_yaw_rad']), np.cos(yaw-c['object_yaw_rad'])))
    xy_error = np.linalg.norm(obj[:, :2]-c['place_target_xy_m'], axis=1)
    strict = m['strict_lift_now'][:].astype(bool)
    held = longest = 0
    for v in strict:
        held = held+1 if v else 0
        longest = max(longest, held)
    tail = max(0, n-round(c['place_settle_s']/dt))
    speed = np.linalg.norm(state['object_linear_velocity'][:], axis=1)
    angspeed = np.linalg.norm(state['object_angular_velocity'][:], axis=1)
    support = (m['object_table_normal_force_N'][:] > .2) & (m['object_hand_normal_force_N'][:] < .1)
    nonhand_present = 'nonhand_object_contact_N' in m
    if nonhand_present:
        support &= m['nonhand_object_contact_N'][:] < .1
    final_stable = bool(n-tail >= round(c['place_settle_s']/dt) and np.all(support[tail:])
                        and max(xy_error[tail:]) < c['place_xy_tolerance_m']
                        and max(obj_tilt[tail:]) < .15 and (c.get('place_target_yaw_rad',c['object_yaw_rad']) is None or max(yaw_error[tail:]) < c['place_yaw_tolerance_rad'])
                        and max(speed[tail:]) < .03 and max(angspeed[tail:]) < .15
                        and max(np.ptp(obj[tail:], axis=0)) < .005)
    names = json.loads(f.attrs['body_names'])
    feet = {}
    for side in ['left', 'right']:
        i = names.index(side+'_ankle_roll_link')
        ps = state['body_position_world'][tail:, i]
        feet[side] = float(np.linalg.norm(np.diff(ps, axis=0)/dt, axis=1).max(initial=0))
    robot_stable = bool(np.min(root[:, 2]) > .45 and max(tilt) < np.deg2rad(20)
                        and max(np.ptp(root[tail:, :2], axis=0)) < .04 and max(feet.values()) < .15)
    rows = [json.loads(l) for l in (r/'table_contacts_physics.jsonl').read_text().splitlines()]
    peak = max((x['total_table_force_N'] for x in rows), default=0)
    collisions = sum(x['total_table_force_N'] > .5 for x in rows)
    phases = [d.get('stage') for d in decisions]
    carry_start = carry_end = None
    for decision in decisions:
        phase = decision.get('stage')
        if phase == 'transfer' and carry_start is None:
            carry_start = json.loads((r/f"observation_{decision['observation_id']:03d}.json").read_text())['step']
        if phase == 'lower' and carry_end is None:
            carry_end = json.loads((r/f"observation_{decision['observation_id']:03d}.json").read_text())['step']
    carry_verified = bool(carry_start is not None and carry_end is not None and carry_end > carry_start
                          and np.all(strict[carry_start:carry_end]))
    ordered = all(k in phases for k in ['hold', 'transfer', 'lower', 'supported', 'release', 'withdraw', 'placed_hold', 'complete'])
    if ordered:
        ordered = [phases.index(k) for k in ['hold', 'transfer', 'lower', 'supported', 'release', 'withdraw', 'placed_hold', 'complete']] == sorted(phases.index(k) for k in ['hold', 'transfer', 'lower', 'supported', 'release', 'withdraw', 'placed_hold', 'complete'])
    release_steps = [d['observation_id'] for d in decisions if d.get('stage') == 'release' and d['action'] == 'open_hand']
    support_before_release = False
    if release_steps:
        obs = json.loads((r/f'observation_{release_steps[0]:03d}.json').read_text())
        support_before_release = obs['grasp_information']['metrics']['object_table_normal_force_N'] > .2
    displacement = float(np.linalg.norm(obj[-1, :2]-obj[0, :2]))
    telemetry = len(f['physics/control_index']) == 4*n and len(f['observation/timestamp']) == n
    success = bool(longest*dt >= 3 and final_stable and robot_stable and collisions == 0 and ordered
                   and support_before_release and displacement > .06 and nonhand_present and telemetry and carry_verified)
    checks = {'lift_at_least_3s':longest*dt >= 3, 'final_released_placement_stable':final_stable,
              'robot_stable':robot_stable, 'no_monitored_obstacle_collision':collisions == 0,
              'all_stages_in_order':ordered, 'table_support_before_release':support_before_release,
              'object_moved_over_6cm':displacement > .06, 'nonhand_contacts_monitored':nonhand_present,
              'complete_telemetry':telemetry, 'every_carry_state_has_grasp_clearance_and_no_other_support':carry_verified}
    report = {'schema':'g1_basket_pick_place_audit_v1', 'task':'Fixed-standing G1 right-hand basket pick, transfer, place, release and withdraw',
              'task_success':success, 'checks':checks, 'failure_reason':None if success else {'failed_checks':[k for k,v in checks.items() if not v], 'last_decision':decisions[-1] if decisions else None},
              'control_steps':n, 'physics_substeps':len(f['physics/control_index']), 'simulation_s':n*dt,
              'strict_lift_max_s':longest*dt, 'carry_control_interval':[carry_start,carry_end], 'final_object_xy_error_m':float(xy_error[-1]),
              'object_xy_displacement_m':displacement, 'final_object_tilt_deg':float(np.rad2deg(obj_tilt[-1])),
              'final_yaw_error_rad':float(yaw_error[-1]), 'target_yaw_constrained':c.get('place_target_yaw_rad',c['object_yaw_rad']) is not None, 'final_object_speed_m_s':float(speed[-1]),
              'final_hand_force_N':float(m['object_hand_normal_force_N'][-1]), 'final_table_force_N':float(m['object_table_normal_force_N'][-1]),
              'final3s_object_span_m':np.ptp(obj[tail:],axis=0).tolist(), 'pelvis_min_m':float(root[:,2].min()),
              'pelvis_tilt_max_deg':float(np.rad2deg(tilt).max()), 'final_feet_speed_max_m_s':feet,
              'monitored_obstacle_contact_peak_N':peak, 'monitored_obstacle_contact_samples_over_0p5N':collisions,
              'contact_force_calibrated':False, 'training_eligible':False, 'video_source':None,
              'coverage_limit':'hands/forearms against table/room at200Hz; object against torso/left-hand/room; not full-body collision certification'}
audit_source=Path(__file__);digest=hashlib.sha256(audit_source.read_bytes()).hexdigest();(r/'scripts_snapshot').mkdir(exist_ok=True);shutil.copy2(audit_source,r/'scripts_snapshot'/f'audit_executed_{digest[:12]}.py');report['auditor_sha256']=digest
(r/'audit.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))

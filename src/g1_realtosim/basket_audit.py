"""Independent whole-object and balance acceptance from physical next states."""
import argparse,json,time
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();r=a.run
with h5py.File(r/'episode_raw.h5') as f:
 dt=float(f.attrs['dt']);n=len(f['observation/timestamp']);root=f['next_observation/root_position'][:];q=f['next_observation/root_quaternion_wxyz'][:];up=Rotation.from_quat(q[:,[1,2,3,0]]).apply(np.tile([0,0,1],(n,1)));tilt=np.arccos(np.clip(up[:,2],-1,1));metric=f['next_observation/metrics'];strict=metric['strict_lift_now'][:].astype(bool)&(tilt<np.deg2rad(20));held=best=0
 for v in strict:held=held+1 if v else 0;best=max(best,held)
 tail=max(0,n-150);root_span=np.ptp(root[tail:,:2],axis=0);body_names=json.loads(f.attrs['body_names']);feet={}
 for side in ('left','right'):
  i=body_names.index(side+'_ankle_roll_link');ps=f['next_observation/body_position_world'][tail:,i];feet[side]={'position_span_m':np.ptp(ps,axis=0).tolist(),'speed_max_m_s':float(np.linalg.norm(np.diff(ps,axis=0)/dt,axis=1).max(initial=0)),'final_position_m':ps[-1].tolist()}
 contact_path=r/'table_contacts_physics.jsonl';rows=[json.loads(l) for l in contact_path.read_text().splitlines()];peak=max((x['total_table_force_N'] for x in rows),default=0);contact_count=sum(x['total_table_force_N']>.5 for x in rows)
 stable=bool(held*dt>=3 and max(root_span)<.04 and all(v['speed_max_m_s']<.15 for v in feet.values()))
 success=stable and contact_count==0
 report={'schema':'basket_physics_audit_v1','contact_force_calibrated':False,'task':'G1 fixed-station right-hand basket lift and hold','control_steps':n,'physics_substeps':len(f['physics/control_index']),'simulation_s':n*dt,'strict_lift_final_s':held*dt,'strict_lift_max_s':best*dt,'whole_basket_final_clearance_m':float(metric['whole_object_clearance_m'][-1]),'whole_basket_peak_clearance_m':float(metric['whole_object_clearance_m'][:].max()),'final_hand_force_N':float(metric['object_hand_normal_force_N'][-1]),'final_basket_table_force_N':float(metric['object_table_normal_force_N'][-1]),'final_thumb_contact_N':float(metric['thumb_contact_N'][-1]),'final_index_middle_contact_N':float(metric['index_middle_contact_N'][-1]),'pelvis_min_m':float(root[:,2].min()),'pelvis_tilt_max_deg':float(np.rad2deg(tilt).max()),'final3s_root_xy_span_m':root_span.tolist(),'feet_final3s':feet,'monitored_obstacle_contact_peak_N':peak,'monitored_obstacle_contact_samples_over_0p5N':contact_count,'physical_hold_success':stable,'collision_quality_passed':contact_count==0,'task_success':success,'training_eligible':False,'clearance_method':'all basket source collider local primitive bounding vertices transformed by actual object pose; conservative for capsules','coverage_limit':'hands and forearms against table/room monitored at200Hz; not full body collision certification','source_video':None,'video_to_seed_success_rate':None,'video_to_seed_denominator':0,'manual_user_annotation_minutes':0,'task_design_source':'manual design using existing basket simulation geometry; no video reconstruction/VLM call'}
 report['nonhand_support_monitored']='nonhand_object_contact_N' in metric
 report['final_nonhand_object_contact_N']=float(metric['nonhand_object_contact_N'][-1]) if 'nonhand_object_contact_N' in metric else None
 # Real commands and aligned final states; no q(t+1) substitution for actions.
 report['final_next_state_present']=len(f['next_observation/timestamp'])==n
 report['all_final_physics_substeps_present']=len(f['physics/control_index'])==4*n
 report['hand_gain_telemetry_present']=all(f'brainco/{s}/stiffness' in f and len(f[f'brainco/{s}/stiffness'])==n for s in ('left','right'))
 if not success:
  decisions=sorted(r.glob('decision_[0-9][0-9][0-9].json'));report['failure_reason']=json.loads(decisions[-1].read_text()).get('reason') if decisions else 'no_decisions'
 else:report['failure_reason']=None
(r/'audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

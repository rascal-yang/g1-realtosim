from pathlib import Path
import os
import json, cv2, numpy as np
from scipy.optimize import least_squares
B=Path(os.environ['REALTOSIM_CASE_DIR']).resolve()
# Manually identified tabletop corners and visible foot centers, in original pixels.
# Corner order: back-left, back-right, front-right, front-left, then feet in same order.
obs={
'00':{0:[478,357],1:[1051,344],2:[1195,437],3:[497,468],5:[1005,674],6:[1105,803],7:[542,879]},
'05':{0:[821,251],1:[1498,476],2:[1137,668],3:[522,340],4:[847,641],5:[1335,955],6:[1104,1173],7:[611,760]},
'06':{0:[199,554],2:[795,388],3:[678,577],4:[330,1025],5:[551,717],6:[785,735],7:[681,1044]},
}
def points(w,d):
 p=np.array([[-w/2,d/2,.75],[w/2,d/2,.75],[w/2,-d/2,.75],[-w/2,-d/2,.75],[-w/2+.035,d/2-.035,.015],[w/2-.035,d/2-.035,.015],[w/2-.035,-d/2+.035,.015],[-w/2+.035,-d/2+.035,.015]])
 return p
initial=[1.2,.6]
for k,o in obs.items():
 K=np.array([[1300,0,853],[0,1300,639.5],[0,0,1.]])
 ids=list(o); ok,r,t=cv2.solvePnP(points(1.2,.6)[ids],np.array(list(o.values()),float),K,None)
 initial.extend([*r.ravel(),*t.ravel(),1300])
def res(a):
 out=[]
 for i,(k,o) in enumerate(obs.items()):
  r=a[2+i*7:5+i*7];t=a[5+i*7:8+i*7];f=a[8+i*7];K=np.array([[f,0,853],[0,f,639.5],[0,0,1.]])
  uv,_=cv2.projectPoints(points(*a[:2])[list(o)],r,t,K,None)
  out.extend((uv.reshape(-1,2)-np.array(list(o.values()))).ravel())
  out.append((f-1300)/500)
 out.extend([(a[0]-1.2)*20,(a[1]-.6)*20])
 return out
lo=[.95,.45]+[-np.inf]*21;hi=[1.5,.8]+[np.inf]*21
for i in range(3):lo[8+i*7]=600;hi[8+i*7]=2600
fit=least_squares(res,initial,bounds=(lo,hi),loss='soft_l1',f_scale=5,max_nfev=2000)
a=fit.x;result={'table_dimensions_m':[a[0],a[1],.75],'scale_basis':'table height assumed 0.75m; no measured absolute scale','observations':obs,'cameras':{}}
for i,(k,o) in enumerate(obs.items()):
 r=a[2+i*7:5+i*7];t=a[5+i*7:8+i*7];f=a[8+i*7];R=cv2.Rodrigues(r)[0];c2w=np.eye(4);c2w[:3,:3]=R.T@np.diag([1,-1,-1]);c2w[:3,3]=-R.T@t
 K=np.array([[f,0,853],[0,f,639.5],[0,0,1.]])
 uv=cv2.projectPoints(points(*a[:2])[list(o)],r,t,K,None)[0].reshape(-1,2)
 err=np.linalg.norm(uv-np.array(list(o.values())),axis=1)
 result['cameras'][k]={'camera_to_world_blender':c2w.tolist(),'intrinsics':K.tolist(),'resolution':[1706,1279],'landmark_rmse_px':float(np.sqrt(np.mean(err**2))),'landmark_errors_px':err.tolist(),'calibration_status':'joint fitted from manually marked desk landmarks; not independently calibrated'}
 im=cv2.imread(str(B/f'inputs/photo_{k}.jpg'))
 for j,(p,q) in enumerate(zip(np.array(list(o.values())),uv)):
  cv2.circle(im,tuple(p.astype(int)),9,(0,255,0),2);cv2.drawMarker(im,tuple(q.astype(int)),(0,0,255),cv2.MARKER_CROSS,20,2)
 cv2.imwrite(str(B/f'reports/layout_fit_{k}.jpg'),im)
(B/'assets/layout_fit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

"""Contract tests protect against silent action/image misalignment."""
import json
from pathlib import Path
import tempfile
import unittest
import h5py
import numpy as np
from g1_realtosim.dataset import verify, export, BODY_FIELDS
from g1_realtosim.cli import interpreter


class AlignmentTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.raw=self.root/'raw.h5';self.frames=self.root/'frames.h5'
        with h5py.File(self.raw,'w') as f:
            f.attrs.update(dt=.02,physics_dt=.01,joint_names=json.dumps(['joint']),task='fixture',camera_calibration='{}')
            f['observation/timestamp']=np.arange(5)*.02
            f['observation/joint_position']=np.arange(5,dtype='float32')[:,None]
            f['observation/joint_velocity']=np.ones((5,1))
            f['next_observation/joint_position']=np.arange(1,6,dtype='float32')[:,None]
            f['observation/root_quaternion_wxyz']=np.tile([1,0,0,0],(5,1))
            f['observation/root_angular_velocity_world']=np.zeros((5,3))
            f['physics/control_index']=np.repeat(np.arange(5),2)
            f['physics/substep_index']=np.tile([0,1],5)
            for k,key in enumerate(BODY_FIELDS):
                f['action/'+key]=np.arange(5)[:,None]+100*k
                f['physics/'+key]=np.arange(10)[:,None]+100*k
        with h5py.File(self.frames,'w') as f:
            f['control_index']=[0,2,4];f['timestamp']=[0,.04,.08]
            f['rgb']=np.zeros((3,480,640,3),dtype='u1')
            f['world_T_camera_cv']=np.tile(np.eye(4),(3,1,1))

    def tearDown(self):self.tmp.cleanup()

    def test_partial_chunk_and_actual_substeps(self):
        output=self.root/'export';export(self.raw,self.frames,output)
        with h5py.File(output/'episode.h5') as f:
            np.testing.assert_array_equal(f['action/valid'][:],[[True,True],[True,True],[True,False]])
            # First command includes both substeps; neither next state nor downsampled action substitutes it.
            np.testing.assert_array_equal(f['action/commands'][0,0],[0,100,200,300,400,1,101,201,301,401])
            np.testing.assert_array_equal(f['action/commands'][2,1],np.zeros(10))
        self.assertFalse(json.loads((output/'provenance.json').read_text())['training_eligible'])

    def test_timestamp_mismatch_rejected(self):
        with h5py.File(self.frames,'r+') as f:f['timestamp'][1]=.06
        with self.assertRaisesRegex(ValueError,'timestamps'):verify(self.raw,self.frames)

    def test_missing_physics_substep_rejected(self):
        with h5py.File(self.raw,'r+') as f:f['physics/control_index'][3]=4
        with self.assertRaisesRegex(ValueError,'substeps'):export(self.raw,self.frames,self.root/'broken')

    def test_venv_interpreter_keeps_symlink(self):
        link=self.root/'venv_python';link.symlink_to('/usr/bin/python3')
        self.assertEqual(interpreter(str(link),self.root),str(link))


if __name__=='__main__':unittest.main()

import copy
import json
from pathlib import Path
import unittest
from g1_realtosim.animation_pipeline import validate_spec

class AnimationContract(unittest.TestCase):
    def setUp(self):
        self.s=json.loads((Path(__file__).resolve().parents[1]/'configs/basket_animation.example.json').read_text())
    def test_example_and_timing_change(self):
        validate_spec(self.s)
        self.s['duration_s']*=1.2
        for e in self.s['events']:e['time_s']*=1.2
        validate_spec(self.s)
    def test_unsupported_contact_phases(self):
        self.s['events'][4]['event']='pour'
        with self.assertRaises(ValueError):validate_spec(self.s)
    def test_nonfinite_geometry(self):
        self.s['basket_goal_xy_m'][0]=float('nan')
        with self.assertRaises(ValueError):validate_spec(self.s)
    def test_cannot_silently_switch_scene(self):
        self.s['tabletop_aabb_m'][1][2]=.8
        with self.assertRaises(ValueError):validate_spec(self.s)
    def test_frame_alignment(self):
        self.s['events'][3]['time_s']+=.001
        with self.assertRaises(ValueError):validate_spec(self.s)
    def test_nonpositive_phase(self):
        self.s['events'][3]['time_s']=self.s['events'][2]['time_s']
        with self.assertRaises(ValueError):validate_spec(self.s)

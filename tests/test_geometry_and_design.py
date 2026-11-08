"""Fast offline tests: no Gym, GPU, window or simulator execution required."""
import ast
import itertools
from pathlib import Path
import random
import unittest
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]

def selected(path,names,scope):
    tree=ast.parse(path.read_text())
    nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),scope)
    return scope

class GeometryAndDesign(unittest.TestCase):
    def setUp(self):
        self.ns=selected(ROOT/'src'/'reference_campaign.py',
            {'wrap_angle','resample_closed','make_tracks','nearest_state','pure_pursuit','stanley'},
            dict(np=np,math=__import__('math'),cKDTree=cKDTree,create_map=lambda *a:None,
                WHEELBASE=.3302,DEFAULT_PARAMS={'s_min':-.4189,'s_max':.4189}))

    def test_projection_and_corrective_steering(self):
        x=np.linspace(-10,10,1200)
        track=dict(x=x,y=np.zeros(1200),yaw=np.zeros(1200),curvature=np.zeros(1200),length=20.,tree=cKDTree(np.c_[x,np.zeros(1200)]))
        self.assertGreater(self.ns['nearest_state'](track,0,.1)[1],0)
        self.assertLess(self.ns['pure_pursuit'](track,0,.1,0,1.,.45),0)
        self.assertLess(self.ns['stanley'](track,0,.1,0,1.,1.8),0)
        self.assertLess(self.ns['stanley'](track,0,0,.1,1.,1.8),0)

    def test_preview_indices(self):
        tracks=self.ns['make_tracks']()
        self.assertEqual([round(.45/(tracks[n]['length']/1200)) for n in ('oval','hairpin','chicane')],[26,23,24])

    def test_half_fraction_and_pairing(self):
        factors=('speed_code','delay_code','noise_code','bias_code','mismatch_code')
        ns=selected(ROOT/'src'/'doe_campaign.py',{'manifest'},dict(itertools=itertools,random=random,pd=pd,
            FACTORS=factors,CONTROLLERS=('Pure Pursuit','Stanley'),TRACKS=('oval','hairpin','chicane'),SEEDS=(3101,3102,3103)))
        m=ns['manifest']()
        self.assertEqual(len(m),360)
        self.assertEqual(m.groupby('pair_id').size().tolist(),[2]*180)
        mat=m[m.phase=='B_fraction'].drop_duplicates('design_point')[list(factors)].to_numpy()
        self.assertTrue((mat.prod(axis=1)==1).all())
        for order in (1,2):
            for terms in itertools.combinations(range(5),order):self.assertEqual(np.prod(mat[:,terms],axis=1).sum(),0)
        archived=pd.read_csv(ROOT/'data'/'published'/'scenario_manifest.csv')
        pd.testing.assert_frame_equal(m,archived,check_dtype=False)

    def test_angle_wrapping(self):
        wrap=self.ns['wrap_angle']
        self.assertAlmostEqual(wrap(3*np.pi),-np.pi)
        self.assertAlmostEqual(wrap(-3*np.pi),-np.pi)

if __name__=='__main__':unittest.main()

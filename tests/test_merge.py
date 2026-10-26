import unittest
from unittest.mock import patch
import numpy as np
from navgen.merge import merge_nav
from navgen.merge_audit import audit_merge
from navgen.hiding import compute_hiding_spots
from navgen.__main__ import validate_nav
from test_generation import collision
import test_generation as fixtures


def mesh(rectangles):
    helper=fixtures.GenerationTests();g=helper.empty_graph()
    for box in rectangles:helper.area(g,box,.03125)
    nav=g.nav(1)
    for i in range(len(rectangles)-1):
        nav['areas'][i]['links'][1]=[i+2];nav['areas'][i+1]['links'][3]=[i+1]
    return nav


class MergeTests(unittest.TestCase):
    def pair(self):return mesh([[0,0,25,25],[25,0,50,25]])

    def test_union_directions_and_hiding_are_recomputed(self):
        nav=mesh([[-25,0,0,25],[0,0,25,25],[25,0,50,25],[50,0,75,25]])
        nav['areas'][0]['attr']=nav['areas'][3]['attr']=8192
        nav['areas'][3]['links'][3]=[] # Preserve an incoming-only frontier.
        world=collision([([-100,-100,-20],[200,200,0])])
        result,stats=merge_nav(nav,scene=world.scene)
        self.assertEqual(len(result['areas']),3);self.assertEqual(len(nav['areas']),4)
        audit=audit_merge(nav,result,stats['mapping'],world.scene)
        self.assertEqual(audit['links']['changed_unverified'],0)
        self.assertEqual(result['areas'][1]['links'][3],[1]);self.assertEqual(result['areas'][1]['links'][1],[4])
        result['areas'][0]['hiding']=[(999,(0.,0.,0.),8)]
        compute_hiding_spots(result,world);validate_nav(result)
        self.assertNotIn(999,[s[0] for a in result['areas'] for s in a['hiding']])

    def test_gap_partial_edge_step_and_one_way_are_not_merged(self):
        for box,z in [([25.01,0,50,25],0),([25,0,50,20],0),([25,0,50,25],.6)]:
            nav=mesh([[0,0,25,25],box]);a=nav['areas'][1]
            a['nw']=(*a['nw'][:2],z);a['se']=(*a['se'][:2],z);a['ne_z']=a['sw_z']=z
            self.assertEqual(len(merge_nav(nav)[0]['areas']),2)
        nav=self.pair();nav['areas'][1]['links'][3]=[]
        self.assertEqual(len(merge_nav(nav)[0]['areas']),2)
        nav=self.pair();nav['areas'][1]['links']=[[1],[],[],[]]
        self.assertEqual(len(merge_nav(nav)[0]['areas']),2)

    def test_semantic_boundaries_are_protected(self):
        for field,value in [('attr',1),('place',2),('light',(0.,0.,0.,0.)),('blocked',True),('damaging',True),('ladders',[[1],[]])]:
            nav=self.pair();nav['areas'][1][field]=value
            self.assertEqual(len(merge_nav(nav)[0]['areas']),2,field)
        for flag in (8192,256,0x80000000):
            nav=self.pair()
            for a in nav['areas']:a['attr']=flag
            self.assertEqual(len(merge_nav(nav)[0]['areas']),2)
        self.assertEqual(len(merge_nav(self.pair(),locked_ids=[2])[0]['areas']),2)
        for water in (lambda p:p[0]>25,lambda p:None,lambda p:p[0]==25):
            self.assertEqual(len(merge_nav(self.pair(),water=water)[0]['areas']),2)

    def test_chained_fit_checks_every_original_corner(self):
        nav=mesh([[0,0,25,25],[25,0,50,25],[50,0,75,25]])
        for a,left,right in zip(nav['areas'],[0,.5,0],[.5,0,-.5]):
            a['nw']=(*a['nw'][:2],left);a['sw_z']=left;a['se']=(*a['se'][:2],right);a['ne_z']=right
        result,stats=merge_nav(nav)
        self.assertEqual(len(result['areas']),2);self.assertGreater(stats['rejected']['original_corner_error'],0)
        self.assertLessEqual(audit_merge(nav,result,stats['mapping'])['max_original_corner_height_error'],.5)

    def test_exact_slope_and_hole_are_preserved(self):
        nav=self.pair()
        for a in nav['areas']:
            a['nw']=(*a['nw'][:2],a['nw'][0]*.5);a['sw_z']=a['nw'][2]
            a['se']=(*a['se'][:2],a['se'][0]*.5);a['ne_z']=a['se'][2]
        result,stats=merge_nav(nav);self.assertEqual(len(result['areas']),1)
        self.assertEqual(audit_merge(nav,result,stats['mapping'])['max_original_corner_height_error'],0)
        nav=mesh([[0,0,100,25],[0,25,25,75],[75,25,100,75],[0,75,100,100]])
        result,stats=merge_nav(nav);self.assertEqual(len(result['areas']),4)
        self.assertTrue(audit_merge(nav,result,stats['mapping'])['exact_xy_union'])

    def test_failed_frontier_witness_rejects_contraction(self):
        nav=mesh([[0,0,25,25],[25,0,50,25],[50,0,75,25]]);nav['areas'][2]['attr']=8192
        def witness(scene,a,b,d,*args):return (4 if a[2]-a[0]>25 else 1,None,None)
        with patch('navgen.merge.portal_probe',side_effect=witness):
            result,stats=merge_nav(nav,scene=object())
        self.assertEqual(len(result['areas']),3);self.assertGreater(stats['rejected']['frontier_probe'],0)

    def test_native_height_threshold_is_preserved(self):
        nav=mesh([[0,0,25,25],[25,0,50,25],[0,-25,50,0]])
        nav['areas'][1]['links'][1]=[];nav['areas'][1]['links'][0]=[3]
        nav['areas'][2]['links']=[[],[],[],[]];nav['areas'][2]['attr']=8192
        for a in nav['areas'][:2]:
            a['nw']=(*a['nw'][:2],a['nw'][0]*.2);a['sw_z']=a['nw'][2]
            a['se']=(*a['se'][:2],a['se'][0]*.2);a['ne_z']=a['se'][2]
        a=nav['areas'][2];a['nw']=(*a['nw'][:2],24.);a['se']=(*a['se'][:2],24.);a['ne_z']=a['sw_z']=24.
        result,stats=merge_nav(nav)
        self.assertEqual(len(result['areas']),3);self.assertGreater(stats['rejected']['native_height_class'],0)

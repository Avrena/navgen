import unittest
from unittest.mock import patch

import numpy as np

from navgen.areas import CROUCH,OBSTACLE_TOP
from navgen.phy_audit import compare_bounds
from navgen.sampling import allocate
import test_generation as fixtures


class M3Tests(unittest.TestCase):
    def graph(self):
        helper=fixtures.GenerationTests();g=helper.empty_graph();g.arrays=allocate(5)
        g.arrays[0][:]=[[x,0,0] for x in range(0,125,25)]
        for i in range(4):g.arrays[2][i,1]=i+1;g.arrays[2][i+1,3]=i
        a=g.insert(np.array([0,0,50,25,0,0,0,0.]),0,cells=np.array([0,1]),nodes=np.full(4,-1))
        b=g.insert(np.array([50,0,100,25,0,0,0,0.]),0,cells=np.array([2,3]),nodes=np.full(4,-1))
        c=g.insert(np.array([100,0,125,25,0,0,0,0.]),CROUCH,cells=np.array([4]),nodes=np.full(4,-1))
        g.connect(a.id,b.id,1);g.connect(b.id,a.id,3);g.connect(b.id,c.id,1);g.connect(c.id,b.id,3)
        return g,a,b,c

    def test_reciprocal_witness_requires_reverse_and_owner(self):
        g,a,b,_=self.graph()
        self.assertEqual(g.reciprocal_node_pair(a,b,1),(1,2))
        self.assertEqual(g.reciprocal_node_pair(b,a,3),(2,1))
        g.arrays[2][2,3]=-1;self.assertIsNone(g.reciprocal_node_pair(a,b,1))
        g.arrays[2][2,3]=1;g.arrays[-1][2]=-1
        self.assertIsNone(g.reciprocal_node_pair(a,b,1))

    def test_witness_must_cross_shared_portal(self):
        g,a,b,_=self.graph()
        g.arrays[0][2,0]=30 # Owned by b but does not reach its edge at x=50.
        self.assertIsNone(g.reciprocal_node_pair(a,b,1))
        g.arrays[0][2,0]=50;g.arrays[0][1:3,1]=25 # Only the far corner, outside owned edge cells.
        self.assertIsNone(g.reciprocal_node_pair(a,b,1))

    def test_witness_survives_split_and_merge(self):
        g,a,b,c=self.graph();left,right=g.split(a.id,0,25)
        self.assertEqual(g.reciprocal_node_pair(left,right,1),(0,1))
        self.assertEqual(g.reciprocal_node_pair(right,b,1),(1,2))
        g.merge();merged=next(v for v in g.areas.values() if v.attr==0)
        self.assertEqual(len(g.areas),2)
        self.assertEqual(g.reciprocal_node_pair(merged,c,1),(3,4))

    def test_unverified_ordinary_kept_with_witness(self):
        g,a,b,_=self.graph()
        with patch('navgen.areas.probe_portal',return_value=('unverified',None)):g.fix_connections()
        self.assertIn(b.id,a.links[1]);counts=g.link_audit['by_kind']['ordinary_step']
        self.assertEqual(counts['node_backed'],4);self.assertEqual(counts['removed_no_node_evidence'],0)
        self.assertTrue(all(e['node_pair'] is not None for e in g.link_audit['ordinary_node_evidence']))

    def test_unbacked_ordinary_removed_special_links_retained(self):
        helper=fixtures.GenerationTests();g=helper.empty_graph()
        areas=[helper.area(g,[25*i,0,25*i+25,25],attr=OBSTACLE_TOP if i==5 else 0) for i in range(6)]
        for i,kind in enumerate(('ordinary_step','drop','stitched_jump','corner_helper','fence')):
            g.connect(areas[i].id,areas[i+1].id,1,kind)
        with patch('navgen.areas.probe_portal',return_value=('unverified',None)):g.fix_connections()
        self.assertNotIn(areas[1].id,areas[0].links[1])
        for i in range(1,5):self.assertIn(areas[i+1].id,areas[i].links[1])
        self.assertEqual(g.link_audit['by_kind']['ordinary_step']['removed_no_node_evidence'],1)
        self.assertEqual(g.counts['invalid_links_removed'],1)

    def test_phy_bounds_order_and_threshold(self):
        a=[np.array([[0,0,0],[4,5,6.]]),np.array([[8,1,2],[10,4,7.]])]
        self.assertEqual(compare_bounds(a,a[::-1])['max_nearest_piece_bound_delta'],0)
        self.assertFalse(compare_bounds(a,[p+1 for p in a])['bounds_differ_over_1_unit'])
        self.assertTrue(compare_bounds(a,[p+1.01 for p in a])['bounds_differ_over_1_unit'])

    def test_piece_bounds_detect_partition_difference(self):
        a=[np.array([[0,0,0],[5,1,1.]]),np.array([[5,0,0],[10,1,1.]])]
        b=[np.array([[0,0,0],[2,1,1.]]),np.array([[2,0,0],[10,1,1.]])]
        result=compare_bounds(a,b)
        self.assertEqual(result['global_bound_delta'],0)
        self.assertEqual(result['max_nearest_piece_bound_delta'],3)
        self.assertTrue(result['bounds_differ_over_1_unit'])


if __name__=='__main__':unittest.main()

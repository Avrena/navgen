import unittest
from unittest.mock import patch

import numpy as np

from navgen.areas import NO_MERGE,OBSTACLE_TOP,Spatial,corner_points
from navgen.sampling import allocate
import test_generation as fixtures


class M6Tests(unittest.TestCase):
    def graph(self,count=4):
        g=fixtures.GenerationTests().empty_graph();g.arrays=allocate(count)
        return g

    def area(self,g,box,z=0,attr=0,nodes=None):
        return g.insert(np.r_[box,np.full(4,float(z))],attr,nodes=np.full(4,-1) if nodes is None else nodes)

    def test_sdk_raise_geometry_all_directions(self):
        expected=[([0,8,25,18],[40,45,45,40]),([0,0,10,25],[45,40,40,45]),
                  ([0,7,25,17],[45,40,40,45]),([8,0,18,25],[45,40,40,45])]
        for d,(xy,z) in enumerate(expected):
            with self.subTest(direction=d):
                g=self.graph();a=self.area(g,[0,0,25,25],nodes=np.arange(4))
                g.arrays[0][:]=corner_points(a.box);ids=[(d+3)%4,(d+2)%4]
                g.arrays[9][ids,d]=[40,45];g.arrays[10][ids,d]=[12,15];g.arrays[11][ids,d]=[21,24]
                g.fence_tops()
                np.testing.assert_allclose(a.box,np.r_[xy,z]);self.assertEqual(a.attr,NO_MERGE|OBSTACLE_TOP)
                self.assertTrue((a.nodes==-1).all())

    def test_raise_width_clamps_to_cell(self):
        for distance,expected in [(1,[0,15,25,25]),(24,[0,0,25,10])]:
            g=self.graph();a=self.area(g,[0,0,25,25],nodes=np.arange(4));g.arrays[0][:]=corner_points(a.box)
            g.arrays[9][[3,2],0]=40;g.arrays[10][[3,2],0]=distance;g.fence_tops()
            np.testing.assert_allclose(a.box[:4],expected)

    def test_creation_stops_at_first_obstacle_top(self):
        g=self.graph();self.area(g,[-50,0,0,50],40,OBSTACLE_TOP)
        a=self.area(g,[0,0,50,50],nodes=np.arange(4));b=self.area(g,[50,0,100,50])
        g.connect(a.id,b.id,1)
        with patch.object(g,'create_fence',return_value=True) as create:g.fence_tops()
        create.assert_not_called();self.assertEqual(g.counts['obstacle_areas_created'],0)

    def fence_graph(self,x=25):
        g=self.graph(2);g.arrays[0][:]=[[x,0,0],[x,25,0]];g.arrays[2][0,2]=1
        g.arrays[9][:,1]=[40,45];g.arrays[10][:,1]=10;g.arrays[11][:,1]=20
        a=self.area(g,[0,0,25,25],nodes=np.array([-1,0,1,-1]));b=self.area(g,[50,0,75,25])
        g.connect(a.id,b.id,1);g.connect(b.id,a.id,3)
        return g,a,b

    def test_portal_membership_uses_both_xy_axes(self):
        g,a,b=self.fence_graph(x=0)
        self.assertFalse(g.create_fence(a,b,1,False));self.assertIn(b.id,a.links[1])

    def test_east_fence_uses_sdk_corner_heights_and_links(self):
        g,a,b=self.fence_graph();self.assertTrue(g.create_fence(a,b,1,False))
        top=next(x for x in g.areas.values() if x.attr&OBSTACLE_TOP)
        np.testing.assert_allclose(top.box,[35,0,45,25,40,45,45,40])
        self.assertNotIn(b.id,a.links[1]);self.assertIn(top.id,a.links[1]);self.assertIn(top.id,b.links[3])
        self.assertIn(a.id,top.links[3]);self.assertIn(b.id,top.links[1])

    def test_multinode_landing_uses_contains_not_two_unit_snap(self):
        g=self.graph(6);g.arrays[0][:]=[[25,0,0],[25,25,0],[50,0,40],[50,25,45],[75,0,5],[75,25,5]]
        g.arrays[2][0,2]=1
        for a,b in [(0,2),(1,3),(2,4),(3,5)]:g.arrays[2][a,1]=b
        a=self.area(g,[0,0,25,25],nodes=np.array([-1,0,1,-1]));b=self.area(g,[75,0,100,25])
        g.connect(a.id,b.id,1)
        self.assertTrue(g.create_fence(a,b,1,True))

    def test_contains_rejects_intervening_floor(self):
        g=self.graph();floor=self.area(g,[0,0,100,100]);self.area(g,[0,0,100,100],10)
        spatial=Spatial(g.areas)
        self.assertTrue(spatial.contains(floor,[25,25,5]))
        self.assertFalse(spatial.contains(floor,[25,25,15]))
        self.assertTrue(spatial.contains(floor,[25,25,-18]))
        self.assertFalse(spatial.contains(floor,[25,25,-18.01]))

    def test_overlap_containment_includes_nw_se_heights(self):
        g=self.graph();low=self.area(g,[0,0,25,25],40,OBSTACLE_TOP);high=self.area(g,[0,0,25,25],60,OBSTACLE_TOP)
        g.remove_overlapping_fences()
        self.assertNotIn(low.id,g.areas);self.assertIn(high.id,g.areas)

    def test_drop_lookup_accepts_area_five_units_above_query(self):
        g=self.graph();floor=self.area(g,[0,0,25,25],5);spatial=Spatial(g.areas)
        self.assertIsNone(spatial.below([10,10,0]));self.assertIs(spatial.below([10,10,0],above=5),floor)
        self.assertIsNone(spatial.below([10,10,-.01],above=5))

    def test_split_incoming_side_requires_outgoing_neighbor(self):
        for has_outgoing in (False,True):
            with self.subTest(outgoing=has_outgoing):
                g=self.graph();high=self.area(g,[0,0,25,100],100);low=self.area(g,[25,0,75,100])
                g.connect(high.id,low.id,1,'drop')
                if has_outgoing:
                    neighbor=self.area(g,[0,0,25,100]);g.connect(low.id,neighbor.id,3)
                pieces=g.split(low.id,1,50)
                self.assertEqual(high.links[1],{a.id for a in pieces} if has_outgoing else set())


if __name__=='__main__':unittest.main()

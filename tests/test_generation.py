import unittest

import numpy as np
from numba import types
from numba.typed import Dict

from navgen.areas import AreaGraph, CROUCH, JUMP, OBSTACLE_TOP, build_areas
from navgen.bsp import box_vertices, convex_planes
from navgen.collision import Collision, SceneBuilder
from navgen.compare import cells, graph_arrays, transition_counts
from navgen.sampling import allocate, can_traverse, sample


def quiet(*args, **kwargs):
    pass


def collision(boxes):
    builder=SceneBuilder()
    for lo,hi in boxes:
        points=box_vertices(lo,hi)
        builder.add(convex_planes(points),points.min(0),points.max(0))
    return Collision(builder.pack(),{},[])


class GenerationTests(unittest.TestCase):
    def empty_graph(self):
        # Explicit nodes avoid looking up a Numba spatial index for these graph fixtures.
        index=Dict.empty(key_type=types.UniTuple(types.int64,2),value_type=types.int64)
        return AreaGraph(collision([]),allocate(0),index,[],[],[])

    def area(self,g,bounds,z=0,attr=0):
        return g.insert(np.array([*bounds,z,z,z,z],float),attr,nodes=np.full(4,-1))

    def test_low_room_end_to_end(self):
        world=collision([([0,0,-20],[200,200,0]),([0,0,60],[200,200,80])])
        arrays,index=sample(world,np.array([[100.,100.,1.]]),[[0,0,0],[200,200,80]],quiet)
        graph,_=build_areas(world,arrays,index,quiet)
        nav=graph.nav(1);points,_=cells(nav)
        self.assertEqual(len(points),64)
        self.assertTrue(all(a['attr']&CROUCH for a in nav['areas']))
        self.assertEqual(graph_arrays(nav)[1]['weak_components'],1)

    def test_one_way_drop_and_wall(self):
        world=collision([([-200,-200,-20],[200,200,0]),([-200,-200,0],[0,200,250])])
        high=np.array([-12.5,0,250.03125]);low=np.array([12.5,0,.03125])
        self.assertTrue(can_traverse(world.scene,high,low))
        self.assertFalse(can_traverse(world.scene,low,high))
        wall=collision([([-200,-200,-20],[200,200,0]),([8,-200,0],[12,200,350])])
        self.assertFalse(can_traverse(wall.scene,np.array([0.,0,.03125]),np.array([25.,0,.03125])))

    def test_overhang_reconnects_exposed_area(self):
        g=self.empty_graph();lower=self.area(g,[0,0,100,100]);upper=self.area(g,[0,0,50,100],100)
        g.connect(upper.id,lower.id,1);g.connect(lower.id,upper.id,3)
        g.split_overhangs()
        self.assertNotIn(lower.id,g.areas)
        exposed=next(a for a in g.areas.values() if a.box[0]==50)
        shadow=next(a for a in g.areas.values() if a.id!=upper.id and a.box[0]==0)
        self.assertIn(exposed.id,upper.links[1]);self.assertIn(upper.id,exposed.links[3])
        self.assertNotIn(shadow.id,upper.links[1])

    def test_jump_stitch_preserves_direction(self):
        g=self.empty_graph();a=self.area(g,[0,0,25,25]);j=self.area(g,[25,0,50,25],attr=JUMP);b=self.area(g,[50,0,75,25])
        g.connect(a.id,j.id,1);g.connect(j.id,b.id,1);g.stitch_jumps()
        self.assertNotIn(j.id,g.areas);self.assertIn(b.id,a.links[1]);self.assertFalse(any(b.links))

    def test_fence_overlap_removes_dangling_links(self):
        g=self.empty_graph();a=self.area(g,[0,0,25,25]);large=self.area(g,[25,0,40,50],40,OBSTACLE_TOP)
        small=self.area(g,[25,0,35,25],40,OBSTACLE_TOP);g.connect(a.id,small.id,1)
        g.remove_overlapping_fences()
        self.assertIn(large.id,g.areas);self.assertNotIn(small.id,g.areas);self.assertFalse(any(a.links))

    def test_transition_metric_detects_missing_reverse_link(self):
        # A split changes area numbering/partitioning, but an internal floor remains traversable.
        points=np.array([[12.5,12.5,0],[37.5,12.5,0]])
        owners=np.array([0,1]);refowners=np.array([0,0]);keys=np.array([0,2**32],np.int64)
        offsets=np.array([-1,2**32,1,-2**32],np.int64)
        ours=np.array([(0*4+1)*2+1],np.int64)
        c=transition_counts(points,owners,refowners,keys,offsets,ours,2,np.empty(0,np.int64),1)
        self.assertEqual(tuple(c[:5]),(2,1,0,1,0))
        both=np.array(sorted([3,(1*4+3)*2]),np.int64)
        c=transition_counts(points,owners,refowners,keys,offsets,both,2,np.empty(0,np.int64),1)
        self.assertEqual(tuple(c[:5]),(2,2,0,0,0))


if __name__=='__main__':unittest.main()

import unittest

import numpy as np

from navgen.bsp import box_vertices, convex_planes, rotation
from navgen.collision import SceneBuilder, trace, triangle_planes, blocked_vertical_span, THIN_MIN, THIN_MAX
from navgen.sampling import move_node
from sdk_m7_1_reference import trace as old_trace, move_node as old_move
from test_generation import collision


class ExactSamplerTests(unittest.TestCase):
    def equal_move(self,world,start,end):
        start=np.asarray(start,dtype=float);end=np.asarray(end,dtype=float)
        actual=move_node(world.scene,start,end,.55);expected=old_move(world.scene,start,end,.55)
        for a,b in zip(actual,expected):np.testing.assert_array_equal(a,b)
        return actual

    def test_deferred_reverse_obstacles_touching_and_failed_moves(self):
        # Fence, steps, ceiling rejection, fully blocked scan and landing off
        # the floor. Includes the collision epsilon and first/last scan heights.
        for height in (17.96875,18.,18.03125,35.,70.,199.,200.,201.,350.):
            for ceiling in (None,90.,150.):
                boxes=[([-200,-200,-20],[200,200,0]),([8,-50,0],[16,50,height])]
                if ceiling is not None:boxes.append(([-50,-50,ceiling],[50,50,ceiling+20]))
                world=collision(boxes)
                for y in (0.,49.55,50.45,50.48125):
                    self.equal_move(world,[0,y,.03125],[25,y,.03125])
                    self.equal_move(world,[25,y,.03125],[0,y,.03125])
        world=collision([([-200,-200,-20],[8,200,0]),([8,-50,0],[16,50,40])])
        self.assertFalse(self.equal_move(world,[0,0,.03125],[25,0,.03125])[0])

    def test_deferred_reverse_overlapping_and_random_obstacles(self):
        rng=np.random.default_rng(8026)
        for i in range(30):
            boxes=[([-200,-200,-30],[200,200,0])]
            for j in range(6):
                lo=rng.uniform([-5,-40,0],[30,40,150]);hi=lo+rng.uniform([1,1,1],[10,30,70])
                boxes.append((lo,hi))
            world=collision(boxes)
            for y in rng.uniform(-50,50,20):
                self.equal_move(world,[0,y,.03125],[25,y,.03125])
                self.equal_move(world,[25,y,.03125],[0,y,.03125])

    def test_solid_early_return_preserves_all_fields_and_first_hit(self):
        rng=np.random.default_rng(8080);builder=SceneBuilder()
        for i in range(30):
            p=box_vertices([-12,-10,-8],[12,10,8])@rotation(rng.uniform(-180,180,3)).T+rng.uniform(-25,25,3)
            planes=convex_planes(p)
            builder.add(planes,p.min(0),p.max(0),contents=1 if i%2 else 2,
                        flags=np.full(len(planes),i,np.int64))
        for z in (0.,5.,10.):
            p=np.array([[-50.,-50,z],[50.,-50,z],[-50.,50,z]])
            planes,front=triangle_planes(p);builder.add(planes,p.min(0),p.max(0),kind=2,front=front)
        scene=builder.pack()
        starts=np.r_[rng.uniform(-35,35,(1000,3)),np.zeros((3,3))]
        ends=np.r_[rng.uniform(-35,35,(1000,3)),[[0.,0,0],[0,0,100],[0,0,-100]]]
        for brute in (False,True):
            for start,end in zip(starts,ends):
                actual=trace(scene,start,end,THIN_MIN,THIN_MAX,3,True,brute)
                expected=old_trace(scene,start,end,THIN_MIN,THIN_MAX,3,True,brute)
                self.assertEqual(actual,expected)

    def test_every_skipped_height_has_a_strict_convex_witness(self):
        rng=np.random.default_rng(8181);skipped=0
        for i in range(30):
            p=box_vertices([-5,-30,-20],[5,30,90])@rotation(rng.uniform(-45,45,3)).T
            builder=SceneBuilder();builder.add(convex_planes(p),p.min(0),p.max(0));scene=builder.pack()
            for z in (0.,18.,18.03125,40.):
                start=np.array([-25.,0,z]);end=np.array([25.,0,z])
                r=old_trace(scene,start,end,THIN_MIN,THIN_MAX)
                if r[0]==1 and not r[4]:continue
                span=blocked_vertical_span(scene,start,end,THIN_MIN,THIN_MAX,r[9],200)
                skipped+=span
                for dz in range(1,span+1):
                    up=np.array([0.,0,dz]);hit=old_trace(scene,start+up,end+up,THIN_MIN,THIN_MAX)
                    self.assertTrue(hit[4] or hit[0]<1,(i,z,dz))
        self.assertGreater(skipped,1000)

    def test_height_span_keeps_saved_phy_bounds_authoritative(self):
        # Real PHY bounds need not equal the extrema implied by their planes.
        p=box_vertices([8,-20,0],[16,20,71]);builder=SceneBuilder()
        builder.add(convex_planes(p),p.min(0),np.array([16.,20.,69.8]));scene=builder.pack()
        start=np.array([0.,0.,18.03125]);end=np.array([25.,0.,18.03125])
        hit=old_trace(scene,start,end,THIN_MIN,THIN_MAX)
        span=blocked_vertical_span(scene,start,end,THIN_MIN,THIN_MAX,hit[9],182)
        self.assertLessEqual(span,51)
        for dz in range(1,span+1):
            up=np.array([0.,0.,dz]);r=old_trace(scene,start+up,end+up,THIN_MIN,THIN_MAX)
            self.assertTrue(r[4] or r[0]<1)
        for a,b in zip(move_node(scene,start-[0,0,18],end-[0,0,18]),
                       old_move(scene,start-[0,0,18],end-[0,0,18])):np.testing.assert_array_equal(a,b)

    def test_height_span_requires_an_epsilon_interior_witness(self):
        # Asymmetric shallow exit plane: the physical triangle still intersects
        # Z=70, but Source's EPS-shifted leaving fraction precedes its entry.
        points=np.array([[x,y,z] for x,z in ((-100.,0.),(12.5,70.01),(1000.,0.)) for y in (-20.,20.)])
        builder=SceneBuilder();builder.add(convex_planes(points),points.min(0),points.max(0));scene=builder.pack()
        start=np.array([0.,0.,18.]);end=np.array([25.,0.,18.]);zero=np.zeros(3)
        r=old_trace(scene,start,end,zero,zero);self.assertTrue(r[4])
        at_seventy=old_trace(scene,start+[0,0,52],end+[0,0,52],zero,zero)
        self.assertEqual(at_seventy[0],1.);self.assertFalse(at_seventy[4])
        span=blocked_vertical_span(scene,start,end,zero,zero,r[9],182)
        self.assertLessEqual(span,51)


if __name__=='__main__':unittest.main()

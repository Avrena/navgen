import unittest
import numpy as np

from navgen.bsp import box_vertices,convex_planes,rotation
from navgen.collision import SceneBuilder,triangle_planes,trace,batch_trace,entity_rules


class CollisionTests(unittest.TestCase):
    def box_scene(self):
        builder=SceneBuilder();p=box_vertices([0,0,0],[10,10,10])
        builder.add(convex_planes(p),p.min(0),p.max(0))
        return builder.pack()

    def test_entry_normal_and_solids(self):
        scene=self.box_scene();zero=np.zeros(3)
        r=trace(scene,np.array([-10.,5,5]),np.array([20.,5,5]),zero,zero)
        self.assertAlmostEqual(r[0],(10-.03125)/30);self.assertEqual(r[1:4],(-1.,0.,0.));self.assertFalse(r[4])
        r=trace(scene,np.array([5.,5,5]),np.array([20.,5,5]),zero,zero)
        self.assertTrue(r[4]);self.assertFalse(r[5]);self.assertEqual(r[0],1)
        r=trace(scene,np.array([5.,5,5]),np.array([6.,5,5]),zero,zero)
        self.assertTrue(r[4]);self.assertTrue(r[5]);self.assertEqual(r[0],0)

    def test_hull_and_mask(self):
        scene=self.box_scene();a=np.array([-10.,5,5]);b=np.array([20.,5,5]);mi=np.full(3,-2.);ma=-mi
        self.assertAlmostEqual(trace(scene,a,b,mi,ma)[0],(8-.03125)/30)
        self.assertEqual(trace(scene,a,b,mi,ma,0x20)[0],1)

    def test_one_sided_terrain(self):
        builder=SceneBuilder();points=np.array([[0.,0,0],[100.,0,0],[0.,100,0]])
        planes,front=triangle_planes(points);builder.add(planes,points.min(0),points.max(0),kind=2,front=front)
        scene=builder.pack();mi=np.array([-.45,-.45,0]);ma=np.array([.45,.45,55.])
        high=np.array([20.,20.,100]);low=np.array([20.,20.,-100])
        r=trace(scene,high,low,mi,ma)
        self.assertLess(r[0],1);self.assertTrue(r[7]);self.assertAlmostEqual(r[3],1)
        self.assertEqual(trace(scene,low,high,mi,ma)[0],1)
        self.assertLess(trace(scene,high,low,np.zeros(3),np.zeros(3))[0],1)

    def test_flat_physics_piece(self):
        points=np.array([[0.,0,0],[100.,0,0],[0.,100,0]])
        builder=SceneBuilder();builder.add(convex_planes(points),points.min(0),points.max(0),kind=1)
        zero=np.zeros(3);scene=builder.pack()
        self.assertLess(trace(scene,np.array([20.,20.,20.]),np.array([20.,20.,-20.]),zero,zero)[0],1)

    def test_filters(self):
        self.assertEqual(entity_rules({'classname':'func_door'}),(True,True))
        self.assertEqual(entity_rules({'classname':'func_brush','solidity':'2'}),(True,False))
        self.assertEqual(entity_rules({'classname':'func_brush','solidity':'1'}),(False,False))
        self.assertEqual(entity_rules({'classname':'func_physbox'}),(True,True))
        self.assertEqual(entity_rules({'classname':'func_breakable','health':'50','spawnflags':'1'}),(True,False))

    def test_bvh_against_all_objects(self):
        rng=np.random.default_rng(7241);builder=SceneBuilder()
        for i in range(35):
            p=box_vertices([-4,-7,-3],[4,7,3])@rotation(rng.uniform(-180,180,3)).T+rng.uniform(-100,100,3)
            builder.add(convex_planes(p),p.min(0),p.max(0),kind=i%2)
        for i in range(35):
            p=np.array([[0.,0,0],[15.,0,0],[0.,15,0]])+rng.uniform(-100,100,3)
            planes,front=triangle_planes(p);builder.add(planes,p.min(0),p.max(0),kind=2,front=front)
        scene=builder.pack();starts=rng.uniform(-120,120,(1500,3));ends=rng.uniform(-120,120,(1500,3))
        mins=np.tile([-2.,-2.,0.],(len(starts),1));maxs=np.tile([2.,2.,10.],(len(starts),1))
        a=batch_trace(scene,starts,ends,mins,maxs);b=batch_trace(scene,starts,ends,mins,maxs,brute=True)
        np.testing.assert_allclose(a[:,:9],b[:,:9],atol=1e-8)


if __name__=='__main__':unittest.main()

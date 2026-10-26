import unittest
import numpy as np
from navgen.pathing import NativeGraph,native_portal
from navgen.collision import SceneBuilder
from navgen.geometry import hull_move
from navgen.navio import read_nav,write_nav
from test_generation import collision
import test_generation as fixtures

class NativePathAndLadderTests(unittest.TestCase):
    def test_floor_probe_recovers_a_surface_slightly_below_a_one_sided_triangle(self):
        from navgen.collision import triangle_planes
        from navgen.geometry import centre_floor
        builder=SceneBuilder();points=np.array([[-100.,-100.,0.],[100.,-100.,0.],[0.,100.,0.]])
        planes,front=triangle_planes(points);builder.add(planes,points.min(0),points.max(0),kind=2,front=front)
        good,z,_=centre_floor(builder.pack(),np.array([0.,0.,-1.]),.7)
        self.assertTrue(good);self.assertAlmostEqual(z,0.)


    def test_invalid_discovery_mode_fails_before_opening_inputs(self):
        from navgen.__main__ import generate
        with self.assertRaisesRegex(ValueError,'Seed discovery'):
            generate('nonexistent.bsp',seed_discovery='unknown')


    def test_corner_connection_is_not_discarded(self):
        helper=fixtures.GenerationTests();graph=helper.empty_graph()
        helper.area(graph,[0,0,25,25],0.);helper.area(graph,[25,25,50,50],0.)
        nav=graph.nav(1);nav['areas'][0]['links'][1]=[2]
        model=NativeGraph(nav);p,width=native_portal(model.bounds[0],model.bounds[1],1)
        self.assertEqual(width,0.);np.testing.assert_array_equal(p,[25.,25.,0.])
        result=model.path([12.5,12.5,4],[37.5,37.5,0])
        self.assertTrue(result['complete']);self.assertEqual(result['area_ids'],[1,2])
        self.assertAlmostEqual(result['length'],np.sqrt(1250))


    def test_partial_path_retains_goal_without_claiming_complete(self):
        helper=fixtures.GenerationTests();graph=helper.empty_graph()
        for box in ([0,0,25,25],[25,0,50,25],[100,0,125,25]):helper.area(graph,box,0.)
        nav=graph.nav(1);nav['areas'][0]['links'][1]=[2]
        result=NativeGraph(nav).path([12.5,12.5,4],[112.5,12.5,0])
        self.assertFalse(result['complete']);self.assertEqual(result['area_ids'],[1,2])
        self.assertAlmostEqual(result['length'],100.)


    def test_full_hull_rejects_thin_gap_and_preserves_one_way_drop(self):
        world=collision([([-200,-200,-20],[200,200,0]),([0,-200,0],[10,-12,100]),([0,12,0],[10,200,100])])
        self.assertEqual(hull_move(world.scene,np.array([-40.,0,.05]),np.array([50.,0,.05])),0)
        world=collision([([-200,-200,-20],[200,200,0]),([-200,-200,0],[0,200,250])])
        self.assertGreater(hull_move(world.scene,np.array([-17.,0,250.05]),np.array([17.,0,.05])),0)
        self.assertEqual(hull_move(world.scene,np.array([17.,0,.05]),np.array([-17.,0,250.05])),0)


    def test_bsp_ladder_points_are_world_coordinates(self):
        from navgen.discovery import ladder_specs
        class Map:
            entities=[{'classname':'func_useableladder','origin':'100 200 300','point0':'1 2 3','point1':'1 2 103'}]
        specs,skipped=ladder_specs(Map())
        self.assertFalse(skipped);np.testing.assert_array_equal(specs[0]['bottom'],[1,2,3])
        np.testing.assert_array_equal(specs[0]['top'],[1,2,103])


    def test_ladder_reciprocal_records_and_blocked_climb(self):
        from navgen.ladders import build_ladders
        from navgen.__main__ import validate_nav
        helper=fixtures.GenerationTests();graph=helper.empty_graph()
        helper.area(graph,[-120,-40,-20,40],.05);helper.area(graph,[20,-40,120,40],128.05)
        nav=graph.nav(1)
        spec={'class':'info_ladder','hammerid':'1','bottom':np.array([10.,0.,0.]),'top':np.array([10.,0.,128.]),
              'width':64.,'axis':0,'lo':np.array([0.,-32.,0.]),'hi':np.array([20.,32.,128.])}
        world=collision([([-200,-100,-20],[200,100,0]),([0,-100,0],[200,100,128])])
        result=build_ladders(nav,world,[spec]);self.assertEqual(result['written'],1)
        self.assertEqual(nav['ladders'][0]['areas'][0],2);self.assertEqual(nav['ladders'][0]['areas'][4],1)
        validate_nav(nav);raw=write_nav(nav);self.assertEqual(raw,write_nav(read_nav(raw)))
        nav['areas'][0]['ladders'][0]=[]
        with self.assertRaisesRegex(ValueError,'Nonreciprocal'):validate_nav(nav)
        world=collision([([-200,-100,-20],[200,100,0]),([0,-100,0],[200,100,128]),([-200,-100,65],[0,100,75])])
        self.assertEqual(build_ladders(graph.nav(1),world,[spec])['written'],0)


    def test_invalid_profile_fails_before_loading_map(self):
        from navgen.__main__ import generate
        with self.assertRaisesRegex(ValueError,'Profile must'):generate('not-an-input.bsp',profile='unknown')



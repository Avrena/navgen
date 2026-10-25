import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from navgen.__main__ import resolve_seed_file
from navgen.bsp import BSP,box_vertices,convex_planes
from navgen.collision import Collision,SceneBuilder
from navgen.hiding import area_spots,compute_hiding_spots,cover_flags,spot_position
from navgen.hiding_compare import compare_hiding,match_count
from navgen.navio import read_nav,write_nav
import test_generation as fixtures


def area(i=1,nw=(0,0,0),se=(100,100,0),attr=0):
    return {'id':i,'nw':nw,'se':se,'ne_z':nw[2],'sw_z':se[2],'attr':attr,'links':[[],[],[],[]]}


def scene(boxes):
    b=SceneBuilder()
    for lo,hi,contents,walk in boxes:
        points=box_vertices(lo,hi);b.add(convex_planes(points),points.min(0),points.max(0),contents,walk)
    return b.pack()


class HidingTests(unittest.TestCase):
    def test_corner_order_and_corner_height(self):
        a=area(nw=(0,0,1),se=(100,100,3));a.update(ne_z=2,sw_z=4)
        self.assertEqual([tuple(spot_position(a,c)) for c in range(4)],
                         [(12.5,12.5,1),(87.5,12.5,2),(87.5,87.5,3),(12.5,87.5,4)])

    def test_inset_fallbacks_and_boundary(self):
        for size,want in [((100,10),(12.5,5)),((10,100),(5,12.5)),((10,10),(5,5)),
                          ((20,20),(12.5,12.5)),((12.5,12.5),(12.5,12.5)),((0,0),(0,0))]:
            with self.subTest(size=size):
                self.assertEqual(tuple(spot_position(area(se=(*size,0)),0)[:2]),want)

    def test_reciprocal_extents_corner_threshold_and_attributes(self):
        a=area();n=area(2,nw=(20,-100,0),se=(80,0,0));a['links'][0]=[2];n['links'][2]=[1]
        self.assertEqual(len(area_spots(a,{1:a,2:n})),4)  # Exactly 20 units leaves both corners.
        n['nw']=(19.99,-100,0)
        self.assertEqual(len(area_spots(a,{1:a,2:n})),3)
        n['links'][2]=[]
        self.assertEqual(len(area_spots(a,{1:a,2:n})),4)  # One-way drop does not fill the edge.
        n['links'][2]=[1];n['attr']=2
        self.assertEqual(len(area_spots(a,{1:a,2:n})),4)
        for attr in (2,0x200):
            a['attr']=attr;self.assertEqual(area_spots(a,{1:a,2:n}),[])

    def test_spot_spacing_is_strict_and_three_dimensional(self):
        a=area(se=(55,25,0))
        self.assertEqual(len(area_spots(a,{1:a})),2)  # Distance 30 survives.
        a['se']=(54,25,0)
        self.assertEqual(len(area_spots(a,{1:a})),1)
        a['ne_z']=40
        self.assertEqual(len(area_spots(a,{1:a})),2)

    def test_cover_overhead_mask_and_entity_filter(self):
        position=np.array([[0,0,0]],np.float32)
        self.assertEqual(cover_flags(scene([]),position).tolist(),[8])
        # A solid brush normally ignored for walking still provides cover.
        ceiling=([-10,-10,50],[10,10,60],1,True)
        self.assertEqual(cover_flags(scene([ceiling]),position).tolist(),[1])
        self.assertEqual(cover_flags(scene([(*ceiling[:2],0x10000,False)]),position).tolist(),[8])
        self.assertEqual(cover_flags(scene([(*ceiling[:2],0x20000,False)]),position).tolist(),[1])
        high=([-10,-10,56],[10,10,60],1,False)
        self.assertEqual(cover_flags(scene([high]),position).tolist(),[8])

    def test_cover_requires_eight_rising_radial_hits(self):
        boxes=[]
        for i in range(8):
            x,y=50*math.cos(i*math.pi/8),50*math.sin(i*math.pi/8)
            # At half range, the ray rises to z=53.25, not the starting 35.5.
            boxes.append(([x-1,y-1,52],[x+1,y+1,55],1,False))
        pos=np.array([[0,0,0]],np.float32)
        self.assertEqual(cover_flags(scene(boxes[:7]),pos).tolist(),[8])
        self.assertEqual(cover_flags(scene(boxes),pos).tolist(),[1])

    def test_ids_area_order_analyzed_and_serialization(self):
        fixture=fixtures.GenerationTests();graph=fixture.empty_graph()
        fixture.area(graph,[0,0,100,100]);fixture.area(graph,[200,0,300,100])
        nav=graph.nav(12);world=Collision(scene([]),{},[])
        stats=compute_hiding_spots(nav,world)
        self.assertEqual((stats['total'],stats['exposed'],nav['analyzed']),(8,8,1))
        self.assertEqual([s[0] for a in nav['areas'] for s in a['hiding']],list(range(8)))
        raw=write_nav(nav);self.assertEqual(write_nav(read_nav(raw)),raw)
        compute_hiding_spots(nav,world);self.assertEqual(write_nav(nav),raw)

    def test_comparison_uses_distance_flags_and_geometry_not_ids(self):
        self.assertEqual(match_count([(1,(0,0,0),1)],[(90,(0,0,1),1)]),1)
        self.assertEqual(match_count([(1,(0,0,0),1)],[(90,(0,0,1.01),1)]),0)
        self.assertEqual(match_count([(1,(0,0,0),1)],[(90,(0,0,0),5)]),0)
        a=area();a['hiding']=[(0,(12.5,12.5,0),8)]
        b=dict(a,id=99,hiding=[(200,(12.5,12.5,0),8)])
        result=compare_hiding({'areas':[a],'analyzed':1},{'areas':[b],'analyzed':1})
        self.assertEqual(result['exact_extents']['same_spot_sets'],1)


class SeedPickupTests(unittest.TestCase):
    def test_absent_sidecar_discovery_and_explicit_override(self):
        with tempfile.TemporaryDirectory(prefix='nav seeds ') as temp:
            root=Path(temp);bsp=root/'test map.bsp';messages=[]
            output=lambda s,**kw:messages.append(s)
            self.assertIsNone(resolve_seed_file(bsp,progress=output))
            self.assertIn('no sidecar found',messages[-1])
            sidecar=root/'test map.navseeds.txt';sidecar.write_text('1 2 3\n',encoding='utf-8')
            self.assertEqual(resolve_seed_file(bsp,progress=output),sidecar)
            self.assertIn('automatic sidecar',messages[-1])
            explicit=root/'override.txt';explicit.write_text('4 5 6\n',encoding='utf-8')
            selected=resolve_seed_file(bsp,explicit,output)
            self.assertEqual(selected,explicit);self.assertIn('explicit override',messages[-1])
            fake=BSP.__new__(BSP);fake.entities=[{'classname':'info_player_start','origin':'0 0 0'}]
            np.testing.assert_array_equal(fake.seeds(selected),[[0,0,0],[4,5,6]])
            self.assertEqual(sidecar.read_text(encoding='utf-8'),'1 2 3\n')

    def test_missing_explicit_and_invalid_sidecar_fail(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);bsp=root/'map.bsp';fake=BSP.__new__(BSP);fake.entities=[]
            selected=resolve_seed_file(bsp,root/'missing.txt',fixtures.quiet)
            with self.assertRaises(FileNotFoundError):fake.seeds(selected)
            sidecar=bsp.with_suffix('.navseeds.txt');sidecar.write_text('1 2\n',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'Invalid seed'):fake.seeds(resolve_seed_file(bsp,progress=fixtures.quiet))


if __name__=='__main__':unittest.main()

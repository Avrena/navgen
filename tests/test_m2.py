from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import struct
import unittest
import zlib

import numpy as np

from navgen.areas import AreaGraph,JUMP,OBSTACLE_TOP,probe_portal
from navgen.bsp import BRUSH_SIDE_DTYPE,Content,convex_planes,steam_libraries
from navgen.collision import Collision,SceneBuilder,entity_rules
from navgen.gma import GMA
from navgen.sampling import adjacent,overlapped,allocate
from navgen.seeds import component_seeds
import test_generation as fixtures
from test_generation import collision

WORK=Path(__file__).resolve().parents[1]/'work'


def gma_bytes(entries,version=3):
    header=b'GMAD'+struct.pack('<BQQ',version,0,0)+(b'\0' if version>1 else b'')
    header+=b'test\0{}\0author\0'+struct.pack('<i',1)
    data=b''
    for i,(name,content) in enumerate(entries,1):
        header+=struct.pack('<I',i)+name.encode()+b'\0'+struct.pack('<qI',len(content),zlib.crc32(content))
        data+=content
    return header+struct.pack('<I',0)+data


class M2Tests(unittest.TestCase):
    def test_brush_solidity_table(self):
        for cls in ('func_smokevolume','func_dustmotes','func_dustcloud','func_precipitation','func_vehicleclip','func_clip_vphysics'):
            with self.subTest(cls=cls):self.assertEqual(entity_rules({'classname':cls}),(False,False))
        for cls,flag in (('func_rotating',64),('func_movelinear',8),('func_wall_toggle',1),('func_rot_button',1),('func_conveyor',2)):
            self.assertTrue(entity_rules({'classname':cls})[0])
            self.assertFalse(entity_rules({'classname':cls,'spawnflags':str(flag)})[0])
        self.assertTrue(entity_rules({'classname':'func_button','spawnflags':'1'})[0])
        self.assertTrue(entity_rules({'classname':'func_lod','Solid':'0'})[0])
        self.assertFalse(entity_rules({'classname':'func_lod','Solid':'1'})[0])
        self.assertFalse(entity_rules({'classname':'func_brush','StartDisabled':'1','Solidity':'0'})[0])
        self.assertTrue(entity_rules({'classname':'func_brush','StartDisabled':'1','Solidity':'2'})[0])
        self.assertFalse(entity_rules({'classname':'func_monitor','Solidity':'1'})[0])
        self.assertEqual(entity_rules({'classname':'func_reflective_glass','Solidity':'0'}),(True,False))
        self.assertIsNone(entity_rules({'classname':'unknown_brush'})[0])

    def test_gma_versions_offsets_and_crc(self):
        WORK.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=WORK) as temp:
            p=Path(temp)/'test.gma'
            for version in (1,2,3):
                p.write_bytes(gma_bytes([('Models/Test.mdl',b'abc'),('models/test.phy',b'defgh')],version))
                gma=GMA(p);self.assertEqual(gma.read('models/test.mdl'),b'abc');self.assertEqual(gma.read('models/test.phy'),b'defgh')
            p.write_bytes(p.read_bytes()[:-1]+b'x')
            with self.assertRaisesRegex(ValueError,'CRC'):gma.read('models/test.phy')
            p.write_bytes(gma_bytes([('a',b'123')])[:-2])
            with self.assertRaisesRegex(ValueError,'Truncated'):GMA(p)
            p.write_bytes(gma_bytes([('../outside',b'a')]))
            with self.assertRaisesRegex(ValueError,'Invalid content path'):GMA(p)

    def test_content_precedence_and_conflict(self):
        WORK.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=WORK) as temp:
            steamapps=Path(temp)/'steamapps';game=steamapps/'common/Demo/garrysmod'
            model=game/'addons/loose/models/test.mdl';model.parent.mkdir(parents=True);model.write_bytes(b'loose')
            gma=steamapps/'workshop/content/4000/123/content.gma';gma.parent.mkdir(parents=True)
            gma.write_bytes(gma_bytes([('models/test.mdl',b'workshop'),('models/test.phy',b'physics')]))
            content=Content(SimpleNamespace(pak=None),game)
            self.assertEqual(content.read('models/test.mdl'),b'loose')
            self.assertEqual(content.read('models/test.phy'),b'physics')
            self.assertEqual(len(content.conflicts),1)
            self.assertEqual(content.conflicts[0]['path'],'models/test.mdl')
            self.assertEqual(content.conflicts[0]['different_sources'][0]['source'],str(gma))

    def test_steam_library_list(self):
        WORK.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=WORK) as temp:
            root=Path(temp);self.assertEqual(steam_libraries(root),[root])
            (root/'steamapps').mkdir()
            (root/'steamapps/libraryfolders.vdf').write_text('"libraryfolders"\n{\n\t"0"\n\t{\n\t\t"path"\t\t"E:\\\\Games\\\\Steam"\n'
                '\t\t"apps"\n\t\t{\n\t\t\t"4000"\t\t"1"\n\t\t}\n\t}\n\t"1"\n\t{\n\t\t"PATH"  "F:\\\\Lib \\"x\\""\n\t}\n}\n',encoding='utf-8')
            self.assertEqual(steam_libraries(root),[root,Path('E:\\Games\\Steam'),Path('F:\\Lib "x"')])

    def test_brush_side_byte_fields(self):
        side=np.frombuffer(struct.pack('<HhhBB',7,2,-1,0,1),dtype=BRUSH_SIDE_DTYPE)[0]
        self.assertEqual(BRUSH_SIDE_DTYPE.itemsize,8);self.assertEqual(side['bevel'],0);self.assertEqual(side['thin'],1)

    def ramp(self):
        # Surface normal Z=0.6, between the engine and optimizer settings.
        p=np.array([[-200,-200,-500],[-200,200,-500],[200,-200,-500],[200,200,-500],
                    [-200,-200,-800/3],[-200,200,-800/3],[200,-200,800/3],[200,200,800/3]],float)
        b=SceneBuilder();b.add(convex_planes(p),p.min(0),p.max(0))
        return Collision(b.pack(),{},[])

    def test_slope_ground_jump_and_fixed_overlap(self):
        world=self.ramp();start=np.array([0.,0.,.7]);target=np.array([25.,0.,.7])
        self.assertFalse(adjacent(world.scene,start,target,.7)[0])
        self.assertTrue(adjacent(world.scene,start,target,.55)[0])
        self.assertTrue(overlapped(world.scene,start))
        helpers=fixtures.GenerationTests()
        for limit,marked in ((.7,True),(.55,False)):
            g=helpers.empty_graph();g.slope_limit=limit;g.collision=world;g.scene=world.scene
            a=helpers.area(g,[0,0,25,25]);a.box[5]=a.box[6]=100/3
            g.mark_jumps();self.assertEqual(bool(a.attr&JUMP),marked)

    def test_component_seed_largest_area_and_grounding(self):
        helpers=fixtures.GenerationTests();g=helpers.empty_graph()
        a=helpers.area(g,[0,0,25,25],20);b=helpers.area(g,[25,0,125,100],20);c=helpers.area(g,[200,0,250,50],100)
        g.connect(a.id,b.id,1);nav=g.nav(1)
        seeds,details=component_seeds(nav)
        self.assertEqual(len(seeds),2);np.testing.assert_allclose(seeds[0],[75,50,20])
        world=collision([([0,0,-20],[125,100,0]),([200,0,80],[250,50,90])])
        seeds,details=component_seeds(nav,world)
        self.assertEqual(len(seeds),2);np.testing.assert_allclose(seeds[:,2],[.03125,90.03125],atol=1e-5)

    def test_portal_midpoint_is_not_entire_connection(self):
        world=collision([([-100,-100,-20],[200,200,0]),([24,45,0],[26,55,350])])
        a=np.array([0,0,25,100,0,0,0,0],float);b=np.array([25,0,50,100,0,0,0,0],float)
        self.assertEqual(probe_portal(world.scene,a,b,1)[0],'unverified')
        self.assertEqual(probe_portal(world.scene,a,b,1,alternatives=True)[0],'portal_sample')

    def test_east_drop_probes_first_narrow_landing(self):
        world=collision([([-25,-25,90],[25,25,100]),([25,-25,-20],[50,25,0])])
        arrays=allocate(8);p=arrays[0];links=arrays[2];owner=arrays[-1]
        p[:4]=[[0,0,100.03125],[25,0,100.03125],[25,25,100.03125],[0,25,100.03125]]
        p[4:]=[[25,0,.03125],[50,0,.03125],[50,25,.03125],[25,25,.03125]]
        for offset in (0,4):
            for a,b,d in ((0,1,1),(1,2,2),(3,2,1),(0,3,2)):
                links[offset+a,d]=offset+b;links[offset+b,(d+2)%4]=offset+a
        owner[0]=0;owner[4]=1
        bounds=np.array([[0,0,25,25,*([100.03125]*4)],[25,0,50,25,*([.03125]*4)]])
        graph=AreaGraph(world,arrays,{},bounds,np.array([[0,1,2,3],[4,5,6,7]]),[0,0])
        graph.initial_links()
        self.assertIn(1,graph.areas[0].links[1]);self.assertEqual(graph.link_kinds[0,1,1],'drop')
        self.assertGreater(graph.counts['drop_part1'],0)


if __name__=='__main__':unittest.main()

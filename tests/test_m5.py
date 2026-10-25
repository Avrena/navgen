import struct
import unittest

import numpy as np

from navgen.bsp import BSP
from navgen.collision import SceneBuilder,trace,triangle_planes


def displacement(corners,normal,side=0,rotation=0,offsets=None,tags=None):
    """One packed BSP face and power-2 displacement, with nonzero lump offsets."""
    bsp=BSP.__new__(BSP)
    bsp.vertices=np.roll(np.asarray(corners,dtype=np.float32),rotation,axis=0)
    bsp.edges=np.array([[0,1],[1,2],[2,3],[3,0]],dtype=np.uint16)
    bsp.surfedges=np.arange(4,dtype=np.int32)
    bsp.planes=np.zeros(2,dtype=[('n','<f4',3),('d','<f4'),('type','<i4')])
    bsp.planes[side]['n']=normal
    bsp.faces=bytearray(56)
    struct.pack_into('<HBBihhh',bsp.faces,0,side,side,1,0,4,0,0)
    info=bytearray(176)
    struct.pack_into('<3f',info,0,*corners[0])
    struct.pack_into('<4ifiH',info,12,2,3,2,0,0.,1,0)
    verts=np.zeros(27,dtype=[('n','<f4',3),('d','<f4'),('alpha','<f4')])
    for index,offset in (offsets or {}).items():
        verts[index+2]['n']=offset;verts[index+2]['d']=1.
    values=np.zeros(35,dtype='<u2')
    if tags is not None:values[3:]=tags
    lumps={26:bytes(info),33:verts.tobytes(),48:values.tobytes()}
    bsp.lump=lambda index:lumps[index]
    return bsp


class DisplacementTests(unittest.TestCase):
    flat=((0,0,0),(0,100,0),(100,100,0),(100,0,0))

    def assert_one_sided(self,tri,expected):
        planes,front=triangle_planes(tri)
        np.testing.assert_allclose(front[:3],expected,atol=1e-12)
        builder=SceneBuilder();builder.add(planes,tri.min(0),tri.max(0),kind=2,front=front)
        scene=builder.pack();center=tri.mean(0)
        a=center+10*front[:3];b=center-10*front[:3]
        for radius in (0.,1.):
            with self.subTest(hull_radius=radius):
                mins=np.full(3,-radius);maxs=-mins
                hit=trace(scene,a,b,mins,maxs)
                self.assertLess(hit[0],1.);self.assertTrue(hit[7])
                np.testing.assert_allclose(hit[1:4],expected,atol=1e-12)
                self.assertEqual(trace(scene,b,a,mins,maxs)[0],1.)

    def test_side_one_front_is_face_winding_for_ray_and_hull(self):
        wall=((0,0,0),(0,0,100),(100,0,100),(100,0,0))
        bsp=displacement(wall,(0,-1,0),side=1)
        triangles=list(bsp.displacements())
        self.assertEqual(len(triangles),32)
        self.assert_one_sided(triangles[0][0],(0,-1,0))

    def test_fold_keeps_grid_winding_for_ray_and_hull(self):
        # Moving the shared vertex past the left edge turns the first triangle over.
        bsp=displacement(self.flat,(0,0,1),offsets={6:(-50,0,0)})
        tri,contents,tag=next(bsp.displacements())
        np.testing.assert_array_equal(tri,[[0,0,0],[-25,25,0],[0,25,0]])
        self.assertEqual((contents,tag),(1,0))
        self.assert_one_sided(tri,(0,0,-1))

    def test_checkerboard_diagonals_and_rotated_start(self):
        expected=[[[0,0,0],[25,25,0],[0,25,0]],[[0,0,0],[25,0,0],[25,25,0]],
                  [[25,0,0],[50,0,0],[25,25,0]],[[50,0,0],[50,25,0],[25,25,0]]]
        for rotation in range(4):
            with self.subTest(rotation=rotation):
                triangles=list(displacement(self.flat,(0,0,1),rotation=rotation).displacements())
                np.testing.assert_array_equal([tri for tri,_,_ in triangles[:4]],expected)
                self.assert_one_sided(triangles[0][0],(0,0,1))

    def test_removed_triangle_preserves_following_tag_and_geometry(self):
        tags=[1,32,4,8]+[0]*28
        bsp=displacement(self.flat,(0,0,1),tags=tags)
        triangles=list(bsp.displacements())
        self.assertEqual(len(triangles),31)
        self.assertEqual([tag for _,_,tag in triangles[:3]],[1,4,8])
        np.testing.assert_array_equal(triangles[1][0],[[25,0,0],[50,0,0],[25,25,0]])


if __name__=='__main__':unittest.main()

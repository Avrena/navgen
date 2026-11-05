"""Deterministic flat ledges and optional bitwise native differential checks."""
import json
import os
import subprocess
import unittest

import numpy as np

from navgen.bsp import _flat_basis, convex_planes


FLAT_CASES = (
    np.array([[0., 0., 0.], [4., 1., 2.], [1., 3., -1.], [5., 4., 1.],
              [2.5, 2., .5]], dtype=np.float64),
    np.array([[10., -5., 3.], [13., -3., 7.], [16., -1., 11.],
              [7., -7., -1.]], dtype=np.float64),
    np.array([[1., 2., 3.], [1., 2., 3.]], dtype=np.float64),
)


class FlatConvexTests(unittest.TestCase):
    def test_canonical_basis(self):
        for rank, points in zip((2, 1, 0), FLAT_CASES):
            centered = points - points[0]
            self.assertEqual(np.linalg.matrix_rank(centered, tol=1e-7), rank)
            basis = _flat_basis(centered, rank)
            np.testing.assert_allclose(basis @ basis.T, np.eye(3), atol=1e-15)
            if rank:
                axis = basis[2 if rank == 2 else 0]
                self.assertGreater(axis[np.argmax(np.abs(axis))], 0)
            planes = convex_planes(points)
            self.assertTrue(np.isfinite(planes).all())
            self.assertTrue(np.all(points @ planes[:, :3].T <= planes[:, 3] + 1e-12))
            np.testing.assert_array_equal(planes.view(np.uint64), convex_planes(points).view(np.uint64))

    @unittest.skipUnless(os.environ.get('NAVGEN_CPP_PROBE'), 'native module probe not configured')
    def test_native_bitwise(self):
        for points in FLAT_CASES:
            request = {'op': 'convex', 'points': points.tolist()}
            result = subprocess.run([os.environ['NAVGEN_CPP_PROBE']], input=json.dumps(request),
                                    text=True, capture_output=True, check=True)
            native = np.asarray(json.loads(result.stdout)['planes'], dtype=np.float64)
            reference = convex_planes(points)
            np.testing.assert_array_equal(native.view(np.uint64), reference.view(np.uint64))

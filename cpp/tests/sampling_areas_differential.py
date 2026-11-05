"""Compare six synthetic sampling/area scenes with the Python reference.

Run in the reference Python environment after building navgen_module_probe:

    OPENBLAS_CORETYPE=SkylakeX python cpp/tests/sampling_areas_differential.py \
        --probe cpp/build/navgen_module_probe --threads 1,2,max

The probe path may point to any CMake build directory. No fixture files are
needed. Sampling floats are compared bit for bit, including signed zero;
the complete NAV records, cleanup counts and link audit are also compared.
"""
import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from navgen.areas import build_areas
from navgen.bsp import box_vertices, convex_planes
from navgen.collision import Collision, SceneBuilder
from navgen.runtime import configure_threads
from navgen.sampling import sample


FIELDS = ('pos', 'normal', 'links', 'parent', 'visited', 'nextxy', 'crouch',
          'blocked', 'ground', 'obsh', 'obsa', 'obsb', 'disp', 'owner')
BASE = ([-150, -150, -20], [150, 150, 0])
CASES = {
    'low-room': ([([0, 0, -20], [200, 200, 0]),
                  ([0, 0, 60], [200, 200, 80])],
                 [[100, 100, 1]], [[0, 0, 0], [200, 200, 80]]),
    'step': ([BASE, ([0, -150, 0], [150, 150, 16])],
             [[-100, 0, 1]], [[-150, -150, 0], [150, 150, 100]]),
    'wall': ([BASE, ([-4, -150, 0], [4, 150, 200])],
             [[-100, 0, 1], [100, 0, 1]],
             [[-150, -150, 0], [150, 150, 220]]),
    'fence': ([BASE, ([-4, -150, 0], [4, 150, 40])],
              [[-100, 0, 1]], [[-150, -150, 0], [150, 150, 100]]),
    'overhang': ([BASE, ([-150, -150, 100], [0, 150, 120])],
                 [[-100, 0, 121], [100, 0, 1]],
                 [[-150, -150, 0], [150, 150, 220]]),
    'empty': ([], [[0, 0, 1]], [[-25, -25, 0], [25, 25, 100]]),
}


def quiet(*args, **kwargs):
    pass


def collision(boxes):
    builder = SceneBuilder()
    for lo, hi in boxes:
        points = box_vertices(lo, hi)
        builder.add(convex_planes(points), points.min(0), points.max(0))
    return Collision(builder.pack(), {}, [])


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def first_difference(left, right, path='root'):
    if isinstance(left, dict) and isinstance(right, dict):
        if left.keys() != right.keys():
            return path + '.keys', list(left), list(right)
        for key in left:
            difference = first_difference(left[key], right[key], path + '.' + key)
            if difference:
                return difference
    elif isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            return path + '.length', len(left), len(right)
        for index, (a, b) in enumerate(zip(left, right)):
            difference = first_difference(a, b, path + f'[{index}]')
            if difference:
                return difference
    elif isinstance(left, float) and isinstance(right, (float, int)):
        if struct.pack('<d', left) != struct.pack('<d', right):
            return path, left, right
    elif left != right:
        return path, left, right
    return None


def compare_arrays(reference, actual):
    if set(actual) != set(FIELDS):
        raise AssertionError(('sampling fields', list(FIELDS), list(actual)))
    for key, expected in zip(FIELDS, reference):
        got = np.asarray(actual[key], dtype=expected.dtype)
        if got.size != expected.size:
            raise AssertionError((key, 'size', expected.size, got.size))
        got = got.reshape(expected.shape)
        left = np.ascontiguousarray(expected).view(np.uint8).reshape(-1, expected.dtype.itemsize)
        right = np.ascontiguousarray(got).view(np.uint8).reshape(-1, expected.dtype.itemsize)
        differences = np.flatnonzero(np.any(left != right, axis=1))
        if len(differences):
            index = np.unravel_index(differences[0], expected.shape)
            raise AssertionError((key, index, expected[index], got[index]))


def run_probe(probe, request):
    result = subprocess.run([str(probe)], input=json.dumps(request), text=True,
                            capture_output=True, check=True)
    return json.loads(result.stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--threads', default='1,2,max',
                        help='Comma-separated counts, with max for logical cores')
    parser.add_argument('--only', action='append', choices=CASES)
    args = parser.parse_args()
    maximum = os.cpu_count() or 1
    counts = sorted({maximum if value == 'max' else int(value)
                     for value in args.threads.split(',')})
    if any(value < 1 or value > maximum for value in counts):
        parser.error(f'thread counts must be between 1 and {maximum}')
    probe = args.probe.resolve()
    for name, (boxes, seeds, bounds) in CASES.items():
        if args.only and name not in args.only:
            continue
        for threads in counts:
            configure_threads(threads)
            world = collision(boxes)
            arrays, index = sample(world, np.array(seeds, dtype=float), bounds, quiet)
            request = {'scene': {'boxes': [{'lo': lo, 'hi': hi} for lo, hi in boxes]},
                       'seeds': seeds, 'bounds': bounds, 'bsp_size': 1,
                       'threads': threads, 'op': 'sample'}
            compare_arrays(arrays, run_probe(probe, request))
            graph, _ = build_areas(world, arrays, index, quiet)
            expected = plain({'nav': graph.nav(1), 'cleanup': graph.counts,
                              'link_audit': graph.link_audit})
            request['op'] = 'areas'
            difference = first_difference(expected, run_probe(probe, request))
            if difference:
                raise AssertionError(difference)
            print(f'MATCH {name:9} threads={threads} nodes={len(arrays[0])} '
                  f'areas={len(graph.areas)}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

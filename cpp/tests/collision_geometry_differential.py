"""Deterministic Python/native collision-geometry differential checks.

Run with the reference Python environment and the built JSON module probe:

    OPENBLAS_CORETYPE=SkylakeX python cpp/tests/collision_geometry_differential.py \
        --probe cpp/build/navgen_module_probe --report geometry-results.json

No fixture files are required. Float arrays are compared bit for bit, including
signed zero. Failed requests and both outputs are saved in --failures, or in a
temporary directory when that option is omitted. --category selects one group.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import platform
import subprocess
import sys
import tempfile

import numpy as np
import scipy
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from navgen.bsp import (BRUSH_MASK, BRUSH_SIDE_DTYPE, NPC_MASK, box_vertices,
                        brush_vertices, convex_planes, rotation)
from navgen.collision import SceneBuilder, trace
from navgen.planes import bevel_planes, triangle_kernel
from navgen.runtime import configure_threads


CATEGORIES = ('convex', 'rotation', 'bevel', 'triangle', 'trace', 'brush_vertices')


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def compare_array(expected, actual, field):
    expected = np.asarray(expected, dtype=np.float64)
    actual = np.asarray(actual, dtype=np.float64)
    if expected.size != actual.size:
        return {'field': field, 'expected_shape': list(expected.shape),
                'native_shape': list(actual.shape)}
    actual = actual.reshape(expected.shape)
    left = np.ascontiguousarray(expected).view(np.uint64).ravel()
    right = np.ascontiguousarray(actual).view(np.uint64).ravel()
    differences = np.flatnonzero(left != right)
    if not len(differences):
        return None
    index = int(differences[0])
    coordinate = np.unravel_index(index, expected.shape)
    return {'field': field, 'different_elements': len(differences),
            'index': list(coordinate), 'expected': expected[coordinate],
            'native': actual[coordinate], 'expected_bits': f'{left[index]:016x}',
            'native_bits': f'{right[index]:016x}'}


def geometry_inputs(rng):
    hulls = []
    boxes = []
    for index in range(32):
        scale = 10. ** rng.uniform(-1., 3.)
        half = rng.uniform(.5, 2., 3) * scale
        angles = rng.uniform(-360., 360., 3)
        origin = rng.uniform(-2000., 2000., 3)
        if index == 0:
            half, angles, origin = np.ones(3), np.zeros(3), np.zeros(3)
        elif index == 1:
            angles, origin = np.array([-0., 90., -0.]), np.array([0., -0., 0.])
        matrix = rotation(angles)
        points = box_vertices(-half, half) @ matrix.T + origin
        boxes.append((points, matrix, origin))
        hulls.append((f'rotated_box_{index:02}', points))
    for index in range(40):
        count = int(rng.integers(4, 25))
        points = rng.normal(size=(count, 3)) * 10. ** rng.uniform(-2., 3.)
        points += rng.uniform(-1000., 1000., 3)
        if index % 7 == 0:
            points = np.concatenate([points, points[[0, -1]]])
        hulls.append((f'spatial_cloud_{index:02}', points))
    for index in range(24):
        local = rng.normal(size=(int(rng.integers(3, 20)), 2))
        local *= 10. ** rng.uniform(-1., 3.)
        matrix = rotation(rng.uniform(-360., 360., 3))
        points = np.column_stack([local, np.zeros(len(local))]) @ matrix.T
        points += rng.uniform(-1000., 1000., 3)
        hulls.append((f'flat_rank2_{index:02}', points))
    for index in range(16):
        direction = rng.normal(size=3)
        factors = np.array([-2., -.25, 0., .5, 2.])
        if index % 4 == 0:
            factors[-1] = factors[0]
        points = factors[:, None] * direction + rng.uniform(-100., 100., 3)
        hulls.append((f'flat_rank1_{index:02}', points))
    origins = [np.array([0., -0., 0.]), np.array([-0., -0., -0.])]
    origins += [rng.uniform(-1000., 1000., 3) for _ in range(6)]
    for index, origin in enumerate(origins):
        hulls.append((f'flat_rank0_{index:02}', np.repeat(origin[None, :], 4, axis=0)))
    hulls += [
        ('signed_zero_xy', np.array([[0., -0., 0.], [8., -0., 0.],
                                    [8., 8., -0.], [-0., 8., -0.]])),
        ('signed_zero_xz', np.array([[-0., 0., -0.], [8., 0., -0.],
                                    [8., -0., 8.], [0., -0., 8.]])),
    ]
    return hulls, boxes


def make_scene(objects):
    builder = SceneBuilder()
    for item in objects:
        options = {key: item[key] for key in ('contents', 'walkable', 'monster',
                   'transparent', 'push', 'kind', 'flags', 'front') if key in item}
        for key in ('flags', 'front'):
            if key in options:
                options[key] = np.asarray(options[key])
        builder.add(np.asarray(item['planes']), np.asarray(item['lo']),
                    np.asarray(item['hi']), **options)
    return builder.pack()


def trace_values(objects, rays):
    scene = make_scene(objects)
    return np.asarray([trace(scene, np.asarray(ray['start'], dtype=float),
                             np.asarray(ray['end'], dtype=float),
                             np.asarray(ray.get('mins', [0., 0., 0.]), dtype=float),
                             np.asarray(ray.get('maxs', [0., 0., 0.]), dtype=float),
                             ray.get('mask', BRUSH_MASK), ray.get('walk', True),
                             ray.get('brute', False)) for ray in rays], dtype=float)


def cases(seed):
    rng = np.random.default_rng(seed)
    hulls, boxes = geometry_inputs(rng)
    for name, points in hulls:
        yield 'convex', name, {'op': 'convex', 'points': points}, lambda p=points: {'planes': convex_planes(p)}

    angles = [np.array([0., -0., 0.]), np.array([-0., -0., -0.])]
    angles += [np.array([pitch, yaw, roll], dtype=float)
               for pitch, yaw, roll in ((0, 90, 0), (90, 0, 0), (0, 0, 90),
                                       (-90, -180, 270), (180, 180, 180),
                                       (360, -360, 720), (45, 45, 45))]
    angles += list(rng.uniform(-720., 720., (64, 3)))
    for index, angle in enumerate(angles):
        yield 'rotation', f'rotation_{index:02}', {'op': 'rotation', 'angles': angle}, lambda a=angle: rotation(a)

    for index, (_, points) in enumerate(hulls[:40]):
        hull = ConvexHull(points)
        edges = np.asarray(list({tuple(sorted((int(t[a]), int(t[b]))))
                                 for t in hull.simplices for a, b in ((0, 1), (1, 2), (2, 0))}), dtype=np.int64)
        yield 'bevel', f'bevel_{index:02}', {'op': 'bevel', 'points': points, 'edges': edges}, lambda p=points, e=edges: {'planes': bevel_planes(p, e)}

    triangles = [
        np.array([[0., -0., 0.], [100., 0., 0.], [0., 100., -0.]]),
        np.array([[0., 0., 0.], [0., 100., 0.], [100., 0., 0.]]),
        np.zeros((3, 3)),
        np.array([[0., 0., 0.], [1., 2., 3.], [2., 4., 6.]]),
    ]
    triangles += [rng.normal(size=(3, 3)) * 10. ** rng.uniform(-1., 3.)
                  + rng.uniform(-1000., 1000., 3) for _ in range(64)]
    for index, points in enumerate(triangles):
        def expected_triangle(p=points):
            planes, front = triangle_kernel(p)
            return {'planes': planes, 'front': front}
        yield 'triangle', f'triangle_{index:02}', {'op': 'triangle', 'points': points}, expected_triangle

    terrain = []
    for index, points in enumerate(triangles[:28]):
        planes, front = triangle_kernel(points)
        if not len(planes):
            continue
        item = {'planes': planes, 'lo': points.min(0), 'hi': points.max(0),
                'front': front, 'kind': 2, 'contents': 1,
                'flags': np.arange(len(planes), dtype=np.int64) % 8}
        terrain.append(item)
        centre, normal = points.mean(0), front[:3]
        rays = []
        for distance in (.03125, 1., 100.):
            for sign in (1., -1.):
                for hull in (False, True):
                    for brute in (False, True):
                        rays.append({'start': centre + sign * normal * distance,
                                     'end': centre - sign * normal * 100.,
                                     'mins': [-.45, -.45, 0.] if hull else [0., 0., 0.],
                                     'maxs': [.45, .45, 55.] if hull else [0., 0., 0.],
                                     'brute': brute})
        yield 'trace', f'one_sided_{index:02}', {'op': 'trace', 'scene': {'objects': [item]}, 'rays': rays}, lambda o=[item], r=rays: trace_values(o, r)

    objects = terrain[:12]
    for index, (points, _, _) in enumerate(boxes[:12]):
        contents = (1, 2, 0x4000, 0x2000000)[index % 4]
        planes = convex_planes(points)
        objects.append({'planes': planes, 'lo': points.min(0), 'hi': points.max(0),
                        'contents': contents, 'walkable': index % 5 == 0,
                        'monster': index % 4 == 3, 'transparent': index % 4 == 1,
                        'push': index % 4 == 2, 'flags': np.arange(len(planes), dtype=np.int64) % 8})
    rays = []
    for index, item in enumerate(objects):
        centre = (item['lo'] + item['hi']) * .5
        delta = rng.normal(size=3) * 300.
        for mask in (1, BRUSH_MASK, NPC_MASK):
            for walk in (False, True):
                for brute in (False, True):
                    rays.append({'start': centre + delta, 'end': centre - delta,
                                 'mins': [-2., -2., 0.], 'maxs': [2., 2., 55.],
                                 'mask': mask, 'walk': walk, 'brute': brute})
    yield 'trace', 'mixed_bvh_filters', {'op': 'trace', 'scene': {'objects': objects}, 'rays': rays}, lambda o=objects, r=rays: trace_values(o, r)

    for index, (points, matrix, _) in enumerate(boxes[:24]):
        normals = np.concatenate([np.eye(3), -np.eye(3)]) @ matrix.T
        planes = np.column_stack([normals, (points @ normals.T).max(0)])
        bevel = np.zeros(len(planes), dtype=np.uint8)
        if index % 3 == 0:
            planes = np.concatenate([planes, [[1., 1., 1., 100000.]]])
            bevel = np.append(bevel, np.uint8(1))
        sides = np.zeros(len(planes), dtype=BRUSH_SIDE_DTYPE)
        sides['bevel'] = bevel
        yield 'brush_vertices', f'brush_{index:02}', {'op': 'brush_vertices', 'planes': planes, 'bevel': bevel}, lambda p=planes, s=sides: brush_vertices(p, s)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--failures', type=Path)
    parser.add_argument('--seed', type=int, default=20261105)
    parser.add_argument('--category', action='append', choices=CATEGORIES)
    args = parser.parse_args()
    configure_threads(1)
    probe = args.probe.resolve()
    failure_dir = args.failures
    rows = []
    totals, failures = Counter(), Counter()
    for category, name, request, oracle in cases(args.seed):
        if args.category and category not in args.category:
            continue
        totals[category] += 1
        request = plain(request)
        expected, actual, difference = None, None, None
        try:
            expected = oracle()
            process = subprocess.run([str(probe)], input=json.dumps(request),
                                     text=True, capture_output=True, check=True)
            actual = json.loads(process.stdout)
            if isinstance(expected, dict):
                for field, values in expected.items():
                    difference = compare_array(values, actual[field], field)
                    if difference:
                        break
            else:
                difference = compare_array(expected, actual, category)
        except Exception as error:
            difference = {'error': type(error).__name__, 'message': str(error)}
            if isinstance(error, subprocess.CalledProcessError):
                difference['stderr'] = error.stderr
        row = {'category': category, 'name': name, 'match': difference is None}
        if difference:
            failures[category] += 1
            row['difference'] = plain(difference)
            if failure_dir is None:
                failure_dir = Path(tempfile.mkdtemp(prefix='navgen-geometry-'))
            failure_dir.mkdir(parents=True, exist_ok=True)
            saved = {'request': request, 'expected': plain(expected),
                     'native': plain(actual), 'difference': plain(difference)}
            (failure_dir / f'{category}-{name}.json').write_text(json.dumps(saved, indent=2) + '\n')
            print(f'FAIL {category}/{name}: {json.dumps(plain(difference))}', flush=True)
        rows.append(row)
    result = {'seed': args.seed, 'python': platform.python_version(),
              'numpy': np.__version__, 'scipy': scipy.__version__,
              'cases': rows, 'totals': dict(totals), 'failures': dict(failures)}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    for category, count in totals.items():
        print(f'{category}: {count - failures[category]}/{count} MATCH', flush=True)
    if sum(failures.values()):
        print(f'Failing inputs: {failure_dir}', flush=True)
    return int(bool(sum(failures.values())))


if __name__ == '__main__':
    raise SystemExit(main())

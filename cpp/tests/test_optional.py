"""Differential tests for native merge, hiding, discovery and ladder helpers.

Run with the reference environment and --probe pointing to navgen_module_probe.
Every comparison is exact; only elapsed merge time is excluded.
"""
import argparse
import copy
import json
import os
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault('OPENBLAS_CORETYPE', 'SkylakeX')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
from navgen.bsp import box_vertices, convex_planes
from navgen.collision import Collision, SceneBuilder, triangle_planes
from navgen.discovery import ladder_specs
from navgen.geometry import support_origin
from navgen.hiding import compute_hiding_spots
from navgen.ladders import build_ladders
from navgen.merge import merge_nav, union_box
from navgen.pathing import native_portal
from navgen.runtime import set_num_threads

PROBE = Path(__file__).resolve().parents[1] / 'build' / 'navgen_module_probe'
THREADS = 1


def normalise(value):
    if isinstance(value, np.ndarray):
        return normalise(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): normalise(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, bytes)):
        return [normalise(item) for item in value]
    return value


def run_probe(request):
    process = subprocess.run([str(PROBE)], input=json.dumps(normalise(request)),
                             text=True, capture_output=True, check=False)
    if process.returncode:
        raise AssertionError(f'Native probe failed: {process.stderr}')
    return json.loads(process.stdout)


def area(ident, rect, z=.03125, attr=0):
    heights = [z] * 4 if np.isscalar(z) else list(z)
    return {'id': ident, 'attr': attr, 'nw': [rect[0], rect[1], heights[0]],
            'se': [rect[2], rect[3], heights[2]], 'ne_z': heights[1],
            'sw_z': heights[3], 'links': [[], [], [], []], 'hiding': [],
            'encounters': [], 'place': 0, 'ladders': [[], []], 'occupy': [0., 0.],
            'light': [1., 1., 1., 1.], 'visible': [], 'inherit_vis': 0}


def mesh(rectangles, z=.03125, chain=False):
    result = {'magic': 0xFEEDFACE, 'version': 16, 'subversion': 0, 'bsp_size': 1,
              'analyzed': 0, 'places': [], 'has_unnamed': 1,
              'areas': [area(i + 1, rect, z) for i, rect in enumerate(rectangles)],
              'ladders': [], 'trailing': []}
    if chain:
        for i in range(len(rectangles) - 1):
            result['areas'][i]['links'][1] = [i + 2]
            result['areas'][i + 1]['links'][3] = [i + 1]
    return result


def scene_boxes(boxes):
    return {'boxes': [{'lo': lo, 'hi': hi} for lo, hi in boxes]}


def collision(value):
    builder = SceneBuilder()
    for item in value.get('boxes', []):
        points = box_vertices(item['lo'], item['hi'])
        options = {key: item[key] for key in ('contents', 'walkable', 'monster',
                                             'transparent', 'push', 'kind') if key in item}
        builder.add(convex_planes(points), points.min(0), points.max(0), **options)
    for item in value.get('objects', []):
        options = {key: item[key] for key in ('contents', 'walkable', 'monster',
                                             'transparent', 'push', 'kind', 'flags', 'front') if key in item}
        for key in ('flags', 'front'):
            if key in options:
                options[key] = np.asarray(options[key])
        builder.add(np.asarray(item['planes'], float), np.asarray(item['lo'], float),
                    np.asarray(item['hi'], float), **options)
    return Collision(builder.pack(), {}, [])


def bsp_entities(path, entities):
    text = '\n'.join('{\n' + '\n'.join(f'"{key}" "{value}"' for key, value in entity.items())
                     + '\n}' for entity in entities).encode() + b'\0'
    header = bytearray(1036)
    struct.pack_into('<4sI', header, 0, b'VBSP', 20)
    struct.pack_into('<IIII', header, 8, len(header), len(text), 0, 0)
    path.write_bytes(header + text)


class OptionalTests(unittest.TestCase):
    def exact(self, expected, actual, path='result'):
        expected = normalise(expected)
        if isinstance(expected, dict):
            self.assertIsInstance(actual, dict, path)
            self.assertEqual(set(expected), set(actual), path)
            for key in expected:
                self.exact(expected[key], actual[key], path + '.' + key)
        elif isinstance(expected, list):
            self.assertIsInstance(actual, list, path)
            self.assertEqual(len(expected), len(actual), path)
            for i, (left, right) in enumerate(zip(expected, actual)):
                self.exact(left, right, f'{path}[{i}]')
        elif isinstance(expected, float):
            self.assertEqual(expected, actual, path)
            if expected == 0:
                self.assertEqual(np.signbit(expected), np.signbit(float(actual)), path)
        else:
            self.assertEqual(expected, actual, path)

    def hiding(self, nav, scene=None):
        scene = scene or {}
        original = copy.deepcopy(nav)
        world = collision(scene)
        stats = compute_hiding_spots(nav, world)
        native = run_probe({'op': 'hiding', 'nav': original, 'scene': scene, 'threads': THREADS})
        self.exact({'nav': nav, 'stats': stats}, native)

    def merge(self, nav, **options):
        request = {'op': 'merge', 'nav': nav, **options}
        kwargs = dict(options)
        if 'scene' in kwargs:
            kwargs['scene'] = collision(kwargs['scene']).scene
        result, stats = merge_nav(nav, **kwargs)
        stats.pop('seconds')
        self.exact({'nav': result, 'stats': stats}, run_probe(request))

    def test_hiding_corner_fallbacks_and_float32_spacing(self):
        for width, height in ((100., 100.), (20., 100.), (100., 20.), (20., 20.),
                              (1., 1.), (0., 0.), (55., 55.), (54.999999, 55.000001)):
            with self.subTest(width=width, height=height):
                self.hiding(mesh([[12500.125, -8500.5, 12500.125 + width, -8500.5 + height]],
                                 z=[.03125, 17.123456, 35.5, 70.000003]))
        for flag in (2, 0x200, 2 | 0x200):
            nav = mesh([[0, 0, 100, 100]])
            nav['areas'][0]['attr'] = flag
            nav['areas'][0]['hiding'] = [[99, [0., 0., 0.], 8]]
            self.hiding(nav)
        self.hiding(mesh([]))

    def test_hiding_topology_thresholds_and_entity_filter(self):
        for edge in (19.999999, 20., 20.000001, 80.):
            nav = mesh([[0, 0, 100, 100], [edge, -25, 80., 0]])
            nav['areas'][0]['links'][0] = [2]
            nav['areas'][1]['links'][2] = [1]
            for reciprocal, jump in ((True, False), (False, False), (True, True)):
                candidate = copy.deepcopy(nav)
                if not reciprocal:
                    candidate['areas'][1]['links'][2] = []
                if jump:
                    candidate['areas'][1]['attr'] = 2
                self.hiding(candidate)
        nav = mesh([[0, 0, 100, 100]])
        for properties in ({}, {'walkable': True}, {'transparent': True}, {'push': True},
                           {'contents': 0x20}, {'monster': True}):
            scene = scene_boxes([([-10, -10, 50], [110, 110, 70])])
            scene['boxes'][0].update(properties)
            self.hiding(copy.deepcopy(nav), scene)
        walls = scene_boxes([([-40, -40, -10], [-10, 140, 140]),
                             ([110, -40, -10], [140, 140, 140]),
                             ([-40, -40, -10], [140, -10, 140]),
                             ([-40, 110, -10], [140, 140, 140])])
        self.hiding(nav, walls)

    def test_merge_semantics_and_ordered_frontiers(self):
        pair = mesh([[0, 0, 25, 25], [25, 0, 50, 25]], chain=True)
        self.merge(pair)
        for field, value in (('attr', 1), ('place', 2), ('light', [0., 0., 0., 0.]),
                             ('blocked', True), ('damaging', True), ('ladders', [[1], []]),
                             ('underwater', True), ('underwater', None)):
            nav = copy.deepcopy(pair)
            nav['areas'][1][field] = value
            self.merge(nav)
        for flag in (8192, 256, 0x80000000):
            nav = copy.deepcopy(pair)
            for item in nav['areas']:
                item['attr'] = flag
            self.merge(nav)
        self.merge(pair, locked_ids=[2])
        for count in (3, 6, 11):
            nav = mesh([[25 * i, 0, 25 * (i + 1), 25] for i in range(count)], chain=True)
            nav['areas'][0]['attr'] = nav['areas'][-1]['attr'] = 8192
            nav['areas'][-1]['links'][3] = []
            self.merge(nav)
            nav['areas'].reverse()
            self.merge(nav)
        self.merge(mesh([]))

    def test_merge_geometric_rejections_and_accumulated_corners(self):
        for rect, z in (([25.01, 0, 50, 25], .03125), ([25, 0, 50, 20], .03125),
                        ([25, 0, 50, 25], .6)):
            nav = mesh([[0, 0, 25, 25], rect], chain=True)
            nav['areas'][1].update(area(2, rect, z))
            nav['areas'][1]['links'][3] = [1]
            self.merge(nav)
        nav = mesh([[0, 0, 25, 25], [25, 0, 50, 25]], chain=True)
        nav['areas'][1]['links'][3] = []
        self.merge(nav)
        nav['areas'][1]['links'][0] = [1]
        self.merge(nav)
        nav = mesh([[0, 0, 25, 25], [25, 0, 50, 25], [50, 0, 75, 25]], chain=True)
        for item, left, right in zip(nav['areas'], [0., .5, 0.], [.5, 0., -.5]):
            item['nw'][2] = item['sw_z'] = left
            item['se'][2] = item['ne_z'] = right
        self.merge(nav)
        for item in nav['areas']:
            item['nw'][2] = item['sw_z'] = item['nw'][0] * .5
            item['se'][2] = item['ne_z'] = item['se'][0] * .5
        self.merge(nav)
        nav = mesh([[0, 0, 25, 25], [25, 0, 50, 25], [0, -25, 50, 0]], chain=True)
        nav['areas'][1]['links'][1] = []
        nav['areas'][1]['links'][0] = [3]
        nav['areas'][2]['links'] = [[], [], [], []]
        nav['areas'][2]['attr'] = 8192
        for item in nav['areas'][:2]:
            item['nw'][2] = item['sw_z'] = item['nw'][0] * .2
            item['se'][2] = item['ne_z'] = item['se'][0] * .2
        for height in (18., 23.999999, 24., 24.000001, 58., -200.):
            item = nav['areas'][2]
            item['nw'][2] = item['se'][2] = item['ne_z'] = item['sw_z'] = height
            self.merge(nav)

    def test_merge_collision_witnesses(self):
        nav = mesh([[-25, 0, 0, 25], [0, 0, 25, 25], [25, 0, 50, 25], [50, 0, 75, 25]], chain=True)
        nav['areas'][0]['attr'] = nav['areas'][3]['attr'] = 8192
        nav['areas'][3]['links'][3] = []
        scenes = [scene_boxes([([-100, -100, -20], [200, 200, 0])]),
                  scene_boxes([([-100, -100, -20], [200, 200, 0]), ([24, -10, 0], [26, 35, 100])]),
                  {}]
        for scene in scenes:
            self.merge(nav, scene=scene)

    def test_native_portal_and_union_boxes(self):
        rng = np.random.default_rng(147)
        for _ in range(24):
            xy = rng.uniform(-20000, 20000, 2)
            wh = rng.uniform(.01, 500, 2)
            a = np.r_[xy, xy + wh, rng.uniform(-300, 300, 4)]
            bxy = xy + rng.uniform(-600, 600, 2)
            b = np.r_[bxy, bxy + rng.uniform(.01, 600, 2), rng.uniform(-300, 300, 4)]
            for direction in range(4):
                self.exact(native_portal(a, b, direction),
                           run_probe({'op': 'native_portal', 'a': a, 'b': b, 'direction': direction}))
        for slope in (0., .125, .5, 1.):
            a = np.array([0., 0., 25., 25., 0., 25 * slope, 25 * slope, 0.])
            b = np.array([25., 0., 50., 25., 25 * slope, 50 * slope, 50 * slope, 25 * slope])
            for delta in (0., .5, .50000001):
                shifted = b.copy(); shifted[4:] += delta
                self.exact(union_box(a, shifted), run_probe({'op': 'union_box', 'a': a, 'b': shifted}))
                self.exact(union_box(shifted, a), run_probe({'op': 'union_box', 'a': shifted, 'b': a}))

    def test_support_origins(self):
        scenes = [{}, scene_boxes([([-200, -200, -20], [200, 200, 0])]),
                  scene_boxes([([-200, -200, -20], [200, 200, 0]), ([-200, -200, 50], [200, 200, 70])]),
                  scene_boxes([([-200, -200, -20], [200, 200, 0]), ([-5, -200, 0], [5, 200, 70])])]
        points = np.array([[-100., -100., 0.], [100., -100., 0.], [0., 100., 20.]])
        planes, front = triangle_planes(points)
        scenes.append({'objects': [{'planes': planes, 'front': front, 'lo': points.min(0),
                                    'hi': points.max(0), 'kind': 2}]})
        for scene in scenes:
            world = collision(scene)
            for point in ([0., 0., .05], [0., 0., -1.], [0., 0., 100.], [200., 200., 0.]):
                good, origin = support_origin(world.scene, np.array(point), .7)
                self.exact({'supported': good, 'origin': origin},
                           run_probe({'op': 'geometry', 'scene': scene, 'point': point, 'slope': .7}))

    def test_ladders_records_support_and_obstruction(self):
        nav = mesh([[-120, -40, -20, 40], [20, -40, 120, 40]], z=.05)
        nav['areas'][1] = area(2, [20, -40, 120, 40], 128.05)
        specs = [{'class': 'info_ladder', 'hammerid': '1', 'bottom': np.array([10., 0., 0.]),
                  'top': np.array([10., 0., 128.]), 'width': 64., 'axis': 0,
                  'lo': np.array([0., -32., 0.]), 'hi': np.array([20., 32., 128.])},
                 {'class': 'func_useableladder', 'hammerid': '2', 'bottom': np.array([0., 0., 0.]),
                  'top': np.array([0., 0., 128.]), 'width': 25., 'axis': None}]
        base = [([-200, -100, -20], [200, 100, 0]), ([0, -100, 0], [200, 100, 128])]
        for support in (False, True):
            for blocks in (base, base + [([-200, -100, 65], [0, 100, 75])], []):
                for spec in specs:
                    scene = scene_boxes(blocks)
                    expected = copy.deepcopy(nav)
                    stats = build_ladders(expected, collision(scene), [spec], support, .7)
                    self.exact({'nav': expected, 'stats': stats},
                               run_probe({'op': 'ladders', 'nav': nav, 'scene': scene,
                                          'specs': [spec], 'support_convention': support, 'slope': .7}))
        self.exact({'nav': nav, 'stats': build_ladders(copy.deepcopy(nav), collision({}), [])},
                   run_probe({'op': 'ladders', 'nav': nav, 'specs': []}))

    def test_ladder_entity_parsing_and_rejections(self):
        entities = [{'classname': 'func_useableladder', 'origin': '100 200 300',
                     'point0': '1 2 103', 'point1': '1 2 3'},
                    {'ClassName': 'info_ladder', 'mins.x': '0', 'mins.y': '-32', 'mins.z': '0',
                     'maxs.x': '20', 'maxs.y': '32', 'maxs.z': '128', 'hammerid': '7'},
                    {'classname': 'info_ladder', 'startdisabled': '1'},
                    {'classname': 'func_useableladder', 'parentname': 'moving'},
                    {'classname': 'func_useableladder', 'point0': '0 0 0', 'point1': '0 0 0'},
                    {'classname': 'func_useableladder', 'point0': 'oops 0 0', 'point1': '0 0 100'},
                    {'classname': 'info_ladder', 'mins.x': '0x1', 'mins.y': '0', 'mins.z': '0',
                     'maxs.x': '10', 'maxs.y': '10', 'maxs.z': '100'},
                    {'classname': 'info_ladder', 'mins.x': '0', 'mins.y': '0', 'mins.z': 'nan',
                     'maxs.x': '10', 'maxs.y': '10', 'maxs.z': '100'},
                    {'classname': 'info_ladder'}, {'classname': 'info_player_start'}]
        result, skipped = ladder_specs(SimpleNamespace(entities=entities))
        keys = ('class', 'hammerid', 'bottom', 'top', 'width', 'axis')
        expected = {'specs': [{key: spec[key] for key in keys} for spec in result], 'skipped': skipped}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'ladder_entities.bsp'
            bsp_entities(path, entities)
            self.exact(expected, run_probe({'op': 'ladder_specs', 'path': str(path)}))


def main():
    global PROBE, THREADS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, default=PROBE)
    parser.add_argument('--threads', type=int, default=1)
    args, remaining = parser.parse_known_args()
    PROBE = args.probe.resolve()
    THREADS = args.threads
    set_num_threads(THREADS)
    unittest.main(argv=[sys.argv[0], *remaining])


if __name__ == '__main__':
    main()

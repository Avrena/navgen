"""Differential coverage of entity discovery and merge environment queries."""
import argparse
import copy
import json
import struct
import tempfile
import unittest
from pathlib import Path

import test_optional as common
import numpy as np
from navgen.bsp import BSP
from navgen.discovery import discover_seeds, entity_sources, leaf_area
from navgen.geometry import centre_floor, clear_move, hull_move
from navgen.merge import merge_environment, merge_nav, world_contents
from navgen.runtime import trace_count


def synthetic_bsp(path, entities):
    lumps = {}
    lumps[0] = ('\n'.join('{\n' + '\n'.join(f'"{key}" "{value}"' for key, value in entity.items())
                           + '\n}' for entity in entities) + '\0').encode()
    lumps[1] = struct.pack('<4fi', 1., 0., 0., 0., 0)
    node = bytearray(32)
    struct.pack_into('<3i', node, 0, 0, -1, -2)
    lumps[5] = node
    leaves = bytearray(64)
    struct.pack_into('<IhH', leaves, 0, 0, 0, 3)
    struct.pack_into('<IhH', leaves, 32, 0x20, 0, 7)
    lumps[10] = leaves
    models = [([-300., -200., -50.], [300., 200., 350.]),
              ([-10., -20., 0.], [10., 20., 72.]),
              ([-5., -15., 0.], [5., 15., 90.])]
    lumps[14] = b''.join(struct.pack('<9f3i', *lo, *hi, 0., 0., 0., 0, 0, 0) for lo, hi in models)
    header = bytearray(1036)
    struct.pack_into('<4sI', header, 0, b'VBSP', 20)
    payload = bytearray()
    for number, content in sorted(lumps.items()):
        struct.pack_into('<4I', header, 8 + 16 * number, len(header) + len(payload), len(content),
                         1 if number == 10 else 0, 0)
        payload.extend(content)
    path.write_bytes(header + payload)


class EnvironmentTests(unittest.TestCase):
    exact = common.OptionalTests.exact

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / 'environment.bsp'

    def test_sources_world_ladders_and_brush_transform(self):
        entities = [{'classname': 'worldspawn'},
                    {'classname': 'info_player_start', 'origin': '25 0 36'},
                    {'classname': 'info_player_deathmatch', 'origin': '-25 0 36'},
                    {'classname': 'info_ladder_dismount', 'origin': '32 32 128'},
                    {'classname': 'func_door', 'origin': '65 10 0', 'model': '*1', 'angles': '17 83 29'},
                    {'classname': 'prop_door_rotating', 'origin': '30 -70 0'},
                    {'classname': 'func_button', 'model': '*2', 'origin': '-60 0 0', 'angle': '-1'},
                    {'classname': 'func_useableladder', 'origin': '100 200 300', 'point0': '0 0 0', 'point1': '0 0 128'},
                    {'classname': 'info_ladder', 'mins.x': '80', 'mins.y': '-32', 'mins.z': '0',
                     'maxs.x': '100', 'maxs.y': '32', 'maxs.z': '128', 'hammerid': '19'},
                    {'classname': 'info_ladder', 'spawnflags': '2'}]
        synthetic_bsp(self.path, entities)
        seeds = np.array([[50., 0., 36.], [50., 0., 36.]])
        sources, specs, skipped = entity_sources(BSP(self.path), seeds)
        self.exact({'sources': sources, 'specs': specs, 'skipped': skipped},
                   common.run_probe({'op': 'sources', 'path': str(self.path), 'seeds': seeds}))

    def test_discovery_limits_rejections_deduplication_and_leafs(self):
        entities = [{'classname': 'info_player_start', 'origin': '25 0 36'},
                    {'classname': 'info_player_start', 'origin': '26 0 36'},
                    {'classname': 'info_player_start', 'origin': '50 0 36'},
                    {'classname': 'info_player_start', 'origin': '-40 0 36'},
                    {'classname': 'info_player_start', 'origin': '600 0 36'},
                    {'classname': 'info_player_start', 'origin': '180 0 36'},
                    {'classname': 'sky_camera', 'origin': '-10 0 200'},
                    {'classname': 'func_useableladder', 'point0': '0 0 0', 'point1': '0 0 128'}]
        synthetic_bsp(self.path, entities)
        seeds = np.array([[50., 0., 36.], [50., 0., 36.], [-50., 0., 36.]])
        scenes = [{}, common.scene_boxes([([-250, -150, -20], [150, 150, 0])]),
                  common.scene_boxes([([-250, -150, -20], [150, 150, 0]), ([-45, -20, 0], [-35, 20, 80])])]
        for scene in scenes:
            for limit, probes in ((4096, 200000), (0, 200000), (1, 200000), (4096, 1), (4096, 0)):
                world = common.collision(scene)
                effective, details, specs = discover_seeds(BSP(self.path), world, seeds, .7, limit, probes,
                                                          lambda *args, **kwargs: None)
                self.exact({'effective': effective, 'details': details, 'specs': specs},
                           common.run_probe({'op': 'discovery', 'path': str(self.path), 'scene': scene,
                                             'seeds': seeds, 'max_seeds': limit, 'max_probes': probes}))
        synthetic_bsp(self.path, [])
        effective, details, specs = discover_seeds(BSP(self.path), common.collision({}), np.empty((0, 3)),
                                                  progress=lambda *args, **kwargs: None)
        self.exact({'effective': effective, 'details': details, 'specs': specs},
                   common.run_probe({'op': 'discovery', 'path': str(self.path), 'seeds': []}))

    def test_environment_water_cache_locks_and_merge(self):
        nav = common.mesh([[-75, 0, -50, 25], [-50, 0, -25, 25], [-25, 0, 0, 25],
                           [0, 0, 25, 25], [25, 0, 50, 25], [50, 0, 75, 25]], chain=True)
        points = np.array([[-50., 0., .03125], [50., 0., .03125], [0., 0., .03125],
                           [-50., 0., .03125], [50., 0., .03125], [120., 0., 128.03125]])
        volume_entities = [{'classname': name, 'model': '*1', 'origin': f'{x} 0 {z}',
                            'angles': '17 83 29', 'startdisabled': '1', 'hammerid': str(i)}
                           for i, (name, x, z) in enumerate((('trigger_hurt', 0, 0), ('func_nav_blocker', 60, 0),
                             ('func_nav_avoid', -30, 100), ('func_nav_cost', 200, 0),
                             ('func_water', -70, 0), ('func_water_analog', 50, 20)))]
        for entities in ([], volume_entities):
            synthetic_bsp(self.path, entities)
            for scene in ({}, common.scene_boxes([([-250, -150, -20], [250, 150, 0])]),
                          common.scene_boxes([([-250, -150, -20], [250, 150, 0]), ([-50, -30, 0], [50, 30, 300])])):
                bsp = BSP(self.path)
                world = common.collision(scene)
                water, locked, environment = merge_environment(bsp, world, nav)
                values = [{'water': water(point), 'contents': world_contents(bsp, point),
                           'leaf_area': leaf_area(bsp, point)} for point in points]
                result, stats = merge_nav(nav, water, locked, scene=world.scene)
                stats.pop('seconds')
                self.exact({'environment': environment, 'locked_ids': list(locked), 'water': values,
                            'nav': result, 'stats': stats, 'traces': trace_count(world.scene)},
                           common.run_probe({'op': 'environment', 'path': str(self.path), 'scene': scene,
                                             'nav': nav, 'points': points}))

    def test_full_hull_movement_boundaries(self):
        scenes = [{}, common.scene_boxes([([-200, -200, -20], [200, 200, 0])]),
                  common.scene_boxes([([-200, -200, -20], [200, 200, 0]), ([0, -200, 0], [10, -12, 100]),
                                      ([0, 12, 0], [10, 200, 100])]),
                  common.scene_boxes([([-200, -200, -20], [200, 200, 0]), ([-200, -200, 0], [0, 200, 250])]),
                  common.scene_boxes([([-200, -200, -20], [200, 200, 0]), ([0, -100, 0], [30, 100, 18]),
                                      ([-200, -200, 80], [200, 200, 100])])]
        moves = [{'start': [-40., 0., .05], 'end': [50., 0., z]} for z in (.05, 18.05, 18.06, 58.05, 64.05, 64.061, -399.95, -400.)]
        moves += [{'start': [-17., 0., 250.05], 'end': [17., 0., .05]},
                  {'start': [17., 0., .05], 'end': [-17., 0., 250.05]}]
        for scene in scenes:
            world = common.collision(scene)
            expected = []
            for move in moves:
                start, end = np.array(move['start']), np.array(move['end'])
                good, z, normal = centre_floor(world.scene, start, .7)
                expected.append({'clear': clear_move(world.scene, start, end),
                                 'move': hull_move(world.scene, start, end),
                                 'floor': [good, None if np.isnan(z) else z, normal]})
            self.exact(expected, common.run_probe({'op': 'movement', 'scene': scene, 'moves': moves}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    args, remaining = parser.parse_known_args()
    common.PROBE = args.probe.resolve()
    unittest.main(argv=[__file__, *remaining])

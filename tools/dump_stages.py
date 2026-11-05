"""Run the unchanged Python generator and save the native differential stages.

Usage: python tools/dump_stages.py MAP --game GAME --dump-stages DIR [CLI options]
The wrappers observe completed values and never modify generator inputs/results.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import navgen.__main__ as cli
import navgen.areas as area_module
from navgen.bsp import DEFAULT_GAME
from navgen.navio import write_nav


def serialisable(value):
    if isinstance(value, bytes):
        return list(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): serialisable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [serialisable(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map', type=Path)
    parser.add_argument('--game', type=Path, default=DEFAULT_GAME)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--seeds', type=Path)
    parser.add_argument('--slope-limit', type=float, default=.7)
    parser.add_argument('--threads', type=int)
    parser.add_argument('--profile', choices=('sdk', 'merged'), default='sdk')
    parser.add_argument('--seed-discovery', choices=('none', 'entities'), default='none')
    parser.add_argument('--ladders', action='store_true')
    parser.add_argument('--missing-phy', choices=('skip', 'bbox'), default='skip')
    parser.add_argument('--dump-stages', type=Path, required=True)
    args = parser.parse_args()
    directory = args.dump_stages
    directory.mkdir(parents=True, exist_ok=True)

    def save(name, value):
        (directory / name).write_text(json.dumps(serialisable(value), sort_keys=True, indent=2) + '\n')

    original_load = cli.Collision.load
    original_sample = cli.sample
    original_areas = cli.build_areas
    original_hiding = cli.compute_hiding_spots
    original_seeds = cli.BSP.seeds
    original_rectangles = area_module.rectangles
    original_graph_init = area_module.AreaGraph.__init__
    original_passes = {}

    def seeds(bsp, *arguments, **keywords):
        result = original_seeds(bsp, *arguments, **keywords)
        np.save(directory / 'seeds.npy', result)
        return result

    def load(cls, *arguments, **keywords):
        collision = original_load(*arguments, **keywords)
        names = ('planes', 'pflags', 'first', 'count', 'lo', 'hi', 'meta', 'front',
                 'blo', 'bhi', 'escape', 'leaf', 'order')
        for name, array in zip(names, collision.scene):
            np.save(directory / f'collision-{name}.npy', array)
        save('collision.json', {'counts': collision.counts, 'unresolved': collision.unresolved,
                                'content_audit': collision.content_audit})
        return collision

    def sample(collision, seeds, *arguments, **keywords):
        np.save(directory / 'effective-seeds.npy', seeds)
        result = original_sample(collision, seeds, *arguments, **keywords)
        names = ('pos', 'normal', 'links', 'parent', 'visited', 'nextxy', 'crouch', 'blocked',
                 'ground', 'obsh', 'obsa', 'obsb', 'disp', 'owner')
        for name, array in zip(names, result[0]):
            np.save(directory / f'sampling-{name}.npy', array)
        return result

    def rectangles(*arguments, **keywords):
        result = original_rectangles(*arguments, **keywords)
        for name, array in zip(('boxes', 'corners', 'attrs', 'valid'), result):
            np.save(directory / f'rectangle-{name}.npy', array)
        return result

    def graph_init(graph, *arguments, **keywords):
        original_graph_init(graph, *arguments, **keywords)
        save('areas-rectangles.json', graph.nav(args.map.stat().st_size))

    def observed_pass(original, phase):
        def run(graph, *arguments, **keywords):
            result = original(graph, *arguments, **keywords)
            save(f'areas-{phase}.json', graph.nav(args.map.stat().st_size))
            return result
        return run

    def areas(*arguments, **keywords):
        graph, timings = original_areas(*arguments, **keywords)
        nav = graph.nav(args.map.stat().st_size)
        save('sdk-nav.json', nav)
        (directory / 'sdk.nav').write_bytes(write_nav(nav))
        save('cleanup.json', graph.counts)
        save('link-audit.json', graph.link_audit)
        return graph, timings

    def hiding(nav, *arguments, **keywords):
        if args.profile == 'merged':
            save('merged-nav.json', nav)
        result = original_hiding(nav, *arguments, **keywords)
        save('nav.json', nav)
        return result

    cli.BSP.seeds = seeds
    cli.Collision.load = classmethod(load)
    cli.sample = sample
    cli.build_areas = areas
    cli.compute_hiding_spots = hiding
    area_module.rectangles = rectangles
    area_module.AreaGraph.__init__ = graph_init
    for phase, name in (('connections', 'initial_links'), ('jump_marking', 'mark_jumps'),
                        ('merge', 'merge'), ('overhang_splits', 'split_overhangs'),
                        ('square_up', 'square'), ('stairs', 'mark_stairs'),
                        ('jump_stitching', 'stitch_jumps'), ('fence_tops', 'fence_tops'),
                        ('fence_overlap_cleanup', 'remove_overlapping_fences'),
                        ('corner_helpers', 'corner_helpers'),
                        ('sdk_connection_fixes', 'sdk_connection_fixes'),
                        ('connection_validation', 'fix_connections')):
        original = getattr(area_module.AreaGraph, name)
        original_passes[name] = original
        setattr(area_module.AreaGraph, name, observed_pass(original, phase))
    try:
        result = cli.generate(args.map, args.game, args.out, args.seeds, args.missing_phy,
                              slope_limit=args.slope_limit, threads=args.threads,
                              profile=args.profile, seed_discovery=args.seed_discovery,
                              ladders=args.ladders)
    finally:
        cli.BSP.seeds = original_seeds
        cli.Collision.load = original_load
        cli.sample = original_sample
        cli.build_areas = original_areas
        cli.compute_hiding_spots = original_hiding
        area_module.rectangles = original_rectangles
        area_module.AreaGraph.__init__ = original_graph_init
        for name, original in original_passes.items():
            setattr(area_module.AreaGraph, name, original)
    save('result.json', result)
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()

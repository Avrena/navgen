"""Compare packed BSP/PHY inputs and collision scenes with the Python reference.

    OPENBLAS_CORETYPE=SkylakeX python cpp/tests/collision_bsp_differential.py \
        --probe cpp/build/navgen_module_probe --report bsp-results.json

Synthetic coverage needs no external files. --fixtures optionally adds every
PHY from a fixture pack's game directory. --failures preserves failing inputs
and model files outside the temporary working directory for later replay.
"""
import argparse
from collections import Counter
import json
import lzma
from pathlib import Path
import platform
import shutil
import struct
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from navgen.bsp import BSP, box_vertices, parse_phy, rotation
from navgen.collision import Collision
from navgen.runtime import configure_threads


SCENE_FIELDS = ('planes', 'pflags', 'first', 'count', 'lo', 'hi', 'meta',
                'front', 'blo', 'bhi', 'escape', 'leaf', 'order')


def quiet(*args, **kwargs):
    pass


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def first_difference(expected, actual, path='root'):
    if isinstance(expected, np.ndarray):
        got = np.asarray(actual)
        if expected.size == got.size == 0:
            return None
        if expected.shape != got.shape:
            return {'path': path, 'expected_shape': expected.shape, 'native_shape': got.shape}
        if expected.dtype.kind == 'f':
            left = np.ascontiguousarray(expected, dtype=np.float64).view(np.uint64)
            right = np.ascontiguousarray(got, dtype=np.float64).view(np.uint64)
            changed = left != right
        else:
            changed = expected != got
        indices = np.argwhere(changed)
        if len(indices):
            coordinate = tuple(indices[0])
            result = {'path': path, 'index': list(coordinate), 'different_elements': len(indices),
                      'expected': expected[coordinate], 'native': got[coordinate]}
            if expected.dtype.kind == 'f':
                result.update(expected_bits=f'{left[coordinate]:016x}', native_bits=f'{right[coordinate]:016x}')
            return result
    elif isinstance(expected, dict) and isinstance(actual, dict):
        if expected.keys() != actual.keys():
            return {'path': path, 'expected_keys': list(expected), 'native_keys': list(actual)}
        for key in expected:
            difference = first_difference(expected[key], actual[key], path + '.' + str(key))
            if difference:
                return difference
    elif isinstance(expected, (list, tuple)) and isinstance(actual, list):
        if len(expected) != len(actual):
            return {'path': path, 'expected_length': len(expected), 'native_length': len(actual)}
        for index, (left, right) in enumerate(zip(expected, actual)):
            difference = first_difference(left, right, path + f'[{index}]')
            if difference:
                return difference
    elif isinstance(expected, (float, np.floating)) and isinstance(actual, (int, float)):
        if struct.pack('<d', expected) != struct.pack('<d', actual):
            return {'path': path, 'expected': expected, 'native': actual}
    elif expected != actual:
        return {'path': path, 'expected': expected, 'native': actual}
    return None


def base_lumps(leaf_version=0):
    entities = ('{\n"classname" "worldspawn"\n}\n'
                '{\n"classname" "info_player_start"\n"origin" "0 0 36"\n}\n\0')
    return {0: entities.encode('ascii'),
            10: bytes(32 if leaf_version == 1 else 56),
            14: struct.pack('<9f3i', -5000., -5000., -5000., 5000., 5000., 5000.,
                            0., 0., 0., -1, 0, 0)}


def compressed_lump(data):
    settings = {'id': lzma.FILTER_LZMA1, 'dict_size': 1 << 20, 'lc': 3, 'lp': 0, 'pb': 2}
    packed = lzma.compress(data, format=lzma.FORMAT_RAW, filters=[settings])
    props = bytes([(settings['pb'] * 5 + settings['lp']) * 9 + settings['lc']])
    props += struct.pack('<I', settings['dict_size'])
    return b'LZMA' + struct.pack('<II', len(data), len(packed)) + props + packed


def write_bsp(path, lumps, compressed=(), leaf_version=0, static_props=None):
    lumps = dict(lumps)
    if static_props:
        version, props = static_props
        lumps[35] = bytes(20) + props
    header, payload = bytearray(1036), bytearray()
    struct.pack_into('<4sI', header, 0, b'VBSP', 20)
    for number, raw in sorted(lumps.items()):
        offset = len(header) + len(payload)
        if number == 35 and static_props:
            raw = struct.pack('<IIHHII', 1, 0x73707270, 0, version, offset + 20, len(props)) + props
        content = compressed_lump(raw) if number in compressed else raw
        struct.pack_into('<4I', header, 8 + 16 * number, offset, len(content),
                         leaf_version if number == 10 else 0,
                         len(raw) if number in compressed else 0)
        payload.extend(content)
    path.write_bytes(header + payload)


def displacement_lumps(power, side, vertex_rotation, folded, tagged):
    n = (1 << power) + 1
    if side:
        corners = np.array([[0., 0., 0.], [0., 0., 160.], [160., 0., 160.], [160., 0., 0.]])
        normal = [0., -1., 0.]
    else:
        corners = np.array([[0., 0., 0.], [0., 160., 0.], [160., 160., 0.], [160., 0., 0.]])
        normal = [0., 0., 1.]
    lumps = base_lumps()
    planes = bytearray(40)
    struct.pack_into('<4fi', planes, side * 20, *normal, 0., 0)
    lumps[1] = bytes(planes)
    lumps[3] = np.roll(corners.astype('<f4'), vertex_rotation, axis=0).tobytes()
    lumps[12] = np.array([[0, 1], [1, 2], [2, 3], [3, 0]], dtype='<u2').tobytes()
    lumps[13] = np.arange(4, dtype='<i4').tobytes()
    face, info = bytearray(56), bytearray(176)
    struct.pack_into('<HBBihhh', face, 0, side, side, 1, 0, 4, 0, 0)
    struct.pack_into('<3f', info, 0, *corners[0])
    struct.pack_into('<4ifiH', info, 12, 2, 3, power, 0, 0., 1, 0)
    lumps[7], lumps[26] = bytes(face), bytes(info)
    verts = np.zeros(n * n + 2, dtype=[('n', '<f4', 3), ('d', '<f4'), ('alpha', '<f4')])
    if folded:
        verts[n + 3]['n'] = [-320. / (n - 1), 0., 0.]
        verts[n + 3]['d'] = 1.
        indices = np.arange(2, len(verts), 5)
        verts[indices]['alpha'] = 7.
        # Non-integral float32 products exercise the displacement cast boundary.
        for index in indices:
            if index == n + 3:
                continue
            verts[index]['n'] = [.1234567, -.2345678, .3456789]
            verts[index]['d'] = np.float32(index * .127)
    lumps[33] = verts.tobytes()
    tags = np.zeros(3 + 2 * (n - 1) ** 2, dtype='<u2')
    if tagged:
        tags[3:] = np.resize(np.array([1, 32, 4, 8, 0, 64, 2], dtype='<u2'), len(tags) - 3)
    lumps[48] = tags.tobytes()
    return lumps


def phy_bytes(solids, vphy=True, header_padding=0):
    result = bytearray(struct.pack('<4i', 16 + header_padding, 0, len(solids), 0))
    result.extend(bytes(header_padding))
    for pieces in solids:
        ledges, point_pools = [], []
        for points, flags in pieces:
            points = np.asarray(points, dtype=float)
            triangles = [(0, i, i + 1) for i in range(1, len(points) - 1)]
            if not triangles:
                triangles = [(0, min(1, len(points) - 1), min(1, len(points) - 1))]
            ledge = bytearray(16 + len(triangles) * 16)
            struct.pack_into('<iiIh', ledge, 0, 0, 0, flags, len(triangles))
            for index, ids in enumerate(triangles):
                struct.pack_into('<4I', ledge, 16 + index * 16, index,
                                 *(0x12340000 | value for value in ids))
            stored = np.column_stack([points[:, 0], -points[:, 2], points[:, 1], np.zeros(len(points))])
            stored[:, :3] *= .0254
            ledges.append(ledge)
            point_pools.append(stored.astype('<f4').tobytes())
        ledger_size, point_offset, ledger_offset = sum(map(len, ledges)), 0, 0
        for ledge, pool in zip(ledges, point_pools):
            struct.pack_into('<i', ledge, 0, ledger_size + point_offset - ledger_offset)
            point_offset += len(pool)
            ledger_offset += len(ledge)
        surface = bytearray(48)
        struct.pack_into('<i', surface, 32, 48 + ledger_size + sum(map(len, point_pools)))
        surface.extend(b''.join(ledges) + b''.join(point_pools))
        if vphy:
            wrapper = bytearray(28)
            struct.pack_into('<4shhi', wrapper, 0, b'VPHY', 0, 0, 0)
            surface = wrapper + surface
        result.extend(struct.pack('<i', len(surface)) + surface)
    result.extend(b'"solid" { }\0')
    return bytes(result)


def mdl_bytes(lo, hi):
    result = bytearray(128)
    result[:4] = b'IDST'
    struct.pack_into('<6f', result, 104, *lo, *hi)
    return bytes(result)


def props_bytes(names, records, version):
    result = bytearray(struct.pack('<i', len(names)))
    for name in names:
        result.extend(name.encode('latin1').ljust(128, b'\0'))
    result.extend(struct.pack('<i3Hi', 3, 0, 1, 2, len(records)))
    stride = 84 if version == 11 else 72 if version == 10 else 56
    for record in records:
        raw = bytearray(stride)
        struct.pack_into('<6fH', raw, 0, *record['origin'], *record['angles'], record['model'])
        raw[30] = record['solid']
        if version == 11:
            struct.pack_into('<f', raw, 80, record.get('scale', 1.))
        result.extend(raw)
    return bytes(result)


def bsp_snapshot(path):
    bsp = BSP(path)
    return {'version': bsp.version, 'entities': bsp.entities, 'models': bsp.models,
            'seeds': bsp.seeds(),
            'displacements': [{'points': points, 'contents': contents, 'tags': tags}
                              for points, contents, tags in bsp.displacements()],
            'props': [{'name': name, 'solid': solid, 'origin': origin, 'angles': angles, 'scale': scale}
                      for name, solid, origin, angles, scale in bsp.static_props()]}


def collision_snapshot(path, game, missing_phy):
    collision = Collision.load(BSP(path), game, missing_phy=missing_phy, progress=quiet)
    counts = dict(collision.counts)
    counts.pop('build_seconds')
    return {'scene': dict(zip(SCENE_FIELDS, collision.scene)), 'counts': counts,
            'unresolved': collision.unresolved, 'content_audit': collision.content_audit}


def cases(root, fixtures):
    game = root / 'game'
    (game / 'models').mkdir(parents=True)
    for leaf_version in (0, 1):
        for compressed in (False, True):
            path = root / f'empty-leaf{leaf_version}-compressed{int(compressed)}.bsp'
            write_bsp(path, base_lumps(leaf_version), (0, 10, 14) if compressed else (), leaf_version)
            yield 'bsp', path.stem, {'op': 'bsp', 'path': str(path)}, lambda p=path: bsp_snapshot(p)
            yield 'collision', path.stem, {'op': 'collision', 'path': str(path), 'game': str(game)}, lambda p=path: collision_snapshot(p, game, 'skip')

    for power in (2, 3, 4):
        for side in (0, 1):
            for turn in range(4):
                folded = turn % 2 == 1
                tagged = (side + turn) % 2 == 1
                compressed = turn >= 2
                path = root / f'displacement-p{power}-side{side}-rotation{turn}.bsp'
                lumps = displacement_lumps(power, side, turn, folded, tagged)
                write_bsp(path, lumps, (0, 1, 3, 7, 12, 13, 14, 26, 33, 48) if compressed else ())
                yield 'bsp', path.stem, {'op': 'bsp', 'path': str(path)}, lambda p=path: bsp_snapshot(p)
                yield 'collision', path.stem, {'op': 'collision', 'path': str(path), 'game': str(game)}, lambda p=path: collision_snapshot(p, game, 'skip')

    box = box_vertices(np.array([-8., -12., -4.]), np.array([16., 20., 28.]))
    flat = np.array([[0., 0., 0.], [20., 0., 0.], [20., 20., 0.], [0., 20., 0.]])
    line = np.array([[-10., -20., -30.], [0., 0., 0.], [10., 20., 30.]])
    point = np.array([[0., -0., 0.]])
    pieces = [(box, 0), (flat, 0), (line, 0), (point, 0)]
    for index, (vphy, padding, solid_pieces) in enumerate([
            (True, 0, [[pieces[0]]]), (False, 0, [[pieces[0]]]),
            (True, 16, [pieces]), (False, 16, [pieces]),
            (True, 0, [[pieces[0]], [pieces[1], pieces[2]]]),
            (False, 0, [[pieces[0], (box * 2., 1), pieces[3]]])]):
        path = root / f'physics-{index}.phy'
        path.write_bytes(phy_bytes(solid_pieces, vphy, padding))
        yield 'phy', path.stem, {'op': 'phy', 'path': str(path)}, lambda p=path: parse_phy(p.read_bytes())

    names = ['models/shape.mdl', 'models/missing_phy.mdl', 'models/missing.mdl']
    (game / names[0]).write_bytes(mdl_bytes([-8., -12., -4.], [16., 20., 28.]))
    (game / names[0]).with_suffix('.phy').write_bytes(phy_bytes([[pieces[0], pieces[1], pieces[2]]]))
    (game / names[1]).write_bytes(mdl_bytes([-10., -20., -5.], [10., 20., 35.]))
    rng = np.random.default_rng(20261105)
    angles = [np.array([0., 0., 0.]), np.array([-0., 90., -0.]), np.array([90., 180., 270.])]
    angles += list(rng.uniform(-360., 360., (9, 3)))
    records = []
    for index, angle in enumerate(angles):
        for translation in range(3):
            records.append({'model': 0, 'solid': 6, 'angles': angle,
                            'origin': [index * 120. + translation * 25., translation * 140., index * -3.5],
                            'scale': (.5, 1., 1.75)[index % 3]})
    records += [{'model': 1, 'solid': solid, 'origin': [2000. + index * 50., -100., 25.],
                 'angles': angles[index], 'scale': 1.25}
                for index, solid in enumerate((6, 6, 2, 0, 3))]
    records += [{'model': 2, 'solid': 6, 'origin': [-300., 0., 0.], 'angles': [0., 0., 0.]}]
    for version in (4, 10, 11):
        path = root / f'static-props-v{version}.bsp'
        write_bsp(path, base_lumps(), static_props=(version, props_bytes(names, records, version)))
        yield 'bsp', path.stem, {'op': 'bsp', 'path': str(path)}, lambda p=path: bsp_snapshot(p)
        for policy in ('skip', 'bbox'):
            request = {'op': 'collision', 'path': str(path), 'game': str(game), 'missing_phy': policy}
            yield 'collision', path.stem + '-' + policy, request, lambda p=path, m=policy: collision_snapshot(p, game, m)

    if fixtures:
        for index, path in enumerate(sorted((fixtures / 'game').rglob('*.phy'))):
            yield 'fixture_phy', f'fixture-physics-{index}', {'op': 'phy', 'path': str(path)}, lambda p=path: parse_phy(p.read_bytes())


def save_failure(folder, name, request, expected, actual, difference):
    destination = folder / name
    destination.mkdir(parents=True, exist_ok=True)
    replay = dict(request)
    source = Path(request['path'])
    shutil.copy2(source, destination / source.name)
    replay['path'] = str((destination / source.name).resolve())
    if 'game' in request:
        shutil.copytree(request['game'], destination / 'game', dirs_exist_ok=True)
        replay['game'] = str((destination / 'game').resolve())
    record = {'request': replay, 'expected': plain(expected), 'native': plain(actual), 'difference': plain(difference)}
    (destination / 'failure.json').write_text(json.dumps(record, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--fixtures', type=Path)
    parser.add_argument('--failures', type=Path)
    args = parser.parse_args()
    configure_threads(1)
    rows, totals, failures = [], Counter(), Counter()
    failure_dir = args.failures
    with tempfile.TemporaryDirectory(prefix='navgen-bsp-') as temporary:
        for category, name, request, oracle in cases(Path(temporary), args.fixtures):
            expected, actual, difference = None, None, None
            totals[category] += 1
            try:
                expected = oracle()
                process = subprocess.run([str(args.probe.resolve())], input=json.dumps(request),
                                         text=True, capture_output=True, check=True)
                actual = json.loads(process.stdout)
                difference = first_difference(expected, actual)
            except Exception as error:
                difference = {'error': type(error).__name__, 'message': str(error)}
                if isinstance(error, subprocess.CalledProcessError):
                    difference['stderr'] = error.stderr
            row = {'category': category, 'name': name, 'match': difference is None}
            if difference:
                failures[category] += 1
                row['difference'] = plain(difference)
                if failure_dir is None:
                    failure_dir = Path(tempfile.mkdtemp(prefix='navgen-bsp-failures-'))
                save_failure(failure_dir, category + '-' + name, request, expected, actual, difference)
                print(f'FAIL {category}/{name}: {json.dumps(plain(difference))}', flush=True)
            rows.append(row)
    report = {'python': platform.python_version(), 'numpy': np.__version__,
              'cases': rows, 'totals': dict(totals), 'failures': dict(failures)}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    for category, count in totals.items():
        print(f'{category}: {count - failures[category]}/{count} MATCH', flush=True)
    if failures:
        print(f'Failing inputs: {failure_dir}', flush=True)
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())

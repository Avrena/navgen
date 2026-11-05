"""Differential set ordering, NAV I/O, and noninteractive CLI contracts.

python cpp/tests/test_runtime.py --binary BUILD/navgen --probe BUILD/navgen_module_probe
Add --fixtures FIXTURES to check the stock engine NAVs and CLI file handling.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import tempfile

from navgen.navio import read_nav, write_nav


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--fixtures', type=Path)
    args = parser.parse_args()
    binary, probe = args.binary.resolve(), args.probe.resolve()
    checks = 0

    def native(request):
        p = subprocess.run([str(probe)], input=json.dumps(request), text=True, capture_output=True)
        if p.returncode:
            raise AssertionError(p.stderr)
        return json.loads(p.stdout)

    rng = random.Random(31011)
    for trial in range(40):
        state = set()
        actions = []
        for step in range(25):
            kind = rng.choice(('add', 'discard', 'copy', 'update', 'union', 'difference', 'intersection'))
            values = [(rng.randrange(-80, 80), rng.randrange(-8, 8)) for _ in range(rng.randrange(1, 110))]
            if kind == 'add':
                for value in values: state.add(value)
            elif kind == 'discard':
                for value in values: state.discard(value)
            elif kind == 'copy': state = state.copy()
            elif kind == 'update': state.update(set(values))
            elif kind == 'union': state = state | set(values)
            elif kind == 'difference': state = state - set(values)
            elif kind == 'intersection': state = state & set(values)
            actions.append({'kind': kind, 'values': values})
        assert native({'op': 'pyset', 'actions': actions}) == [list(value) for value in state], trial
        checks += 1

    # Exercise every NAV field, including opaque place/trailer bytes and encounters.
    area = dict(id=1, attr=0x80000000, nw=(-25.125, -10., -0.), se=(100., 150., 3.5),
                ne_z=-2.25, sw_z=19.5, links=[[], [], [], []], hiding=[(9, (1.25, -2.5, 3.75), 8)],
                encounters=[((1, 2, 1, 3), [(9, 255)])], place=1, ladders=[[2], [3]],
                occupy=(.5, 2.75), light=(0., .25, .5, 1.), visible=[(1, 255)], inherit_vis=1)
    nav = dict(magic=0xFEEDFACE, version=16, subversion=0, bsp_size=0x87654321, analyzed=1,
               places=[b'\xffroom\0'], has_unnamed=1, areas=[area],
               ladders=[dict(id=2, width=25., top=(0., 0., 100.), bottom=(0., 0., 0.),
                             length=100., dir=3, areas=(1, 0, 0, 0, 1))], trailing=b'\0\xffopaque')
    serial = {**nav, 'places': [list(v) for v in nav['places']], 'trailing': list(nav['trailing'])}
    result = native({'op': 'navio', 'nav': serial})
    assert result['sha256'] == hashlib.sha256(write_nav(nav)).hexdigest()
    assert result['nav']['trailing'] == list(nav['trailing'])
    checks += 1
    for field, value in (('id', -1), ('id', 2**32), ('id', 1.0), ('place', 65536),
                         ('ne_z', 1e300)):
        invalid = copy.deepcopy(serial)
        invalid['areas'][0][field] = value
        process = subprocess.run([str(probe)], input=json.dumps({'op': 'navio', 'nav': invalid}),
                                 text=True, capture_output=True)
        assert process.returncode != 0, (field, value)
        checks += 1
    for argv, code, stderr in (([], 2, 'error:'), (['--help'], 0, ''),
                               (['x.bsp', '--threads', '0'], 1, 'ValueError:'),
                               (['x.bsp', '--threads', '-1'], 1, 'ValueError:'),
                               (['x.bsp', '--threads', '1.5'], 2, 'error:'),
                               (['x.bsp', '--slope-limit', '0'], 1, 'ValueError:'),
                               (['x.bsp', '--slope-limit', 'nan'], 1, 'ValueError:'),
                               (['x.bsp', '--slope-limit', '1.1'], 1, 'ValueError:'),
                               (['x.bsp', '--profile', 'other'], 2, 'error:'),
                               (['x.bsp', '--unknown'], 2, 'error:'),
                               (['missing-file.bsp'], 1, 'FileNotFoundError:')):
        p = subprocess.run([str(binary), *argv], text=True, capture_output=True)
        assert p.returncode == code and stderr in p.stderr, (argv, p.returncode, p.stderr)
        checks += 1
    if args.fixtures:
        fixtures = args.fixtures.resolve()
        with tempfile.TemporaryDirectory(prefix='navgen-test-') as folder:
            directory = Path(folder)
            for filename in ('gm_construct.nav', 'gm_flatgrass.nav'):
                source = fixtures / 'engine-nav' / filename
                output = directory / filename
                p = subprocess.run([str(probe), 'nav-roundtrip', str(source), str(output)], capture_output=True)
                assert p.returncode == 0 and source.read_bytes() == output.read_bytes()
                assert write_nav(read_nav(source.read_bytes())) == source.read_bytes()
                checks += 1
            mapfile = directory / '地图 with spaces.bsp'
            shutil.copyfile(fixtures / 'maps/gm_flatgrass.bsp', mapfile)
            original = mapfile.read_bytes()
            game = fixtures / 'game/garrysmod'
            def cli(*arguments):
                # The installed native executable has no Python or PATH dependency.
                environment = dict(os.environ, PATH=str(directory / 'empty-path'))
                return subprocess.run([str(binary), str(mapfile), '--game', str(game), '--threads', '1',
                                       *map(str, arguments)], cwd=directory, env=environment,
                                      capture_output=True, text=True)
            p = cli()
            assert p.returncode == 0, p.stderr
            assert mapfile.with_suffix('.nav').exists()
            checks += 1
            p = cli('--out', mapfile)
            assert p.returncode == 1 and 'Output must not replace' in p.stderr
            assert mapfile.read_bytes() == original
            checks += 1
            sidecar = mapfile.with_suffix('.navseeds.txt')
            sidecar.write_text('0 0 0\n')
            p = cli('--out', sidecar)
            assert p.returncode == 1 and sidecar.read_text() == '0 0 0\n'
            checks += 1
            p = cli()
            result = json.loads(p.stdout.splitlines()[-1])
            assert p.returncode == 0 and result['seed_file']['source'] == 'sidecar'
            checks += 1
            explicit = directory / 'empty seeds.txt'
            explicit.write_text('')
            p = cli('--seeds', explicit)
            result = json.loads(p.stdout.splitlines()[-1])
            assert p.returncode == 0 and result['seed_file']['source'] == 'explicit'
            checks += 1
            before = mapfile.with_suffix('.nav').read_bytes()
            p = cli('--seeds', directory / 'missing.txt')
            assert p.returncode == 1 and mapfile.with_suffix('.nav').read_bytes() == before
            assert not list(directory.glob('*.tmp'))
            checks += 1
    print(f'{checks} runtime, ordering and I/O checks passed')


if __name__ == '__main__':
    main()

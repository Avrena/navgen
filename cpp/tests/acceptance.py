"""Run every supplied case at explicit thread counts; gate NAV bytes and JSON.

The fixture pack remains external. --reference points to logs from run_cases.py
on this platform; expected.json supplies the independent Windows NAV hashes.
"""
import argparse
import hashlib
import json
import subprocess
import time
from pathlib import Path

KEYS = ('areas', 'links', 'nodes', 'seeds', 'bsp_size', 'map_sha256', 'nav_sha256',
        'nav_bytes', 'profile', 'slope_limit', 'missing_phy_policy', 'analyzed',
        'cleanup', 'hiding_spots', 'ladders', 'seed_discovery', 'link_audit',
        'collision', 'merge', 'unresolved', 'content_audit', 'sampling_speculation')


def result_from(text):
    for line in reversed(text.splitlines()):
        if line.startswith('{'):
            return json.loads(line)
    raise ValueError('No final JSON result')


def comparable(result):
    value = {key: result[key] for key in KEYS}
    value['collision'] = {key: item for key, item in value['collision'].items()
                          if key != 'build_seconds'}
    if value['merge'] is not None:
        value['merge'] = {key: item for key, item in value['merge'].items() if key != 'seconds'}
    return value


def differences(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(a.keys() | b.keys()):
            here = f'{path}.{key}' if path else key
            if key not in a or key not in b:
                yield here + ' (missing key)'
            else:
                yield from differences(a[key], b[key], here)
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            yield f'{path} (length {len(a)} != {len(b)})'
        for index, (left, right) in enumerate(zip(a, b)):
            yield from differences(left, right, f'{path}[{index}]')
    elif a != b:
        yield f'{path}: {a!r} != {b!r}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixtures', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--generator', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--threads', required=True, help='Comma-separated thread counts')
    parser.add_argument('--only', action='append')
    args = parser.parse_args()
    fixtures, generator, out = args.fixtures.resolve(), args.generator.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    cases = json.loads((fixtures / 'cases.json').read_text())
    expected = json.loads((fixtures / 'expected.json').read_text())
    results = []
    for threads in map(int, args.threads.split(',')):
        for case in cases:
            name = case['name']
            if args.only and name not in args.only:
                continue
            argv = list(case['args'])
            while '--threads' in argv:
                at = argv.index('--threads'); del argv[at:at + 2]
            nav = out / f'{name}-t{threads}.nav'
            if nav.exists():
                nav.unlink()
            start = time.perf_counter()
            process = subprocess.run([str(generator), *argv, '--threads', str(threads),
                                      '--game', 'game/garrysmod', '--out', str(nav)],
                                     cwd=fixtures, text=True, capture_output=True)
            seconds = time.perf_counter() - start
            (out / f'{name}-t{threads}.log').write_text(process.stdout + process.stderr)
            row = {'case': name, 'threads': threads, 'exit': process.returncode,
                   'seconds': seconds, 'expected_sha256': expected[name]['nav_sha256']}
            raw = nav.read_bytes() if nav.exists() else b''
            row['nav_sha256'] = hashlib.sha256(raw).hexdigest() if raw else None
            row['nav_match'] = row['nav_sha256'] == row['expected_sha256']
            row['json_match'] = False
            if process.returncode == 0:
                result = result_from(process.stdout)
                reference = result_from((args.reference / f'{name}.log').read_text())
                diff = list(differences(comparable(reference), comparable(result)))
                row['json_match'] = not diff
                row['json_differences'] = diff
                row['timings_seconds'] = result['timings_seconds']
                row['phase_keys_match'] = result['timings_seconds'].keys() == reference['timings_seconds'].keys()
            else:
                row['error'] = process.stderr[-2000:]
            results.append(row)
            print(f'{name:40} t={threads} NAV={"MATCH" if row["nav_match"] else "FAIL"} '
                  f'JSON={"MATCH" if row["json_match"] else "FAIL"} {seconds:.3f}s', flush=True)
            for difference in row.get('json_differences', [])[:8]:
                print(' ', difference, flush=True)
            (out / 'results.json').write_text(json.dumps(results, indent=2, sort_keys=True) + '\n')
    return int(any(not row['nav_match'] or not row['json_match'] or not row.get('phase_keys_match', False)
                   for row in results))


if __name__ == '__main__':
    raise SystemExit(main())

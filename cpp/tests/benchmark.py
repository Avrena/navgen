"""Measure warm-cache Python/native CLI phases and peak RSS on Linux.

This development tool uses wait4's maximum-resident-set measurement. Run it
after correctness checks, without other benchmark/build jobs, and retain all
repetitions. Inputs and generated NAVs remain outside the source repository.
"""
import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixtures', type=Path, required=True)
    parser.add_argument('--python', type=Path, required=True)
    parser.add_argument('--native', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--threads', type=int, required=True)
    parser.add_argument('--repetitions', type=int, default=3)
    args = parser.parse_args()
    fixtures, out = args.fixtures.resolve(), args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    # Resolving a virtualenv interpreter symlink would select the base environment.
    commands = {'python': [str(args.python.absolute()), '-I', '-m', 'navgen'],
                'native': [str(args.native.resolve())]}
    rows = []
    environment = dict(os.environ, OPENBLAS_CORETYPE='SkylakeX')
    for iteration in range(args.repetitions):
        for label in (('python', 'native') if iteration % 2 == 0 else ('native', 'python')):
            name = f'{label}-{iteration + 1}'
            command = [*commands[label],
                       'maps/gm_construct.bsp', '--game', 'game/garrysmod', '--threads',
                       str(args.threads), '--out', str(out / f'{name}.nav')]
            start = time.perf_counter()
            log = out / f'{name}.log'
            with log.open('w') as stream:
                process = subprocess.Popen(command, cwd=fixtures, stdout=stream,
                                           stderr=subprocess.STDOUT, text=True, env=environment)
                _, status, usage = os.wait4(process.pid, 0)
                process.returncode = os.waitstatus_to_exitcode(status)
            wall = time.perf_counter() - start
            output = log.read_text()
            if process.returncode:
                raise RuntimeError(f'{name}: {output[-2000:]}')
            result = json.loads(output.splitlines()[-1])
            row = {'label': label, 'iteration': iteration + 1, 'wall_seconds': wall,
                   'peak_rss_kib': int(usage.ru_maxrss), 'nav_sha256': result['nav_sha256'],
                   'timings_seconds': result['timings_seconds']}
            rows.append(row)
            print(name, f'{wall:.3f}s wall', row['peak_rss_kib'], 'KiB RSS', flush=True)
            (out / 'runs.json').write_text(json.dumps(rows, indent=2, sort_keys=True) + '\n')
    assert len({row['nav_sha256'] for row in rows}) == 1
    summary = {}
    for label in commands:
        selected = [row for row in rows if row['label'] == label]
        summary[label] = {
            'wall_seconds': statistics.median(row['wall_seconds'] for row in selected),
            'wall_range_seconds': [min(row['wall_seconds'] for row in selected),
                                   max(row['wall_seconds'] for row in selected)],
            'peak_rss_kib': statistics.median(row['peak_rss_kib'] for row in selected),
            'timings_seconds': {key: statistics.median(row['timings_seconds'][key] for row in selected)
                               for key in selected[0]['timings_seconds']}}
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
    print(json.dumps(summary, sort_keys=True))


if __name__ == '__main__':
    main()

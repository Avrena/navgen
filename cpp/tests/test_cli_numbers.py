"""Compare numeric spelling, option boundaries and exit codes with argparse.

Run in the reference environment with --binary pointing to the native CLI.
Coverage includes the Python 3.10 Unicode decimal-digit and whitespace rules.
"""
import argparse
import contextlib
import importlib.util
import io
import json
import math
import random
import struct
from pathlib import Path
import subprocess
import sys
import tempfile

from navgen.__main__ import main as python_main
import navgen.__main__ as reference_entry


def reference(arguments):
    stdout, stderr = io.StringIO(), io.StringIO()
    previous = sys.argv
    try:
        sys.argv = ['navgen', *arguments]
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            try:
                status = python_main()
            except SystemExit as error:
                status = error.code
    finally:
        sys.argv = previous
    return status, stdout.getvalue(), stderr.getvalue()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--probe', type=Path, help='Also compare exact numeric helper results')
    parser.add_argument('--reference-argparse', type=Path, help='CPython 3.10 argparse.py when using a newer test interpreter')
    args = parser.parse_args()
    if args.reference_argparse:
        spec = importlib.util.spec_from_file_location('reference_argparse', args.reference_argparse)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        reference_entry.argparse = module
    binary = args.binary.resolve()
    integers = ('1', '+1', '0001', '0_1', '0', '-0', '-1', '-1_0', '1_0', ' 1 ', '\t1\r\n',
                '2147483647', '2147483648', '-2147483648', '-2147483649',
                '999999999999999999999999999999', '-999999999999999999999999999999',
                '1' * 4300, '1' * 4301, '0' * 4301,
                '١', '１', '𝟙', '१', '٠_١', '-١', '-１', '\u20031\u00a0', '\u3000+١\u3000',
                '-1\n', '-١\n', '-1\r\n', '\x1c1', '\u2003\x1c1', '²', '一', '−1',
                '', ' ', '+', '-', '_1', '1_', '1__0', '0x1', '0b1', '1.0', '1e0')
    floats = ('.5', '0.5', '+0.5', '5.e-1', '5E-001', '0_0.5_5', '1_0e-2', ' .5 ', '\v.5\f',
              '0', '-0', '-0.0', '-0.', '-.5', '-1e-3', '1.', '1.0001',
              '1e-9999', '-1e-9999', '1e9999', '-1e9999', '1.7976931348623157e308',
              '4.9406564584124654e-324', '2.4703282292062327e-324',
              'inf', 'INFINITY', '+iNf', '-inf', 'nan', 'NaN', '+nan', '-NaN',
              '٠.٥', '０.５', '𝟢.𝟧', '٠_٠.٥_٥', '٥e-١', '-٠.٥', '-٠.٥\n',
              '\u2003.٥\u00a0', '\u202f+0.5\u205f', '\u0085.5\u1680', '\u2028.5\u2029',
              '\x1c.5', '\u2003\x1c.5', '².5', '−.5', 'İNf',
              '', ' ', '+', '-', '.', '1e', '1e+', '1e_', '1._0', '1_.0', '_0.5', '0.5_',
              '0.5__0', '1e_2', 'nan(0)', 'nan(payload)', '0x1p-1', '+0x1.0p-1', '0x1')
    checks = 0
    with tempfile.TemporaryDirectory(prefix='navgen-cli-') as folder:
        missing = str(Path(folder) / 'missing map.bsp')
        cases = []
        for option, values in (('--threads', integers), ('--slope-limit', floats)):
            for value in values:
                cases.extend(([missing, option, value], [missing, option + '=' + value]))
        cases += [[], ['--help'], ['-h'], ['--hel'], ['--h'], ['--help='], ['-hfoo'],
                  ['-hh'], ['-hhh'], ['-hhx'], ['-h='], ['-hh='], ['-h-h'],
                  ['--', '--help'], ['--', '-h'], ['--', '--'], [missing, '--', '--help'],
                  ['--help', '--unknown'], ['--unknown', '--help'], ['first', 'second', '--help'],
                  ['--help', '--s'], [missing, '--game', '--ladders'],
                  [missing, '--out', '--help'], [missing, '--threads', '--help'],
                  [missing, '--slope-limit', '--'], [missing, '--profile', '--ladders'],
                  [missing, '--ladders=yes'], [missing, '--ladders='],
                  ['-', '--threads', '1'], ['-1', '--threads', '1'],
                  [missing, '--gam', 'game folder'], [missing, '--pro=sdk'],
                  [missing, '--seed-discovery=none', '--profile=merged'],
                  [missing, '--game=--ladders'], [missing, '--seeds='], [missing, '--out=']]
        for arguments in cases:
            expected, _, reference_error = reference(arguments)
            # CPython 3.10 rejects these before executing the help action.
            # Newer argparse versions defer the same errors until afterwards.
            target_status = {('-hfoo',): 2, ('-hhx',): 2, ('--help', '--s'): 2}
            if tuple(arguments) in target_status:
                expected = target_status[tuple(arguments)]
            process = subprocess.run([str(binary), *arguments], text=True, capture_output=True)
            assert process.returncode == expected, (arguments, expected, process.returncode,
                                                     reference_error, process.stderr)
            if expected == 1:
                assert process.stderr.startswith('navgen failed: '), (arguments, process.stderr)
                if 'ValueError:' in reference_error:
                    assert 'ValueError:' in process.stderr, (arguments, reference_error, process.stderr)
            if expected == 2:
                assert 'navgen: error:' in process.stderr, (arguments, process.stderr)
            if expected == 0:
                assert process.stdout.startswith('usage: navgen'), (arguments, process.stdout)
            checks += 1
    print(f'{checks} numeric and argparse boundary comparisons passed')
    if args.probe:
        numbers = list(floats) + ['0.0', '-0.0', '2.2250738585072014e-308',
                   '1.00000000000000011102230246251565404236316680908203125',
                   '1.00000000000000011102230246251565404236316680908203126']
        rng = random.Random(31011)
        for _ in range(256):
            value = struct.unpack('>d', rng.getrandbits(64).to_bytes(8, 'big'))[0]
            if math.isfinite(value):
                numbers.append(format(value, '.17g'))
        whole = list(integers) + [str(value) for value in
                    (-(2**64), -(2**63)-1, -(2**63), 2**63-1, 2**63, 2**64-1, 2**64,
                     2**100, 2**100+1, -(2**100)-1, -(2**100)+1)]
        expected = {'floats': [], 'integers': []}
        for text in numbers:
            try:
                expected['floats'].append({'bits': struct.pack('>d', float(text)).hex()})
            except ValueError:
                expected['floats'].append({'error': 'ValueError'})
        for text in whole:
            try:
                value = int(text)
                expected['integers'].append({'low_bits': value & (2**64-1), 'nonzero': bool(value),
                    'clamped': min(2**63-1, max(-(2**63), value)),
                    'equals': [value == candidate for candidate in (-2, -1, 0, 1, 2)]})
            except ValueError:
                expected['integers'].append({'error': 'ValueError'})
        process = subprocess.run([str(args.probe.resolve())],
                    input=json.dumps({'op': 'python_numbers', 'floats': numbers, 'integers': whole}),
                    text=True, capture_output=True)
        assert process.returncode == 0, process.stderr
        actual = json.loads(process.stdout)
        for key, inputs in (('floats', numbers), ('integers', whole)):
            assert len(actual[key]) == len(expected[key])
            for text, wanted, result in zip(inputs, expected[key], actual[key]):
                assert wanted == result, (key, text, wanted, result)
        print(f'{len(numbers)} exact float conversions and {len(whole)} integer representations passed')


if __name__ == '__main__':
    main()

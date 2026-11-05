"""Compare Python/native stage arrays, preserving signed zero and every ULP."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path)
    parser.add_argument('native', type=Path)
    args = parser.parse_args()
    failed = 0
    arrays = sorted(args.reference.glob('*.npy'), key=lambda p: (
        0 if p.name == 'seeds.npy' else 1 if p.name.startswith('collision-') else
        2 if p.name == 'effective-seeds.npy' else 3 if p.name.startswith('sampling-') else 4, p.name))
    for path in arrays:
        other = args.native / path.name
        if not other.exists():
            print(path.name, 'MISSING'); failed += 1; continue
        a, b = np.load(path), np.load(other)
        if a.shape != b.shape or a.dtype != b.dtype:
            print(path.name, 'SHAPE/DTYPE', a.shape, a.dtype, b.shape, b.dtype)
            failed += 1; continue
        aa = np.ascontiguousarray(a).view(np.uint8).reshape(-1, a.dtype.itemsize)
        bb = np.ascontiguousarray(b).view(np.uint8).reshape(-1, b.dtype.itemsize)
        differences = np.flatnonzero(np.any(aa != bb, axis=1))
        if not len(differences):
            print(path.name, 'MATCH'); continue
        failed += 1
        print(path.name, 'MISMATCH', len(differences), 'elements')
        for index in differences[:5]:
            coordinate = np.unravel_index(index, a.shape)
            print(' ', coordinate, repr(a[coordinate]), repr(b[coordinate]),
                  aa[index].tobytes().hex(), bb[index].tobytes().hex())
    phases = ('rectangles', 'connections', 'jump_marking', 'merge', 'overhang_splits',
              'square_up', 'stairs', 'jump_stitching', 'fence_tops', 'fence_overlap_cleanup',
              'corner_helpers', 'sdk_connection_fixes', 'connection_validation')
    for filename in (*(f'areas-{phase}.json' for phase in phases), 'sdk.nav', 'sdk-nav.json',
                     'cleanup.json', 'link-audit.json', 'merged-nav.json', 'nav.json'):
        left, right = args.reference / filename, args.native / filename
        if not left.exists():
            continue
        same = right.exists() and (left.read_bytes() == right.read_bytes() if filename.endswith('.nav')
                                   else json.loads(left.read_text()) == json.loads(right.read_text()))
        print(filename, 'MATCH' if same else 'MISMATCH'); failed += not same
    return int(failed != 0)


if __name__ == '__main__':
    raise SystemExit(main())

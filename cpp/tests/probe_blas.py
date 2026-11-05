"""Measure the flat (N,3)@(3,2) projection and BLAS signed-zero behaviour.

Set OPENBLAS_CORETYPE=SkylakeX before starting Python. Fraction supplies a
portable, correctly rounded finite FMA oracle on Python 3.10 and Windows.
"""
from __future__ import annotations
import argparse
from fractions import Fraction
import json
import os
from pathlib import Path
import platform

import numpy as np
import numba
import scipy


def fma(a, b, c):
    return float(Fraction(float(a)) * Fraction(float(b)) + Fraction(float(c)))


def dot3(a, b):
    return fma(a[2], b[2], fma(a[1], b[1], float(a[0]) * float(b[0]))) + 0.


def gemv3(a, b):
    return fma(a[2], b[2], fma(a[0], b[0], float(a[1]) * float(b[1]))) + 0.


@numba.njit(cache=True)
def numba_dot(a, b):
    return np.dot(a, b)


@numba.njit(cache=True)
def numba_gemv(a, b):
    return a @ b


def bit_strings(values):
    a = np.asarray(values, dtype=np.float64)
    return [f"{int(v):016x}" for v in a.reshape(-1).view(np.uint64)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--trials", type=int, default=40)
    parser.add_argument("--rows", default="1,2,3,4,5,7,8,9,16,33,100,1000")
    args = parser.parse_args()
    rng = np.random.default_rng(9)
    rows = [int(value) for value in args.rows.split(",")]
    projection = []
    for count in rows:
        mismatches = {"dot3": 0, "gemv3": 0}
        for _ in range(args.trials):
            points = rng.normal(size=(count, 3))
            basis = rng.normal(size=(3, 3))
            actual = (points @ basis[:2].T).view(np.uint64)
            for name, chain in (("dot3", dot3), ("gemv3", gemv3)):
                expected = np.array([[chain(point, axis) for axis in basis[:2]] for point in points])
                mismatches[name] += int(np.count_nonzero(actual != expected.view(np.uint64)))
        item = {"rows": count, "elements": args.trials * count * 2, "mismatches": mismatches}
        projection.append(item)
        print(json.dumps(item), flush=True)
    zeros = []
    vector = np.array([-1., -0., -0.])
    for count in (1, 2, 3, 4):
        points = np.zeros((count, 3))
        item = {"rows": count, "numpy_gemv": bit_strings(points @ vector),
                "numba_gemv": bit_strings(numba_gemv(points, vector)),
                "numpy_dot": bit_strings(np.dot(points[0], vector)),
                "numba_dot": bit_strings(numba_dot(points[0], vector)),
                "numpy_projection": bit_strings(points @ np.array([vector, vector]).T)}
        zeros.append(item)
        print(json.dumps(item), flush=True)
    report = {"platform": platform.platform(), "python": platform.python_version(),
              "numpy": np.__version__, "scipy": scipy.__version__, "numba": numba.__version__,
              "OPENBLAS_CORETYPE": os.environ.get("OPENBLAS_CORETYPE"),
              "projection": projection, "signed_zero": zeros,
              "flat_projection_note": "Rank-two projection requires at least three points; N=1 is diagnostic only."}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    relevant = [item for item in projection if item["rows"] >= 3]
    all_positive_zero = all(bit == "0000000000000000" for item in zeros
                            for key, values in item.items() if key != "rows" for bit in values)
    return 0 if all(item["mismatches"]["dot3"] == 0 for item in relevant) and all_positive_zero else 1


if __name__ == "__main__":
    raise SystemExit(main())

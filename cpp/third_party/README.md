# Native dependencies

These sources are vendored and linked statically. Building or running the native
generator does not download dependencies, import Python, or load a Python/BLAS
runtime. Each directory contains a `SHA256SUMS` manifest of the vendored files,
including its licence and provenance notes.

## Qhull

`qhull/` contains the unmodified re-entrant **Qhull 2019.1** sources shipped by
SciPy **1.15.3**, rather than a newer standalone Qhull release. SciPy's own
`README.scipy` records that version.

- Source: [SciPy's Qhull directory at v1.15.3](https://github.com/scipy/scipy/tree/v1.15.3/scipy/spatial/qhull_src).
- SciPy commit: `e29dcb65a2040f04819b426a04b60d44a8f69c04`.
- Licence: [`qhull/COPYING.txt`](qhull/COPYING.txt).
- File digests: [`qhull/SHA256SUMS`](qhull/SHA256SUMS).

The port uses the same `i Qt` options, explicit triangulation, facet-list order,
facet vertex order and 2-D boundary walk as SciPy's `ConvexHull` wrapper. The
orientation swap in SciPy's `_qhull.pyx` is guarded by `_is_delaunay`; it is not
applied to ordinary 3-D convex hulls. SciPy's `qh_new_qhull_scipy` modification
adds a half-space feasible-point parameter, which these convex-hull calls do not
use. The vendored `README.scipy` retains the original explanation and patch.

Qhull is compiled with the same floating-point contraction policy as the port.
No generated facet ordering, planes or bevels are sorted to conceal a difference.

## zlib

`zlib/` contains sources from **zlib 1.3.1**. The native build links the inflate,
Adler-32 and CRC-32 support used by ZIP and GMA reading.

- Source archive: [zlib-1.3.1.tar.gz](https://zlib.net/fossils/zlib-1.3.1.tar.gz).
- Archive SHA-256: `9a93b2b7dfdac77ceba5a558a580e74667dd6fede4585b91eefb60f03b72df23`.
- Licence: [`zlib/LICENSE`](zlib/LICENSE).
- File digests: [`zlib/SHA256SUMS`](zlib/SHA256SUMS).

The reader handles stored and raw-deflate ZIP entries and validates the
uncompressed size and CRC. Empty compressed entries are included in the
differential tests.

## LZMA decoder

`lzma/` contains `LzmaDec.c` and its four required headers from **7-Zip 24.09**.
Their original file headers place this code in the public domain.

- Source: [7-Zip's C directory at 24.09](https://github.com/ip7z/7zip/tree/24.09/C).
- Commit: `e5431fa6f5505e385c6f9367260717e9c47dc2ee`.
- Licence notes: [`lzma/LICENSE.txt`](lzma/LICENSE.txt).
- File digests: [`lzma/SHA256SUMS`](lzma/SHA256SUMS).

The decoder is used for raw LZMA1 BSP lumps and method-14 ZIP entries. It reads
the stream's five-byte properties and checks the expected output length. No xz
executable or shared library is required.

## Unicode text rules

`python/unicodetype_db.h` is copied verbatim from **CPython 3.10.11**, whose
Unicode database is **13.0.0**. The small native helpers in `gma.cpp` follow
`Objects/unicodectype.c` and the lowercase/final-sigma logic in
`Objects/unicodeobject.c` from that release. The CP437 mapping follows the same
release's `Lib/encodings/cp437.py`.

- Source: [CPython 3.10.11](https://github.com/python/cpython/tree/v3.10.11).
- Commit: `7d4cc5aa854fdea4d01a5b9c59e37c6e62f1103d`.
- Licences: [`python/LICENSE`](python/LICENSE) and
  [`python/UNICODE_LICENSE`](python/UNICODE_LICENSE). The latter is the Unicode
  copyright holder's [published data licence](https://www.unicode.org/license.txt).
- File digests: [`python/SHA256SUMS`](python/SHA256SUMS).

These are data and small native routines, not an embedded Python runtime. They
preserve Python's full lowercase expansion, contextual Greek sigma, decimal
digits, whitespace and line breaks. BSP strings are decoded as Latin-1; ZIP
names use UTF-8 when its flag is set and CP437 otherwise; GMA and VPK names use
UTF-8. Filesystem paths are converted explicitly between UTF-8 and the native
path representation. Directory ordering follows the reference on each host:
original path components on POSIX, lowercased components on Windows. This
operating-system difference is inherited from Python's `pathlib` behaviour.

The local reference environment uses Python 3.12. Tests of Unicode behaviour use
characters already present in Unicode 13. The native tables deliberately remain
at the requested Python 3.10.11 version; characters added in later Unicode
versions must not be used to redefine its behaviour.

## JSON and Python set semantics

`json/` contains **nlohmann/json 3.11.3** headers from the
[upstream v3.11.3 release](https://github.com/nlohmann/json/tree/v3.11.3), under
[`json/LICENSE.MIT`](json/LICENSE.MIT). The file digests are recorded in
[`json/SHA256SUMS`](json/SHA256SUMS). This is the only JSON implementation linked
into the generator.

`src/pyset.hpp` adapts CPython **3.10.11** integer and tuple hashing, set probing,
resize and iteration rules to preserve the reference's observable order. The
upstream sources are `Objects/setobject.c`, `Objects/tupleobject.c` and
`Objects/longobject.c` at the CPython commit recorded above; the licence is
[`python/LICENSE`](python/LICENSE). This helper is native C++ and has no Python
runtime dependency.

## Reproducing the checks

From the repository root, using the pinned Python reference environment and the
native module probe built by CMake:

```sh
OPENBLAS_CORETYPE=SkylakeX python cpp/tests/collision_geometry_differential.py --probe cpp/build/navgen_module_probe
OPENBLAS_CORETYPE=SkylakeX python cpp/tests/collision_bsp_differential.py --probe cpp/build/navgen_module_probe
OPENBLAS_CORETYPE=SkylakeX python cpp/tests/collision_content_differential.py --probe cpp/build/navgen_module_probe
OPENBLAS_CORETYPE=SkylakeX python cpp/tests/collision_text_differential.py --probe cpp/build/navgen_module_probe
```

The scripts generate temporary synthetic input and accept `--report` for JSON
results. The BSP suite can additionally check supplied PHY files via its fixture
option. They do not modify Python generator behaviour or write fixtures into the
repository. Windows can run the same scripts after setting
`$env:OPENBLAS_CORETYPE = "SkylakeX"` in PowerShell and supplying the executable's
build path.

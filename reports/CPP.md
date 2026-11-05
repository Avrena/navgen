# Native generator port

The `cpp-port` branch adds a C++20 implementation of the offline BSP-to-NAV generator, with the Python implementation retained as its reference. The native executable covers both profiles, entity seed discovery, ladders, seed files, both missing-PHY policies, content archives, and the full collision, sampling, area-cleanup and hiding pipeline. The seed-helper CLI, comparison/audit programs, A* path search and in-game route harness remain outside this port.

The starting revision is `68e6b1c`. Commit `bfbdc5a` contains the sole generator Python change: deterministic bases for rank-deficient convex pieces. Development observers live under `tools/`; native code, vendored sources and differential tests live under `cpp/`. Fixture game assets are external and are not part of the branch.

## Qualification environment and acceptance

Testing used Linux x86-64, GCC 13.3.0, glibc 2.39, CMake 4.4.4, and nine available logical processors on an AVX-512/FMA-capable AMD EPYC processor. The local reference interpreter was CPython 3.12.14 with NumPy 2.2.6, SciPy 1.15.3, Numba 0.63.1 and llvmlite 0.46.0. Every reference invocation set `OPENBLAS_CORETYPE=SkylakeX`. The independent acceptance hashes remain the supplied Windows CPython 3.10.11 hashes; no alternative Linux expected file was substituted.

Before editing Python, all eleven supplied cases matched the Windows hashes. The supplied BLAS probes also reproduced the measured dot, GEMM, GEMV and norm reduction orders. All eleven cases still matched after Stage 0.

All **33/33** final native invocations matched both the Windows NAV hash and all required semantic JSON fields. The final executable was checked at `--threads 1`, `2` and the local maximum, `9`.

| Case | Python Windows SHA-256 | C++ SHA-256 at 1 / 2 / 9 threads | 1 | 2 | 9 |
| --- | --- | --- | --- | --- | --- |
| construct-sdk | `7758c8a4c2baf0c1eda7e4b590dc5f02b80cebb1f97d9250fcfa09e9dd5afc88` | `7758c8a4c2baf0c1eda7e4b590dc5f02b80cebb1f97d9250fcfa09e9dd5afc88` | MATCH | MATCH | MATCH |
| construct-sdk-055 | `a95f71b33b49468b6ee1f598621b902ed1a6752f9bee50a8389b9dcc7e5b5e2d` | `a95f71b33b49468b6ee1f598621b902ed1a6752f9bee50a8389b9dcc7e5b5e2d` | MATCH | MATCH | MATCH |
| construct-sdk-055-t1 | `a95f71b33b49468b6ee1f598621b902ed1a6752f9bee50a8389b9dcc7e5b5e2d` | `a95f71b33b49468b6ee1f598621b902ed1a6752f9bee50a8389b9dcc7e5b5e2d` | MATCH | MATCH | MATCH |
| construct-sdk-bbox | `7758c8a4c2baf0c1eda7e4b590dc5f02b80cebb1f97d9250fcfa09e9dd5afc88` | `7758c8a4c2baf0c1eda7e4b590dc5f02b80cebb1f97d9250fcfa09e9dd5afc88` | MATCH | MATCH | MATCH |
| construct-merged-055 | `adb80cb082e355cd7673008bfa4b9fa18d48864a05284f1099c110dd86b81c9b` | `adb80cb082e355cd7673008bfa4b9fa18d48864a05284f1099c110dd86b81c9b` | MATCH | MATCH | MATCH |
| construct-merged-055-entities | `679e59dbd01a4efc06791ba0a54bbfe55290f5624cc9b55c535ae6b903c3a8d6` | `679e59dbd01a4efc06791ba0a54bbfe55290f5624cc9b55c535ae6b903c3a8d6` | MATCH | MATCH | MATCH |
| construct-sdk-055-entities-ladders | `bbbff0a92410c09cbc7f192d991dff3a85b8e610cd95cbd3b2f4a6291627c10c` | `bbbff0a92410c09cbc7f192d991dff3a85b8e610cd95cbd3b2f4a6291627c10c` | MATCH | MATCH | MATCH |
| construct-merged-055-entities-ladders | `679e59dbd01a4efc06791ba0a54bbfe55290f5624cc9b55c535ae6b903c3a8d6` | `679e59dbd01a4efc06791ba0a54bbfe55290f5624cc9b55c535ae6b903c3a8d6` | MATCH | MATCH | MATCH |
| construct-merged-055-seeds | `9178d31c67fb1a4bb439803e4eb6277c59baec5460008cf873234fa65df65123` | `9178d31c67fb1a4bb439803e4eb6277c59baec5460008cf873234fa65df65123` | MATCH | MATCH | MATCH |
| flatgrass-sdk | `11c6d765ec8c661fb3f285844d421d598eda9a25dcde25cf677e4c7c7c254770` | `11c6d765ec8c661fb3f285844d421d598eda9a25dcde25cf677e4c7c7c254770` | MATCH | MATCH | MATCH |
| flatgrass-merged-055 | `11c6d765ec8c661fb3f285844d421d598eda9a25dcde25cf677e4c7c7c254770` | `11c6d765ec8c661fb3f285844d421d598eda9a25dcde25cf677e4c7c7c254770` | MATCH | MATCH | MATCH |

The acceptance driver compares the complete required result structure: counts and sizes, input/output hashes, profile and policy, cleanup, hiding, ladder and discovery details, link audits, collision counters, merge details, unresolved content, content audit, and sampling speculation. It excludes only the contract's diagnostic fields and the nested collision/merge elapsed times. It checks the set of timing-phase keys separately. Each case's embedded thread argument is overridden for the matrix, including the case whose name ends in `-t1`.

The two supplied engine NAVs also round-trip byte for byte through the native reader and writer. Their SHA-256 values are:

| Engine input | Areas | Read/write SHA-256 |
| --- | ---: | --- |
| gm_construct | 2,271 | `e839c5ef26dfaaa852d8c23b7b434515f975021aa36be9f7371bda70f08f76b8` |
| gm_flatgrass | 853 | `545d8873b5444b5cba378a063274b4e1089653ccb3f5bb4eb672e91aea8166a4` |

These files validate NAV binary handling; they are not expected outputs of offline generation.

## Stage 0: deterministic flat pieces

The original rank-deficient path selected its basis from LAPACK SVD output. That basis can vary in sign and in the orientation of its null space, affecting projected hull order and the eventual bevel planes. The change retains `matrix_rank(..., tol=1e-7)` in Python and changes only basis construction:

1. Rank two selects the largest squared cross product of centred point pairs, preserving the first candidate on equal scores.
2. Rank one selects the longest centred direction, again preserving the first tie.
3. The largest absolute component determines the canonical sign. The least-aligned coordinate axis supplies the first perpendicular through a cross product; a second cross product completes the frame.
4. Rank zero uses the coordinate frame. Projection order, the two-dimensional Qhull call, edge handling and plane construction remain unchanged.

Python and C++ use the same sequence of ordinary operations and explicit BLAS-equivalent dot chains. The separate Stage 0 tests include planar, linear and point inputs and compare plane dtype, shape and every byte when the native probe is supplied. The wider geometry differential suite checks additional flat orientations, duplicate points and signed zeros. All eleven fixture hashes were rechecked after the change; none changed. The effect on the private maps' flat pieces still needs measurement on those maps.

## Numeric and order audit

### Arithmetic, reductions and precision

| Reference site | Native reproduction |
| --- | --- |
| Ordinary Python, NumPy elementwise and Numba arithmetic | Separate multiplies/additions, with GCC/Clang `-ffp-contract=off -fno-fast-math` or MSVC `/fp:precise`; no contraction/fast-math option is enabled. The same flags apply to Qhull. |
| Three-element dot, `(N,3) @ (3,3)` and transformed vertices in collision, discovery and merge environments | `dot3`: `fma(a2,b2,fma(a1,b1,a0*b0))`. Each matrix result is computed in this order before translation. |
| Plane-offset translation in collision, brush inside tests, axial planes and bevel distance tests | `gemv3`: `fma(a2,b2,fma(a0,b0,a1*b1))`. The NumPy single-row exception uses `dot3`; Numba GEMV retains `gemv3` even for one row. |
| Flat-piece squared lengths and `(N,3) @ (3,2)` projection | Explicit dot chain; the additional projection shape was measured locally with SkylakeX for sizes 2 through 1,000. Rank-two projection has at least three points. The single-row shape differs, but is not used by this path. The extra shape still needs the Windows probe described below. |
| Triangle normals, bevel normals and plane distances | Cross products use ordinary arithmetic; vector lengths and scalar plane dots use the explicit dot chain. Candidate order, rejection thresholds and first surviving normals follow Python. |
| Sampling rectangle coplanarity | `dot3` reproduces Numba's three-vector `np.dot`; the difference and the five-unit comparison retain their original precision and order. |
| Area surface normals, SDK area-merge normal tests, stair normal tests and conservative post-merge tests | One-dimensional norms use `sqrt(dot3(...))`; each surface-normal comparison uses `dot3`. |
| Portal distances, discovery proximity and ladder candidate/attachment costs | Two-dimensional norms use `sqrt(fma(x1,x1,x0*x0))`; three-dimensional norms use the three-term dot chain. Sums of candidate costs retain the reference order. |
| BSP tree traversal for discovery and merge water state | The split-plane test uses the three-vector dot chain and the same strict comparison. |
| Axis-based norms: displacement corner selection and area corner/node selection | Plain left-associated squared sums and `sqrt`, without BLAS fusion. First equal-distance candidate wins. |
| Small `np.sum` reductions in sampling, stair tracing and corner checks | The original two- or three-term order and dtype are explicit. Hiding distances retain float32 accumulation. The reachable generator has no floating reduction of eight or more elements requiring NumPy's eight-accumulator pairwise sum. Large boolean/count sums are integer reductions. The path-length reduction in the unported A* tool is out of scope. |
| BLAS signed zero | Helpers add the zero accumulator after the explicit chain, reproducing positive zero for an all-negative-zero reduction. This was necessary for bitwise synthetic flat-piece parity. |
| Displacement vertices | On-disk coordinates are float32; the displacement vector/distance product rounds in float32 before being combined with double-precision interpolated corners. Side winding, diagonal choice and triangle tags are preserved. |
| Hiding and native portal geometry | NAV-facing positions, angle increments, cover-ray endpoints and native portal height classes retain their float32 casts. General collision, sampling positions and planes remain float64. |
| NAV output | Explicit little-endian integer fields and IEEE float32 conversion. Range and list-length overflow fail rather than silently narrowing; opaque place strings and trailing bytes are retained. |

Representative disassembly was inspected: explicit dot/GEMV sites contain hardware FMA instructions, while collision clipping and rotation arithmetic do not acquire fused operations. `std::fma` is the only intentional fusion mechanism. The default x86 build enables `-mfma` or `/arch:AVX2`; `NAVGEN_HARDWARE_FMA=OFF` retains the explicit correctly rounded calls for a slower runtime implementation.

The rank decision uses a one-sided Jacobi SVD with the same absolute `1e-7` singular-value threshold, rather than forming a squared-condition-number Gram matrix. Its intermediates are double precision on both GCC and MSVC. Brush reconstruction uses three-by-three LU with partial pivoting, a determinant threshold of `1e-9`, then six-decimal rounding and lexicographic unique points. No rank or rounded-vertex mismatch was found in the tested synthetic inputs. Singular values exactly near the threshold and solutions exactly near a decimal rounding boundary remain residual risks.

Rounding uses the round-to-nearest-even environment, `nearbyint`, and `rint(x*scale)/scale`-equivalent decimal rounding. Node keys and spatial bins preserve the distinction between `round`, truncation and `floor`, including negative coordinates. Python floor division/modulo helpers are provided where required. Integer/tuple hashes use defined unsigned wraparound, without compiler extensions. Entity/CLI numeric parsing accepts Python decimal syntax, underscores, Unicode decimal digits and whitespace; it rejects C hexadecimal float syntax. Integer consumers retain exact low-bit masks, zero tests and small equality comparisons even for values exceeding 64 bits, while bounded indices and thread counts fail through their range checks.

Rotations call `sin` and `cos` with the same double arguments and degree conversion. Hiding steps its angle in float32 and calls the double transcendental functions at the same points. Stair tests retain `tan(acos(slope_limit))`. No replacement math library is used. The local libm produced the supplied Windows fixture hashes and all tested synthetic outputs; rare glibc/UCRT last-bit differences on untested angles remain possible.

### Hulls, containers and traversal

SciPy 1.15.3 vendors Qhull **2019.1 / 7.3.2**. The matching re-entrant sources are included, with `qhull i Qt`, the same triangulation and facet iteration, direct simplex vertex order for ConvexHull, and the same two-dimensional vertex walk. SciPy's orientation swap guarded by its Delaunay branch is not applied to ConvexHull. Plane deduplication compares six-decimal keys while retaining the first original plane and its original bits.

| Order-sensitive site | Native reproduction |
| --- | --- |
| Hull edge-set comprehension and subsequent bevel insertion | CPython 3.10 64-bit integer/xxHash tuple hashes and its set-table probing, perturbation, growth, tombstones and iteration order. Edges are inserted in simplex row/edge order, never sorted for bevel generation. |
| SDK graph links/incoming edges and conservative merge sets | The same set implementation, including copy, update, union, intersection and difference. Self-union follows Python's copy-without-update path, avoiding an otherwise observable rehash. |
| Entity parsing and lowercased entity-key dictionaries | Insertion-ordered key/value storage; duplicate keys update their original position. Lowercasing traverses that order, including differently cased keys that collide after normalisation. |
| Area dictionaries and graph snapshots | IDs are assigned monotonically; ordered ID maps therefore preserve the live Python insertion order after deletions and newly appended IDs. Snapshot processing preserves that order. |
| Graph queues and explicitly sorted links, model names, missing-content summaries and unique brush points | The corresponding lexicographic sorts are retained. No unordered-map iteration feeds output order. |
| BVH split axis and child object order | First maximum-extent axis; stable centre sort; original midpoint and leaf threshold. |
| `argmin`/`argmax` choices in displacement rotation, ladder axes, canonical flat bases, rectangle squaring and corner selection | Strict comparisons retain the first equal candidate. |
| Sorts with ties | Stable sorting for BVH centres, effective seeds and ladder candidates. Ladder option selection preserves Python's tuple prefix and first tie. Relevant generator `argsort` calls explicitly request stability; unstable sorts in comparison tools are outside scope. |
| Spatial buckets, node lookup and caches | Hash tables are lookup-only. Per-bucket vectors, chain links and inserted records retain reference order; sorted or ID-ordered traversal is used where the reference demands it. |
| Content source selection | BSP pak, loose game/addon files, GMA/Workshop and VPK precedence, conflict records and archive diagnostics follow the reference. `Path` ordering follows the platform: component-wise case-sensitive order on Linux and lowercased components on Windows. |

Unicode lowercasing uses the CPython 3.10.11 Unicode 13 table, including multi-character lowercase expansion and final sigma. BSP text follows Latin-1 decoding; ZIP names follow the UTF-8 flag or CP437; GMA decoding preserves the reference error behaviour. UTF-8 filesystem helpers and a wide Windows entry point preserve non-ASCII input/output paths. Third-party origins and licences are documented in `cpp/third_party/README.md`.

The VPK reader deliberately retains `vpk` 1.4.0's lack of per-file CRC validation. Synthetic VPKs with incorrect CRC metadata are read by both implementations; changing that behaviour would change the reference's accepted input. ZIP and GMA CRC failures are rejected. Other preserved quirks include the SDK fence-height assignments and incoming-link split gates, retained unverified SDK connection exceptions, and conservative handling of unknown water state during merging.

## Parallelism and failure handling

The sampling DFS remains serial. Parallel stages fill independent per-item results; graph edits and rectangle claims commit in reference order. One-thread rectangle claiming and multi-thread speculative candidates retain their different trace-count behaviour. The four corner flags in rectangle testing are read through a packed mask without changing floating-point work or candidate order. Trace counters use padded per-worker slots and are totalled after joins.

Thread construction failures join workers already launched before propagating. Exceptions inside worker bodies are collected and rethrown after joining, so a failed launch or allocation follows the normal failure path instead of destroying joinable threads. A Linux fault-injection test exercises later `pthread_create` failures in the generic, sampling and area launchers. No speed claim is attributed to the counter-layout change in isolation.

The CLI validates options, reports generation failures with status one and the required prefix, and reports usage failures with status two. The result JSON has sorted keys and retains the required phase names. `build_identifier` is derived from native source files and the CMake configuration file. The executable runs without Python in `PATH`; all archive and geometry dependencies are linked into it. Windows uses the static CRT.

Output is first validated, serialised and round-tripped. A temporary file is exclusively created in the destination directory and then renamed/replaced. Existing output remains intact on failure, temporary files are cleaned up, and resolved BSP/seed paths cannot be overwritten. The Windows implementation preserves wide filenames through temporary-name construction and replacement.

## Differential verification

| Gate | Coverage | Result |
| --- | --- | --- |
| Python reference suite | 79 tests, including the added flat-piece tests; the local-game-install `test_navio` case excluded | PASS |
| Native CTest | Core numeric/set/NAV checks and sampling/area graph checks, including six worker-body exception cases | 2/2 PASS |
| End-to-end fixtures | Eleven cases at one, two and nine threads; Windows NAV hashes, semantic JSON and phase keys | 33/33 MATCH |
| Full merged-stage comparison | 33 arrays, thirteen area graphs and six final/intermediate NAV/audit artefacts; entity discovery and ladders enabled, slope 0.55 | 52/52 MATCH |
| Geometry | 122 convex cases across all ranks, 73 rotations, 40 bevels, 68 triangles, 27 trace scenes containing 912 rays, and 24 brush reconstructions | 354/354 MATCH |
| BSP and PHY | 31 packed BSP snapshots, 34 complete collision scenes, six synthetic PHYs and nine supplied PHY files | 80/80 MATCH |
| Archives and content | Stored/deflate/LZMA and empty ZIP entries, UTF-8/CP437/ZIP64, GMA 1–3, VPK 1/2 with embedded/preload/split data, package-generated VPK, Workshop and precedence/conflict/error audits | 34/34 MATCH |
| Text parsing | Latin-1, CP437, Unicode lowercasing/expansion/final sigma, whitespace, malformed UTF-8 and duplicate-case entity keys | 31/31 MATCH |
| Sampling and areas | Six synthetic scenes at one, two and nine threads, including empty, step, wall, fence, overhang and low-room geometry | 18/18 MATCH |
| Optional modules | Nine groups covering geometry, hiding, discovery, written and blocked ladders, and merging, repeated at one and nine threads | All PASS |
| Merge environment | Four groups covering brush restrictions, water states and locked areas | All PASS |
| Runtime, ordering and NAV I/O | Forty deterministic sequences of 25 set operations, opaque NAV fields and range errors, both engine NAVs, CLI validation, sidecars, input protection, non-ASCII/spaced paths and execution without Python in `PATH` | 65/65 PASS |
| CLI and numeric parsing | 266 option/number boundary cases; 328 exact double conversions; 59 arbitrary-size integer mask/truth/equality/range cases | All PASS |
| Worker construction failure | Failed second thread creation in sampling, area and generic helpers; already-running work joined | 3/3 PASS |
| Extended BLAS probe | Flat `(N,3) @ (3,2)` projection for sizes 1–1,000, 40 trials per size, plus NumPy/Numba signed-zero cases | Relevant chains MATCH |

The BSP suite includes powers two through four, both displacement sides, corner rotations, folds, tags, float32 displacement products, LZMA lumps, 32- and 56-byte leaves, static-prop versions 4/10/11, missing MDL/PHY handling, both missing-PHY policies, bounding-box and unsupported solid types, translated orientation-cache reuse, and flat/linear/point physics ledges. Ladder tests write reciprocal records and test blocked climbs; they do not merely exercise construct's skipped ladder.

The parser comparisons were also run using CPython 3.10.11's `argparse.py`. The three help/ambiguity cases that differ in the local Python 3.12 implementation are pinned to the verified 3.10 behaviour in the retained test. Unicode fixtures use characters present in the target Unicode 13 database.

`tools/dump_stages.py` observes the Python stages without changing their inputs or results. Native `--dump-stages` emits the same arrays and graphs. `tools/compare_stages.py` compares dtype, shape and every element byte, including signed zero, followed by each area pass and the serialised NAV. The native module probe allows smaller failing inputs to be reproduced independently. These tools are development dependencies only.

## Same-machine performance and memory

The measured workload was `gm_construct`, SDK profile, slope limit 0.7, at nine threads on the same Linux allocation. Three fresh CLI invocations of each implementation were alternated with no concurrent build or differential-test jobs. The Python Numba cache was warm before measurement. Each invocation still imports its runtime and loads cached specialisations; this measures the compile-step CLI workflow, rather than repeated calls inside an already-running Python process. Peak resident memory comes from each child process's Linux `wait4` result.

| Metric, median of three | Python | C++ | Comparison |
| --- | ---: | ---: | --- |
| CLI wall time | 12.905 s | 3.853 s | **3.35× faster** |
| Reported generator total | 12.257 s | 3.669 s | 3.34× faster |
| Peak resident memory | 405.9 MiB | 173.9 MiB | **57.1% lower** |
| CLI wall range | 12.771–13.988 s | 3.741–3.975 s | All repetitions retained |

The three paired repetitions were:

| Repetition | Python wall (s) | C++ wall (s) | Python peak RSS (KiB) | C++ peak RSS (KiB) |
| --- | ---: | ---: | ---: | ---: |
| 1 | 13.988 | 3.741 | 412,148 | 178,124 |
| 2 | 12.771 | 3.853 | 415,600 | 178,048 |
| 3 | 12.905 | 3.975 | 416,116 | 178,212 |

Every repetition produced `7758c8a4c2baf0c1eda7e4b590dc5f02b80cebb1f97d9250fcfa09e9dd5afc88`, the supplied Windows hash. The phase medians follow; ratios use the unrounded measurements. Individual phase medians need not sum exactly to the median total, and very short phase ratios are sensitive to process-startup and cached-kernel loading costs.

| Phase key | Python (s) | C++ (s) | Python / C++ |
| --- | ---: | ---: | ---: |
| `input` | 0.029573 | 0.038201 | 0.77× |
| `collision` | 0.940740 | 0.053606 | 17.55× |
| `sampling` | 4.373108 | 2.237240 | 1.95× |
| `rectangles` | 2.440654 | 0.973985 | 2.51× |
| `connections` | 0.409396 | 0.082804 | 4.94× |
| `jump_marking` | 0.319099 | 0.000592 | 539.12× |
| `merge` | 1.613232 | 0.112471 | 14.34× |
| `overhang_splits` | 0.019926 | 0.003036 | 6.56× |
| `square_up` | 0.135290 | 0.019409 | 6.97× |
| `stairs` | 0.087569 | 0.002506 | 34.94× |
| `jump_stitching` | 0.005173 | 0.001294 | 4.00× |
| `fence_tops` | 1.385928 | 0.053728 | 25.80× |
| `fence_overlap_cleanup` | 0.000479 | 0.000060 | 8.01× |
| `corner_helpers` | 0.141464 | 0.005283 | 26.78× |
| `sdk_connection_fixes` | 0.010861 | 0.004884 | 2.22× |
| `connection_validation` | 0.088561 | 0.020155 | 4.39× |
| `hiding_spots` | 0.064217 | 0.008751 | 7.34× |
| `write_validate` | 0.204837 | 0.025784 | 7.94× |
| `total` | 12.256650 | 3.669163 | 3.34× |

The native serial sampling phase remains the largest cost, followed by rectangles; together they account for about 88% of the native generator total in this workload. Collision building, graph edits and fence handling improve substantially. The threefold goal is met for this stock-map measurement. These results do not predict the private maps' speedup or establish the 15-second production target on the separate 16-thread Windows machine.

The retained `cpp/tests/benchmark.py` driver records every invocation's phase timings, wall time, peak RSS and NAV hash before calculating medians. It preserves virtual-environment interpreter paths instead of resolving their symlinks to a different Python environment.


## Remaining qualification and residual risks

There is no known NAV or required-result divergence in the supplied fixtures or retained differential suites. That evidence does not establish parity for every unseen BSP. The following work remains:

* Build and run the branch with MSVC 2022 x64. Windows is the production gate; no Windows compiler or runtime execution was available in this qualification. The source was reviewed for Windows CRT, wide paths, threading and floating-point flags, but that review cannot replace the build and run.
* Run the eleven-case matrix on Windows at one, two and the available maximum thread count. Repeat `python cpp/tests/probe_blas.py --report BLAS_RESULTS.json` against the target OpenBLAS build with `OPENBLAS_CORETYPE=SkylakeX`.
* Run both large production BSPs, with their production seeds/options, and compare stage dumps if a hash differs. Their larger object counts, area counts and orientations were not available here. Measure the Stage 0 effect on their flat pieces and repeat the in-game route check if their NAV bytes change.
* Watch rank decisions near `1e-7`, LU determinant decisions near `1e-9`, six-decimal brush-vertex rounding boundaries, and rare cross-platform transcendental differences. No failing example was found in the exercised cases, so Python behaviour was not changed to hide a discrepancy.
* Measure the performance goal on the large maps and a 16-thread production machine. Stock-map results here cannot establish the large-map target.

Written ladders, `side=1` displacements, missing PHYs, archive sources, compressed BSP/ZIP data and varied prop rotations are covered by synthetic differential tests. They still need integration coverage with real maps and content. The optional seed-helper CLI was not ported; existing seed files remain accepted by the generator.

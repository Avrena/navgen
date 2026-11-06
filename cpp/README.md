# Native navgen

This C++20 implementation generates Source/Garry's Mod NAV v16, subversion 0 files directly from a compiled BSP. It preserves the Python generator's processing order and numerical behaviour. Python is needed only for the reference and differential tests; the executable does not start Python, download assets, or require a game process.

Inputs include BSP v20 and L4D2 BSP v21. The reader detects standard and L4D2 v21 lump-directory layouts, validates every lump range and normalises the directory before loading geometry. This is BSP input support for Garry's Mod NAV generation; the output does not contain L4D2-specific NAV records or Director analysis. See the main README for model content and seed requirements.

The production target is MSVC x64 (Visual Studio 2022 or 2026). GCC on Linux is also supported. The dependencies are vendored, including the exact Qhull sources from SciPy 1.15.3. No dependency is downloaded during configuration or at runtime. See `third_party/README.md` for versions, source provenance and licences, and `../reports/CPP.md` for measured parity, performance and remaining qualification work.

## Build

Use CMake 3.21 or newer and a C++20 compiler. From the repository root, name the installed Visual Studio's generator: `"Visual Studio 18 2026"` (toolset v145) or `"Visual Studio 17 2022"` (v143). A generator without its toolset fails with MSB8020.

```powershell
cmake -S cpp -B cpp/build -G "Visual Studio 18 2026" -A x64
cmake --build cpp/build --config Release --parallel
ctest --test-dir cpp/build -C Release --output-on-failure
```

The executable is `cpp/build/Release/navgen.exe`. The CRT and the vendored libraries are linked statically; no Python or third-party DLLs need to accompany it.

On Linux:

```sh
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build --parallel
ctest --test-dir cpp/build --output-on-failure
```

The executable is `cpp/build/navgen`. The Linux build uses the system C/C++ runtime and statically linked vendored dependencies.

Hardware FMA is enabled by default: GCC/Clang receive `-mfma`, and MSVC receives `/arch:AVX2`. The resulting x86-64 executable requires an AVX2/FMA-capable processor. To build for an older processor, configure with `-DNAVGEN_HARDWARE_FMA=OFF`; explicit `std::fma` calls then use the compiler/runtime's correctly rounded implementation. This can be slower. The reference production processor supports AVX-512, but the native executable does not require AVX-512.

## Usage

```text
navgen <map.bsp> [--game DIR] [--out FILE] [--seeds FILE]
       [--slope-limit F] [--threads N] [--profile sdk|merged]
       [--seed-discovery none|entities] [--ladders] [--missing-phy skip|bbox]
```

The Hammer++ compile-step command is unchanged apart from selecting the native executable:

```text
navgen "$path\$file.bsp" --game "$gamedir" --slope-limit 0.55 --profile merged --seeds <file> --out "$path\$file.nav"
```

Defaults match Python: SDK profile, slope limit `0.7`, all logical cores, no additional entity discovery, no ladders, and missing-PHY policy `skip`. Without `--game`, both implementations locate Garry's Mod through the same Steam library search (see the main README). Threads must be between one and the logical-core count, and the slope limit must be finite and in `(0, 1]`.

The default output replaces the BSP suffix with `.nav`. A `<map>.navseeds.txt` sidecar is discovered automatically; `--seeds` overrides the sidecar choice, and an explicit empty seed file uses BSP seeds alone. Paths containing spaces work when quoted. Generation refuses to overwrite the BSP or the selected seed file. It validates the NAV and its binary read/write round trip before replacing the output through a temporary file in the destination directory.

Exit status is zero on success, one for generation failures, and two for malformed command-line arguments. Generation failures use `navgen failed: <Kind>: <message>` on stderr. Progress goes to stdout, followed by one result-JSON line. `build_identifier` replaces the Python compiled-cache fingerprint and identifies the native source/configuration inputs. Timings and total trace counts remain diagnostic; semantic result fields and NAV bytes are parity gates.

## Floating-point and ordering requirements

Do not enable fast-math, contraction, reassociation or floating-point flags that replace the configured settings. GCC/Clang use `-ffp-contract=off -fno-fast-math`; MSVC uses `/fp:precise`, without `/fp:contract` or `/fp:fast`. The same restrictions apply to Qhull. Ordinary multiplies and additions round separately. Only the named BLAS helpers use explicit `std::fma`.

`dot3` reproduces the reference three-term dot/GEMM chain. `gemv3` reproduces the different row/vector reduction order used by the reference GEMV kernel; NumPy's single-row exception uses `dot3`. Both preserve BLAS's positive-zero result for an all-negative-zero reduction. Float32 hiding positions, angle stepping and native portal height classes remain float32. The normalised plane arrays, node arrays and their order are checked before considering the final NAV hash.

`pyset.hpp` implements CPython 3.10's 64-bit integer/tuple hashes, table probing, resize policy, tombstones, copy and set operations. It is used wherever set iteration can affect a plane, graph edit, or diagnostic example. Sorted traversals and insertion-ordered traversals retain their Python order. Worker loops compute independent results; graph mutations and rectangle claims commit sequentially. SDK sampling remains a serial DFS.

The only Python reference change replaces the SVD basis of rank-deficient convex pieces with deterministic vector construction. Rank-two, rank-one and rank-zero inputs have bitwise differential tests. The report records the MSVC and large-map qualification results.

## Tests and stage comparison

Native CTest tests require no game installation. Differential tests use the repository's pinned Python reference environment. Set `OPENBLAS_CORETYPE=SkylakeX` before starting Python on an AVX-512 machine. Do not force that kernel on a processor which cannot execute it.

The additional flat-projection and signed-zero probe can be rerun on the Windows reference environment:

```sh
python cpp/tests/probe_blas.py --report BLAS_RESULTS.json
```

```sh
python cpp/tests/test_runtime.py --binary cpp/build/navgen --probe cpp/build/navgen_module_probe --fixtures FIXTURES
python cpp/tests/sampling_areas_differential.py --probe cpp/build/navgen_module_probe --threads 1,2,max
python cpp/tests/test_optional.py --probe cpp/build/navgen_module_probe --threads 1 -v
python cpp/tests/test_optional.py --probe cpp/build/navgen_module_probe --threads 9 -v
python cpp/tests/test_optional_environment.py --probe cpp/build/navgen_optional_environment_probe -v
python cpp/tests/collision_geometry_differential.py --probe cpp/build/navgen_module_probe
python cpp/tests/collision_bsp_differential.py --probe cpp/build/navgen_module_probe --fixtures FIXTURES
python cpp/tests/collision_content_differential.py --probe cpp/build/navgen_module_probe
python cpp/tests/collision_text_differential.py --probe cpp/build/navgen_module_probe
python cpp/tests/test_cli_numbers.py --binary cpp/build/navgen --probe cpp/build/navgen_module_probe
```

Use the available logical-core count instead of `9` on another machine. `FIXTURES` is a fixture directory, kept outside the repository. The stock engine NAVs are read/write round-trip inputs, not generator references.

On Linux, the worker-launch failure regression injects a failed second thread creation and checks that each helper joins the worker already running before propagating the error:

```sh
python cpp/tests/test_thread_failure.py --test-binary cpp/build/navgen_sampling_areas_tests
```

For the flat-piece differential test, set `NAVGEN_CPP_PROBE` to the absolute path of `navgen_module_probe` and run:

```sh
python -I -m unittest discover -s tests -p test_flat_convex.py -v
```

Run `FIXTURES/run_cases.py` with the reference interpreter first. Then compare all native cases at explicit worker counts, including the complete required result-JSON fields:

```sh
python FIXTURES/run_cases.py --out PYTHON_RESULTS -- python -I -m navgen
python cpp/tests/acceptance.py --fixtures FIXTURES --reference PYTHON_RESULTS --generator cpp/build/navgen --out NATIVE_RESULTS --threads 1,2,9
```

The native CLI's hidden `--dump-stages DIR` writes the collision arrays, sampling arrays, rectangles, every area cleanup pass, optional merged graph, final NAV records and result JSON. Its Python counterpart observes the reference without changing generation:

```sh
python tools/dump_stages.py MAP.bsp --game GAME --out PYTHON.nav --threads 1 --dump-stages PYTHON_STAGES
cpp/build/navgen MAP.bsp --game GAME --out NATIVE.nav --threads 1 --dump-stages NATIVE_STAGES
python tools/compare_stages.py PYTHON_STAGES NATIVE_STAGES
```

Array comparisons include dtype, shape, element bits and signed zero. The comparison tool reports the first differing elements rather than applying a numeric tolerance. Development dumps and timings do not enter generation decisions.

For a same-machine Linux benchmark, first warm the Python Numba cache by running the acceptance cases, then stop other build/test jobs and run:

```sh
python cpp/tests/benchmark.py --fixtures FIXTURES --python PYTHON_EXECUTABLE --native cpp/build/navgen --out BENCHMARK_RESULTS --threads 9 --repetitions 3
```

This alternates Python and native invocations, records every phase and wall time, and obtains each process's peak resident memory with `wait4`. It saves all repetitions and reports medians. Replace `9` with the local logical-core count and keep the outputs outside the repository.

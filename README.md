# navgen

navgen builds navigation meshes for Garry's Mod maps without starting the game. It reads a compiled BSP and writes the `.nav` file (version 16, subversion 0) that NextBots and other navmesh users load with the map.

It follows the Source SDK 2013 `nav_generate` pipeline and adds the hiding-spot analysis that Garry's Mod runs when it generates a mesh. The native executable handles a large map in seconds. A Python implementation is kept as the reference, and both produce byte-identical files.

## Features

- **Collision as the generator sees it.** World brushes, brush entities in their spawn state, displacements and static props. Prop physics models are read from the BSP pakfile, addon folders, addon and Workshop GMAs, and the game's VPKs.
- **SDK generation.** Node sampling, area building, connections, jump and drop links, stairs, obstacle tops and the SDK's cleanup passes.
- **Hiding spots.** Written the way Garry's Mod writes them.
- **Seeds.** Spawn points and teleport destinations are used automatically. A seed file beside the map adds floors the spawns cannot reach. Optional discovery adds floors under doors, buttons and ladder ends.
- **Merged profile.** An optional pass that combines areas without changing routing. It removes about 8–12% of areas on the maps tested.
- **Deterministic output.** The NAV bytes are the same at any thread count.
- **Safe output.** The NAV is validated and read back before it replaces an existing file.

## Download

Download `navgen-<version>-windows-x64.zip` from the [releases page](https://github.com/Avrena/navgen/releases) and extract it anywhere. It contains `navgen.exe` and the licences.

It needs 64-bit Windows on a processor with AVX2 and FMA, which most desktop processors from the last decade have (some Pentium and Celeron models do not). Python and other runtimes are not needed. On Linux, or for an older CPU, build from source (see below).

## Quick start

```powershell
navgen.exe "C:\path\to\my_map.bsp"
```

This writes `my_map.nav` next to the BSP. Put it in `garrysmod/maps/` beside the map's BSP; Garry's Mod loads it when the map starts.

Without `--game`, navgen finds Garry's Mod through Steam (see [Game content](#game-content)). Progress is printed while it runs, followed by a one-line JSON summary.

## Hammer++ compile step

Generate the mesh after the last step that changes the BSP, normally after VRAD and any packing. The NAV header records the BSP's file size, and the game reports a mesh as built for a different version of the map when the sizes differ.

In Hammer++'s *Run Map* dialog, switch to expert mode and add a command after VRAD:

- **Command:** the full path to `navgen.exe`
- **Parameters:** `"$path\$file.bsp" --game "$gamedir" --out "$path\$file.nav"`

Then add a *Copy File* command with the parameters `"$path\$file.nav" "$bspdir\$file.nav"`, so the mesh is copied with the map. Append any of the options below to the navgen parameters, for example `--slope-limit 0.55 --profile merged`.

## Options

```text
navgen <map.bsp> [--game DIR] [--out FILE] [--seeds FILE]
       [--slope-limit F] [--threads N] [--profile sdk|merged]
       [--seed-discovery none|entities] [--ladders] [--missing-phy skip|bbox]
```

| Option | Default | Meaning |
| --- | --- | --- |
| `--game DIR` | found through Steam | The `garrysmod` directory used to find models and physics files. |
| `--out FILE` | `<map>.nav` beside the BSP | Output path. An existing file is replaced only after the new mesh passes validation. |
| `--seeds FILE` | `<map>.navseeds.txt` if present | Extra seed positions. See [Seeds](#seeds). |
| `--slope-limit F` | `0.7` | Steepest walkable ground, as the minimum surface normal Z (0 < F ≤ 1). `0.7` is the engine's value; lower values such as `0.55` accept steeper slopes. |
| `--threads N` | all logical cores | Worker threads. The result does not depend on this. |
| `--profile sdk\|merged` | `sdk` | `sdk` keeps the SDK's areas. `merged` combines compatible areas afterwards. |
| `--seed-discovery none\|entities` | `none` | `entities` adds checked floors under spawns, teleport exits, doors, buttons and ladder ends. |
| `--ladders` | off | Writes ladder records from `info_ladder` and `func_useableladder`. Off by default because stock NextBots fail paths that use ladder segments. |
| `--missing-phy skip\|bbox` | `skip` | What a physics prop without a PHY file contributes: nothing (`skip`), or its model's bounding box (`bbox`). |

Exit status is 0 on success, 1 if generation fails (`navgen failed: <Kind>: <message>` on stderr) and 2 for invalid arguments.

### Seeds

Generation floods outward from seed positions, so floors that no seed can reach get no areas. Seeds come from every `info_player_start` and every `trigger_teleport` destination. To cover more, put a `<map>.navseeds.txt` next to the BSP, or pass a file with `--seeds`. Each line is `x y z`, and `#` starts a comment. An empty file passed with `--seeds` means "BSP seeds only", even when a sidecar exists. Teleport destinations seed the floor they land on, but no teleport links are written into the mesh.

The Python package includes a helper that carries floors over from an older mesh. It prints one grounded seed for each connected part of the old mesh:

```powershell
python -I -m navgen.seeds 'old_map.nav' --bsp 'my_map.bsp' | Set-Content -LiteralPath 'my_map.navseeds.txt' -Encoding utf8
```

Review the file before use. One old component can span several floors that the new geometry has separated.

### Merged profile

`--profile merged` builds the SDK mesh first, then combines neighbouring areas only when routing stays the same:

- the areas share a whole edge and link to each other;
- their attributes match;
- every original corner stays within 0.5 units of the merged surface;
- no merged side is longer than 125 units.

Areas with special attributes, ladder connections or hazard and water volumes are never merged, and every changed link is re-tested. Hiding spots are recomputed afterwards.

## Game content

Without `--game`, navgen looks for `steamapps/common/GarrysMod/garrysmod` in each Steam library. On Windows it starts from the Steam folder recorded in the registry, then `C:\Program Files (x86)\Steam`; elsewhere it checks `~/.local/share/Steam` and `~/.steam/steam`. Each Steam folder is followed by the libraries listed in its `libraryfolders.vdf`. If Garry's Mod is in none of them, pass `--game`.

Models and physics files are looked up in this order, and the first match wins:

1. the BSP's own pakfile;
2. unpacked addon folders, in sorted order;
3. addon GMAs and Workshop GMAs;
4. `garrysmod_dir.vpk`;
5. the mounted Source game VPKs (`hl2_misc`, `content_hl2`, `content_cstrike`, then other `sourceengine/*_dir.vpk`);
6. loose files.

Files that exist in several places with different contents are listed in the JSON summary (`content_audit`). The order is fixed and may differ from the running game's mount order, which matters only when two addons ship different versions of the same model.

## Accuracy and limits

On a large test map, a mesh generated from the same BSP as the engine's own `nav_generate` mesh had 99.9% coverage precision and 98.7% recall against it. On the engine's own area layout, hiding-spot positions and cover flags matched 7,700 of its 7,729 spots, and in-game NextBot route tests reached the same 52 of 60 goals as the engine mesh. The measurements are in [reports/M5.md](reports/M5.md) and [reports/M6.md](reports/M6.md).

Expect some differences from in-game generation:

- Collision is reimplemented, so area boundaries can differ slightly. Some fence and drop links that offline probes cannot confirm are kept, as the SDK keeps them, and counted as unverified in the JSON summary (under 2% of links on the large test map).
- Physics props without a PHY file add no collision unless `--missing-phy bbox` is used.
- Only spawn-state static geometry is used. Entities spawned by Lua, moving geometry and gamemode-specific traversal are not modelled.
- As in Garry's Mod's own generation (`nav_quicksave 2`), no visibility, encounter or sniper data is computed. Place names are not written, light values are 1 and occupy times are 0.

## Performance

On a 16-thread desktop:

| Map | Profile | Native | Python (warm cache) |
| --- | --- | ---: | ---: |
| gm_construct | sdk | 2.0 s | 14.8 s |
| Large map (164 MB BSP) | sdk, slope 0.55 | 10.9 s | 48.1 s |
| Large map (164 MB BSP) | merged, slope 0.55, seed file | 16.2 s | 72.2 s |

## Building from source

### Native executable

The native build needs CMake 3.21 or newer and a C++20 compiler. All dependencies are vendored.

```powershell
cmake -S cpp -B cpp/build -G "Visual Studio 18 2026" -A x64   # or "Visual Studio 17 2022"
cmake --build cpp/build --config Release --parallel
ctest --test-dir cpp/build -C Release --output-on-failure
```

On Linux, run `cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release`, then `cmake --build cpp/build --parallel`. Builds target AVX2/FMA by default; configure with `-DNAVGEN_HARDWARE_FMA=OFF` for older processors. [cpp/README.md](cpp/README.md) covers compiler settings, the differential test suites and stage dumps.

### Python reference

The reference uses Python 3.10 with NumPy, SciPy, Numba and vpk at the versions pinned in `pyproject.toml`:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -I -m pip install -e .
.\.venv\Scripts\python.exe -I -m navgen "C:\path\to\my_map.bsp"
```

It takes the same options as the native executable. Compiled kernels are cached under `navgen/__pycache__/numba/` (or `NUMBA_CACHE_DIR`), so the first run is slower. The package also includes development tools:

- `navgen.compare` compares coverage and links with a reference mesh;
- `navgen.hiding_compare` compares hiding spots;
- `navgen.phy_audit` checks the physics files involved in content conflicts.

## Tests

```powershell
.\.venv\Scripts\python.exe -I -m unittest discover -s tests
ctest --test-dir cpp/build -C Release
```

Most tests are synthetic. The NAV round-trip test reads the stock `gm_construct` and `gm_flatgrass` meshes from the local Garry's Mod install.

## Repository layout

| Path | Contents |
| --- | --- |
| `cpp/` | Native generator (C++20, CMake), vendored dependencies and differential tests |
| `navgen/` | Python reference implementation and tools |
| `tests/` | Python unit and regression tests |
| `tools/` | Harnesses used for acceptance runs and stage-by-stage comparison |
| `reports/` | Development notes: M1–M9 for the Python generator, `CPP.md` for the native port. Map A, Map B and Map C are private test maps there; A-old is an earlier revision of Map A. |

## Licence

navgen is released under the MIT licence; see [LICENSE](LICENSE). The vendored sources under `cpp/third_party/` keep their own licences, listed in [cpp/third_party/README.md](cpp/third_party/README.md).

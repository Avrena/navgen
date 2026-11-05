# Offline Source nav generator — M9

Generate a Source/GMod v16, subversion 0 NAV from a compiled BSP using Python, NumPy, SciPy and Numba. It does not start the engine. See [M8's independent audits and limitations](reports/M8.md), [M7](reports/M7.md), [M6](reports/M6.md), and the preserved [M5](reports/M5.md), [M4](reports/M4.md), [M3](reports/M3.md), [M2](reports/M2.md) and [M1](reports/M1.md) reports before using its output for pathing. Map A, Map B and Map C in the reports are private test maps; A-old is an earlier revision of Map A.

M5 fixes displacement collision orientation. The SDK's fixed grid indices determine the front, including for `side=1` faces and triangles folded away from their base plane. The previous face-normal alignment could hide walkable terrain and expose floors underneath. The port now swaps the SDK index order once to match its collision kernel's cross-product convention; it preserves triangle order, tags and start-corner rotation.

M7 separates deterministic performance work from the SDK crouch correction. Stage A is preserved under `work/m7/stage-a/`. Stage B evaluates crouch/blocked/ground values once per node at its stored position after sampling; crouch and blocked flags remain sticky, matching `CheckCrouch` on static collision. Incoming step heights no longer overwrite an existing node's corner flags. This correction can change area partitions and NAV bytes; it is separate from thread determinism.

M8's SDK sampler defers reverse obstacle traces until their span survives, skips a range of blocked heights only when a strict convex-interior witness proves every height blocked, and ends collision traversal once the hit fraction and solid flags cannot change. Touching and one-sided terrain cases retain the original scan. These shortcuts preserve node arrays and NAV bytes; reduced trace counts are expected. The original algorithms remain in differential tests, and Part 1's source snapshot is saved separately under `work/m8/part1/`.

`--profile sdk` remains the default and preserves the SDK mesh. `--profile merged` contracts compatible SDK rectangles. It keeps complete shared edges, reciprocal compass links, equal attributes/place/water state and every original corner within 0.5 units of the merged bilinear surface, including chained merges. The merged rectangle is limited to 125 units per side to constrain changes to native area-centre path costs. See [M9 measurements](reports/M9.md) for route deviations and the size/time tradeoff.

Merging protects NO_MERGE, TRANSIENT, NAV_BLOCKER, ladder attachments and known static hazard/cost/water volumes. Shared edges and all changed frontier links require fresh SDK movement/drop/portal witnesses. Unchanged SDK links keep their existing semantics, including explicitly reported unverified exceptions. The exact XY union and every incoming/outgoing compass direction are checked independently. Hiding spots are recomputed afterwards. Runtime blockers and damage state require native validation.

`--seed-discovery entities` opts either profile into validated BSP spawn, teleport-exit, door, button and ladder-endpoint seeds. The default is `none` for both profiles. Provenance and effective seeds are recorded in JSON. Disconnected roofs or rooms may be included; gameplay permissions and teleport traversal edges are not inferred.

`--ladders` separately opts either profile into static `info_ladder` and world-space `func_useableladder` records. Ladders are off by default: native M8.1 tests loaded the records but the stock NextBot failed ladder segments. Supported attachments and swept climb/dismount witnesses are required; parented, disabled, unsupported and unconnected sources are skipped. Attached areas carry NO_MERGE. Format validation does not establish bot traversal.

The separate Source path model retains directed corner/disjoint links and distinguishes complete from valid partial paths using default NextBot jump/drop limits (58/200). It models static area costs and portal placement; native motion and dynamic Lua costs remain outside its scope. The former independent flood, fitting and refinement profile has been retired.

## Run

With an editable installation in `.venv` (see below), in PowerShell:

```powershell
& 'C:\path\to\navgen\.venv\Scripts\python.exe' -I -m navgen `
  'D:\path with spaces\example.bsp' `
  --game 'C:\Program Files (x86)\Steam\steamapps\common\GarrysMod\garrysmod' `
  --out 'D:\path with spaces\example.nav'
```

For a fresh installation, from this directory:

```powershell
py -3.10 -m venv .venv
& '.\.venv\Scripts\python.exe' -I -m pip install --no-cache-dir -e .
```

Without `--game`, navgen uses the first `steamapps/common/GarrysMod/garrysmod` directory found in the Steam libraries: each Steam root (`SteamPath` from the Windows registry, then `C:\Program Files (x86)\Steam`; `~/.local/share/Steam` and `~/.steam/steam` elsewhere), followed by the folders listed in its `steamapps/libraryfolders.vdf`. If none exists, it falls back to the default Steam location. Without `--out`, the output is the BSP path with its suffix changed to `.nav`. Existing output is atomically replaced after validation. Use an explicit staging destination to retain an installed NAV. `bspSize` records the exact bytes of the input BSP, including its current pakfile. Phase timings and counts go to stdout; failures return a nonzero exit code.

`--threads N` selects Numba worker threads; the default uses all available logical cores (or the limit set by `NUMBA_NUM_THREADS` before Python starts). Values must be positive integers within that limit. SDK DFS sampling is serial at every setting; its completed nodes get a parallel crouch pass. Collision, rectangle and later pure probes use workers while area edits retain their sequential order. `--threads 1` also uses direct SDK rectangle claiming. M7.1 removes speculative SDK sampling after look-ahead alternatives failed to beat serial movement on both comparison maps. Trace counts remain exact. The `sampling_speculation` diagnostic remains for compatibility, with `enabled=false` and zero lookup/job counters in the SDK profile.

Numba compilation is cached under `navgen/__pycache__/numba/`, or under `NUMBA_CACHE_DIR` when set. Each cache namespace includes the complete package source hash and NumPy/SciPy/Numba/llvmlite versions, so changing an imported helper also invalidates compiled callers. Numba additionally checks its Python signature and CPU compatibility. The cache stores compiled code only. First-run compilation remains part of reported timings; subsequent processes reuse compatible compiled functions. Restart Python after editing sources; do not change source files during generation.

Seeds come from all `info_player_start` origins and entities whose `targetname` matches a `trigger_teleport` target. Without `--seeds`, an existing `<bsp stem>.navseeds.txt` beside the BSP supplies extra positions. The CLI prints the selected path or that no sidecar was found. `--seeds 'D:\path with spaces\seeds.txt'` overrides that file choice; its positions still supplement BSP seeds. Each nonempty line is `x y z`; `#` starts a comment. A missing explicit file or invalid seed fails the run. An explicit empty file selects BSP seeds only even if a sidecar exists. Input files are read only. Teleport destinations seed islands; this does **not** encode teleport traversal edges in the NAV.

`--slope-limit` sets the minimum ground-normal Z for floor sampling and jump-area marking. Its default is the engine's `0.7`; use `0.55` for the navmesh_optimizer workflow. Overlap rejection deliberately keeps its independent hard-coded `0.7` check. The option also follows the SDK slope threshold in stair and fence cleanup. Valid values are greater than 0 and at most 1.

Carry floors from a previous mesh into the next compile:

```powershell
& '.\.venv\Scripts\python.exe' -I -m navgen.seeds 'D:\maps\previous.nav' `
  --bsp 'D:\maps\current.bsp' | Set-Content -LiteralPath 'D:\maps\current.navseeds.txt' -Encoding utf8
```

The helper selects the largest area by XY size in each weakly connected component and prints a seed on it. With `--bsp`, it grounds the center (or another interior point on that same area) against current collision. Unsupported components are reported on stderr and skipped; a run with no usable seeds fails. It emits only coordinates and comments on stdout. Keep the reviewed `.navseeds.txt` beside the compiled BSP (beside the VMF when Hammer writes the BSP there). A single component can span several floors; if current geometry disconnects those floors, one component seed cannot reach them all. Additional explicit coordinates can cover such floors. SDK generation performs no additional seed discovery by default; either profile can opt into the validated BSP entity floors described above.

`--missing-phy skip` is the default: a VPhysics static prop without a PHY contributes no collision. `--missing-phy bbox` explicitly uses its transformed MDL hull box instead. In isolated M2 runs on Map B, changing only this option at slope 0.7 raised coverage precision/recall from 74.497%/81.997% to 83.422%/91.287%; skip at 0.55 gave 83.083%/91.292%. This supports skipping on these assets, but is not independent proof of every engine branch's behavior. Bounds can both block real floors and create artificial prop tops. Missing MDLs are reported and skipped. Solid-none props remain nonblocking; genuine SOLID_BBOX props still use their MDL boxes. Generation, comparison and seed grounding share the skip default; pass `--missing-phy bbox` explicitly when reproducing older behavior. All available referenced PHY files are parsed, including models whose instances are nonblocking.

Content lookup order is BSP pakfile, sorted unpacked addon directories, local addon GMAs and Workshop GMAs, `garrysmod_dir.vpk`, known Source-engine VPKs (`hl2_misc`, `content_hl2`, `content_cstrike`), other `sourceengine/*_dir.vpk`, then game/sourceengine/download loose directories. Workshop is located at the enclosing Steam library's `steamapps/workshop/content/4000/*/*.gma`. GMA tables are indexed without extracting archives, and requested payload CRCs are verified. The JSON `content_audit` reports same-path files with differing bytes, selected source and SHA256s. The first source wins; alternatives are reported, not silently preferred. This deterministic order is not the running game's mount order. Unsupported archives and unknown brush-entity classes are reported and omitted; review those diagnostics before accepting coverage.

## Hammer++ compile step

Place this **after** the last BSP-changing compile/packing step, normally after vrad++:

Executable:

```text
C:\path\to\navgen\.venv\Scripts\python.exe
```

Arguments:

```text
-I -m navgen "$path\$file.bsp" --game "C:\Program Files (x86)\Steam\steamapps\common\GarrysMod\garrysmod" --slope-limit 0.55 --out "$path\$file.nav"
```

These are Hammer compile-profile variables, not PowerShell variables. The same step works with or without a seed sidecar. Use the executable picker and keep each path argument quoted. The CLI runs without a prompt or dependency on its working directory. Hammer UI integration has not been exercised.

## Compare and check

```powershell
& '.\.venv\Scripts\python.exe' -I -m unittest discover -s tests -v
& '.\.venv\Scripts\python.exe' -I -m navgen.compare `
  'D:\staging\example.nav' 'D:\reference\example.nav' 'D:\maps\example.bsp'
```

Comparison reports coverage on a 25-unit world grid with an 18-unit height tolerance, weak/strong components, directed transitions on common grid coverage, height bands and portal-crossing probes using this implementation's collision. Pass the generation's `--slope-limit` and any nondefault `--missing-phy` option to comparison too. Failed midpoint probes trigger drop and wider-portal probes. Links with no positive-width portal are removed. An ordinary-step link whose movement probes still fail is retained only when a reciprocal sampled-node pair, owned by the two final areas, crosses their shared edge. Ownership follows merges and splits. Other traversal kinds remain explicitly unverified when probes fail. A node witness is sampling evidence, not native route acceptance. Directed transition scores exclude long jump/drop gaps and cannot establish complete graph parity.

Audit the collision bounds behind byte-conflict records with `python -I -m navgen.phy_audit <generation.json> --out <audit.json>`. It compares each convex piece's bounds against the nearest bounds in the other file, in both directions, without relying on piece order. Deltas over one Source unit are reported along with source hashes and overall bounds. Equal bounds do not prove equal hulls; different piece partitions need not change the union. The audit does not change content priority.

The stock NAV round-trip fixtures are read from the local GMod `maps` directory; they are not distributed with the project. Synthetic kernel/generation tests have no game-content dependency. `tools/acceptance_m5.py all` runs the real generation CLI for Construct, Map B, Map A BSP seeds and Map A historical seeds. It writes under `work/m5/` and `reports/measurements/m5/`, records source/input hashes and exact commands, and compares coverage, directed transitions, retained unverified links and hiding spots against M4 and the same references. `python -I -m navgen.hiding_compare ours.nav reference.nav` reports 3D matches within one unit with the same complete flag byte in both directions, plus agreement on uniquely matched exact area extents. The Map A engine reference uses the same BSP; Map B's reference has a different BSP size and remains a historical comparison. M1–M4 reports, measurements and staged NAVs are retained unchanged; their archives preserve the earlier implementations.

Hiding analysis follows GMod's `nav_quicksave 2` generation mode: reciprocal neighbor extents select corners; positions use the SDK 12.5-unit inset/fallbacks and retain corner Z; spots closer than 30 units within one area are suppressed. Cover uses point rays with `MASK_NPCSOLID_BRUSHONLY` and ordinary entity filtering, including brushes ignored by walking generation. IDs start at zero in saved area order. The generator writes `analyzed=1`, with IN_COVER (1) or EXPOSED (8) spots. This flag records completion of that analysis mode, not validation of native routes.

Sniper/encounter analysis is skipped by the engine at nonzero `nav_quicksave`; visibility is skipped at 2. GMod's occupy-time implementation clears both values, and its analysis state skips lighting. Output retains light values 1, occupy times 0, and no visibility, encounters or places. Both profiles leave ladders empty unless --ladders is supplied. Moving entities, Lua-spawned collision, gamemode traversal and teleport links require later runtime integration. M9 generation and audits remain offline. The path model is compared with previously recorded native lengths; new meshes still require native loading and route acceptance.

## Licence

navgen is released under the MIT licence; see [LICENSE](LICENSE). The vendored sources under `cpp/third_party/` keep their own licences, listed in [cpp/third_party/README.md](cpp/third_party/README.md).

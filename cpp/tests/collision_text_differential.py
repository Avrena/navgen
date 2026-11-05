"""BSP text, seed and insertion-order checks against the Python reference."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from navgen.bsp import BSP, entity_angles, vector
from navgen.collision import entity_rules


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    results = []

    def check(name, request, expected, *, bits=False):
        proc = subprocess.run([str(args.probe.resolve())], input=json.dumps(request), text=True,
                              capture_output=True, timeout=30)
        assert proc.returncode == 0, (name, proc.stderr)
        actual = json.loads(proc.stdout)
        if bits:
            a, b = np.asarray(actual, np.float64), np.asarray(expected, np.float64)
            np.testing.assert_array_equal(a.view(np.uint64), b.view(np.uint64), err_msg=name)
        else:
            assert actual == expected, (name, expected, actual)
        results.append({"case": name, "match": True})
        print(f"MATCH {name}", flush=True)

    for i, value in enumerate(("", " \t\n", "1 2 3 4", "1_2.5 .25e+2 -0", "١.٥\u20032\u00a03",
                               "1\x1c2\x1d3", "1e-400 -1e-400 0", "1", "1 2")):
        default = [7., 8., 9.]
        check(f"vector-{i}", {"op": "vector", "value": value, "default": default}, vector(value, default), bits=True)
    for i, entity in enumerate(({}, {"angle": "-1"}, {"angle": "-2"}, {"angle": "3_0"},
                               {"angle": "٣٠"}, {"angles": " \t", "angle": "90"},
                               {"angles": "12.5\u2003-4.25 1_8.0"})):
        check(f"angles-{i}", {"op": "entity_angles", "items": list(entity.items())}, entity_angles(entity), bits=True)
    cases = [
        [("classname", "func_wall_toggle"), ("spawnflags", "1"), ("SpawnFlags", "0")],
        [("classname", "func_wall_toggle"), ("SpawnFlags", "0"), ("spawnflags", "1")],
        [("classname", "func_wall_toggle"), ("spawnflags", "1"), ("SpawnFlags", "0"), ("spawnflags", "2")],
        [("classname", "func_wall"), ("CLASSNAME", "func_door")],
        [("classname", "func_rotating"), ("spawnflags", str(2**100 + 64))],
        [("classname", "func_rotating"), ("spawnflags", str(-2**128 + 64))],
        [("classname", "func_rotating"), ("spawnflags", "0_0_0")],
        [("classname", "func_brush"), ("solidity", str(2**100)), ("startdisabled", "0")],
        [("classname", "func_brush"), ("solidity", "٢"), ("startdisabled", "unused")],
        [("classname", "func_lod"), ("solid", str(2**100))],
        [("classname", "func_breakable"), ("health", "1_2.5")],
        [("classname", "func_breakable_surf"), ("health", "unused")],
        [("classname", "func_breakable"), ("health", "nan")],
    ]
    for i, items in enumerate(cases):
        expected = list(entity_rules(dict(items)))
        check(f"entity-rules-{i}", {"op": "entity_rules", "items": items}, expected)
    with tempfile.TemporaryDirectory(prefix="navgen-text-") as temporary:
        root = Path(temporary)
        text = ('{\n"classname" "info_player_start"\n"origin" "1\xa02\xa03"\n'
                '"targetname" "café"\n"spawnflags" "1"\n"SpawnFlags" "0"\n"spawnflags" "2"\n}\n').encode("latin1")
        header = bytearray(1036)
        struct.pack_into("<4sI", header, 0, b"VBSP", 20)
        struct.pack_into("<4I", header, 8, len(header), len(text), 0, 0)
        path = root / "café.bsp"; path.write_bytes(header + text)
        bsp = BSP(path)
        expected = {"version": bsp.version, "entities": bsp.entities, "models": [],
                    "seeds": bsp.seeds().tolist(), "displacements": [], "props": []}
        check("latin1-bsp-entities", {"op": "bsp", "path": str(path)}, expected)
        seedfile = root / "σπόροι.txt"
        seedfile.write_text("\ufeff# comment\r\n١٠ ٢٠ ٣٠\u20284 5 6 # tail\x859 8 7\n", encoding="utf-8")
        expected["seeds"] = bsp.seeds(seedfile).tolist()
        check("unicode-seed-file", {"op": "bsp", "path": str(path), "seeds": str(seedfile)}, expected)
    report = {"cases": len(results), "matches": len(results), "results": results}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f'{len(results)}/{len(results)} text cases MATCH')


if __name__ == "__main__":
    main()

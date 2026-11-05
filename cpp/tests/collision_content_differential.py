"""Synthetic BSP pakfile, GMA, VPK and content-order differential checks.

Run with the pinned reference environment and --probe <navgen_module_probe>.
All temporary archives are generated locally; no game installation is needed.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import io
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import traceback
import warnings
import zipfile
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from navgen.bsp import BSP, Content
from navgen.gma import GMA
import vpk


def write_bsp(path, pak=b""):
    header = bytearray(1036)
    struct.pack_into("<4sI", header, 0, b"VBSP", 20)
    struct.pack_into("<4I", header, 8 + 40 * 16, len(header), len(pak), 0, 0)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + pak)


def make_zip(entries):
    data = io.BytesIO()
    with warnings.catch_warnings(), zipfile.ZipFile(data, "w") as archive:
        warnings.simplefilter("ignore", UserWarning)
        for name, payload, method in entries:
            archive.writestr(name, payload, compress_type=method)
    return data.getvalue()


def make_single_zip(name, payload, *, zip64=False, comment=b""):
    """A stored CP437 entry, optionally with ZIP64 central-directory fields."""
    crc = zlib.crc32(payload)
    local = struct.pack("<I5H3I2H", 0x04034B50, 20, 0, 0, 0, 0,
                        crc, len(payload), len(payload), len(name), 0) + name + payload
    extra = struct.pack("<HH3Q", 1, 24, len(payload), len(payload), 0) if zip64 else b""
    size = 0xFFFFFFFF if zip64 else len(payload)
    offset = 0xFFFFFFFF if zip64 else 0
    central = struct.pack("<I6H3I5H2I", 0x02014B50, 20, 20, 0, 0, 0, 0,
                          crc, size, size, len(name), len(extra), 0, 0, 0, 0, offset)
    central += name + extra
    end = struct.pack("<I4H2IH", 0x06054B50, 0, 0, 1, 1, len(central), len(local), len(comment))
    return local + central + end + comment


def make_gma(path, entries, version=3):
    data = bytearray(b"GMAD" + struct.pack("<BQQ", version, 1, 2))
    if version > 1:
        data += b"required-content\0\0"
    data += b"synthetic\0description\0author\0" + struct.pack("<i", 1)
    payloads = []
    for i, (name, payload) in enumerate(entries, 1):
        raw = name.encode("utf-8") if isinstance(name, str) else name
        data += struct.pack("<I", i) + raw + b"\0" + struct.pack("<qI", len(payload), zlib.crc32(payload))
        payloads.append(payload)
    data += struct.pack("<I", 0) + b"".join(payloads)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def make_vpk(path, entries, version=2, *, split=False, preload=0, wrong_crc=False):
    """Write the vpk 1.4.0 reader's embedded/preload/external layouts."""
    tree = defaultdict(lambda: defaultdict(list))
    for name, payload in entries:
        directory, filename = name.rsplit("/", 1) if "/" in name else (" ", name)
        stem, extension = filename.rsplit(".", 1)
        tree[extension][directory].append((stem, payload))
    encoded = bytearray()
    body = bytearray()
    for extension, directories in tree.items():
        encoded += extension.encode("utf-8") + b"\0"
        for directory, files in directories.items():
            encoded += directory.encode("utf-8") + b"\0"
            for stem, payload in files:
                prefix = payload[:preload]
                rest = payload[len(prefix):]
                crc = (zlib.crc32(payload) + int(wrong_crc)) & 0xFFFFFFFF
                encoded += stem.encode("utf-8") + b"\0"
                encoded += struct.pack("<IHHIIH", crc, len(prefix), 0 if split else 0x7FFF,
                                       len(body), len(rest), 0xFFFF) + prefix
                body += rest
            encoded += b"\0"
        encoded += b"\0"
    encoded += b"\0"
    header = struct.pack("<3I", 0x55AA1234, version, len(encoded))
    if version == 2:
        header += struct.pack("<4I", 0 if split else len(body), 0, 48, 0)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + encoded + (b"" if split else body) + (bytes(48) if version == 2 else b""))
    if split:
        Path(str(path).replace("english", "").replace("dir.", "000.")).write_bytes(body)


class Checks:
    def __init__(self, probe, root):
        self.probe, self.root, self.results = probe, root, []

    def native(self, request, *, failure=False):
        proc = subprocess.run([str(self.probe)], input=json.dumps(request), text=True,
                              capture_output=True, timeout=120)
        if failure:
            assert proc.returncode != 0, (request, proc.stdout)
            return proc.stderr.strip()
        if proc.returncode:
            raise AssertionError(f"Native request failed: {request}\n{proc.stderr}")
        return json.loads(proc.stdout)

    def compare_content(self, bsp_path, game, names):
        reference = Content(BSP(bsp_path), game)
        reads = {}
        for name in names:
            data = reference.read(name)
            reads[name] = hashlib.sha256(data).hexdigest() if data is not None else None
        expected = {"reads": reads, "used": dict(reference.used), "conflicts": reference.conflicts,
                    "archive_errors": reference.archive_errors,
                    "gma_archives": sum(kind == "gma" for kind, *_ in reference.sources)}
        actual = self.native({"op": "content", "path": str(bsp_path), "game": str(game), "names": names})
        assert actual == expected, json.dumps({"expected": expected, "actual": actual}, indent=2)

    def check(self, name, action):
        begin = time.perf_counter()
        try:
            action()
        except Exception:
            details = traceback.format_exc()
            self.results.append({"case": name, "match": False, "error": details})
            print(f"FAIL {name}\n{details}", flush=True)
        else:
            self.results.append({"case": name, "match": True, "seconds": time.perf_counter() - begin})
            print(f"MATCH {name}", flush=True)

    def zip_case(self, method):
        root = self.root / f"zip-{method}"
        names = ["models/first.mdl", "models/empty.mdl", "models/duplicate.mdl", "models/i\u0307/ος/café.mdl"]
        entries = [("MODELS\\FIRST.MDL", b"payload\0\xff" * 1000, method),
                   ("models/empty.mdl", b"", method),
                   ("models/duplicate.mdl", b"old", method),
                   ("MODELS/DUPLICATE.MDL", b"last normalised name wins", method),
                   ("MODELS/İ/ΟΣ/CAFÉ.MDL", b"unicode", method)]
        bsp_path = root / "pak.bsp"
        write_bsp(bsp_path, make_zip(entries))
        self.compare_content(bsp_path, root / "game", names + ["absent.mdl"])

    def zip_cp437(self, zip64):
        root = self.root / f"zip-cp437-{zip64}"
        bsp_path = root / "pak.bsp"
        write_bsp(bsp_path, make_single_zip("MODELS/CAFÉ.MDL".encode("cp437"), b"cp437", zip64=zip64,
                                          comment=b"directory comment"))
        self.compare_content(bsp_path, root / "game", ["models/café.mdl"])

    def gma_case(self, version):
        path = self.root / f"gma-v{version}" / "test.gma"
        make_gma(path, [("MODELS\\TEST.MDL", b"abc\0\xff" * 400), ("empty.bin", b""),
                        ("MODELS/İ/ΟΣ/CAFÉ.MDL", b"unicode"), ("models/Σ.mdl", b"sigma")], version)
        reference = GMA(path)
        expected = {name: hashlib.sha256(reference.read(name)).hexdigest() for name in reference.entries}
        actual = self.native({"op": "gma", "path": str(path)})
        assert actual == expected, (expected, actual)

    def bad_gma(self, name, kind):
        root = self.root / f"bad-gma-{name}"
        game = root / "game"
        path = game / "addons" / "bad.gma"
        make_gma(path, [("models/a.mdl", b"payload")])
        data = bytearray(path.read_bytes())
        if kind == "signature":
            data[:4] = b"BAD!"
        elif kind == "version":
            data[4] = 4
        elif kind == "payload":
            data.pop()
        elif kind == "duplicate":
            make_gma(path, [("models/a.mdl", b"a"), ("MODELS/A.MDL", b"b")])
            data = bytearray(path.read_bytes())
        elif kind == "path":
            make_gma(path, [("../a.mdl", b"a")]); data = bytearray(path.read_bytes())
        elif isinstance(kind, bytes):
            make_gma(path, [(kind, b"a")]); data = bytearray(path.read_bytes())
        path.write_bytes(data)
        bsp_path = root / "empty.bsp"
        write_bsp(bsp_path)
        self.compare_content(bsp_path, game, ["models/a.mdl"])

    def crc_failure(self, kind):
        root = self.root / f"crc-{kind}"
        if kind == "gma":
            path = root / "test.gma"
            make_gma(path, [("models/test.mdl", b"abc")])
            data = bytearray(path.read_bytes()); data[-1] ^= 1; path.write_bytes(data)
            try:
                GMA(path).read("models/test.mdl")
            except ValueError as exc:
                expected = str(exc)
            else:
                raise AssertionError("Reference accepted the damaged GMA payload")
            actual = self.native({"op": "gma", "path": str(path)}, failure=True)
        else:
            raw = bytearray(make_single_zip(b"models/test.mdl", b"abc"))
            raw[30 + len(b"models/test.mdl")] ^= 1
            path = root / "pak.bsp"; write_bsp(path, raw)
            try:
                Content(BSP(path), root / "game").read("models/test.mdl")
            except zipfile.BadZipFile as exc:
                expected = str(exc)
            else:
                raise AssertionError("Reference accepted the damaged ZIP payload")
            actual = self.native({"op": "content", "path": str(path), "game": str(root / "game"),
                                  "names": ["models/test.mdl"]}, failure=True)
        assert actual == expected, (expected, actual)

    def vpk_case(self, version, split, preload):
        root = self.root / f"vpk-{version}-{split}-{preload}"
        game = root / "garrysmod"
        path = game / "garrysmod_dir.vpk"
        entries = [("MODELS/FIRST.MDL", b"first payload" * 90), ("empty.bin", b""),
                   ("models/İ/ΟΣ/café.mdl", b"unicode payload"),
                   ("models\\backslash/test.mdl", b"not slash-normalised by vpk 1.4.0")]
        make_vpk(path, entries, version, split=split, preload=preload, wrong_crc=True)
        bsp_path = root / "empty.bsp"; write_bsp(bsp_path)
        self.compare_content(bsp_path, game, ["models/first.mdl", "empty.bin", "models/i\u0307/ος/café.mdl",
                                              "models/backslash/test.mdl", "missing.mdl"])

    def vpk_package(self):
        root = self.root / "vpk-package"
        source = root / "source"; (source / "models").mkdir(parents=True)
        (source / "models" / "test.mdl").write_bytes(b"package generated VPK")
        (source / "root.txt").write_bytes(b"root")
        game = root / "garrysmod"; game.mkdir()
        vpk.new(str(source)).save(str(game / "garrysmod_dir.vpk"))
        bsp_path = root / "empty.bsp"; write_bsp(bsp_path)
        self.compare_content(bsp_path, game, ["models/test.mdl", "root.txt"])

    def precedence(self):
        root = self.root / "order" / "steamapps"
        game = root / "common" / "GarrysMod" / "garrysmod"
        engine = game.parent / "sourceengine"
        names = [f"models/{name}.mdl" for name in ("all", "addon", "gma", "workshop", "game_vpk",
                 "known_vpk", "extra_vpk", "loose", "engine", "download", "missing", "same")]
        folders = [(game / "addons" / "a", "addon-a"), (game / "addons" / "z", "addon-z"),
                   (game, "loose"), (engine, "engine"), (game / "download", "download")]
        for folder, label in folders:
            (folder / "models").mkdir(parents=True, exist_ok=True)
            (folder / "models" / "all.mdl").write_bytes(label.encode())
            (folder / "models" / "same.mdl").write_bytes(b"same payload")
        (folders[0][0] / "models" / "addon.mdl").write_bytes(b"addon")
        for label, path in (("gma", game / "addons" / "a.gma"),
                            ("workshop", root / "workshop" / "content" / "4000" / "123" / "workshop.gma")):
            make_gma(path, [("models/all.mdl", label.encode()), (f"models/{label}.mdl", label.encode()),
                            ("models/same.mdl", b"same payload")])
        vpks = [(game / "garrysmod_dir.vpk", "game_vpk"), (engine / "hl2_misc_dir.vpk", "known_vpk"),
                (engine / "content_hl2_dir.vpk", "known-vpk-second"),
                (engine / "content_cstrike_dir.vpk", "known-vpk-third"),
                (engine / "aaa_extra_dir.vpk", "extra_vpk"), (engine / "zzz_extra_dir.vpk", "extra-second")]
        for path, label in vpks:
            make_vpk(path, [("models/all.mdl", label.encode()), (f"models/{label}.mdl", label.encode()),
                            ("models/same.mdl", b"same payload")])
        for folder, label in folders[2:]:
            (folder / "models" / f"{label}.mdl").write_bytes(label.encode())
        bsp_path = root / "order.bsp"
        write_bsp(bsp_path, make_zip([("MODELS/ALL.MDL", b"pak", zipfile.ZIP_DEFLATED),
                                    ("models/same.mdl", b"same payload", zipfile.ZIP_STORED)]))
        self.compare_content(bsp_path, game, names + [names[0]])

    def unicode_filesystem(self):
        root = self.root / "café-Σ-𐐀"
        game = root / "garrysmod"
        # Supplementary-plane and BMP private-use names test codepoint ordering
        # independently of Windows' UTF-16 storage order.
        for label in ("\ue000-addon", "𐀀-addon"):
            folder = game / "addons" / label / "models"
            folder.mkdir(parents=True)
            (folder / "café.mdl").write_bytes(label.encode("utf-8"))
        make_gma(game / "addons" / "Ω.gma", [("models/İ/ΟΣ/CAFÉ.MDL", b"unicode gma filename")])
        bsp_path = root / "κόσμος.bsp"; write_bsp(bsp_path)
        self.compare_content(bsp_path, game, ["MODELS/CAFÉ.MDL", "models/i\u0307/ος/café.mdl"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="navgen-content-") as temporary:
        checks = Checks(args.probe.resolve(), Path(temporary))
        for method in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED, zipfile.ZIP_LZMA):
            checks.check(f"zip-method-{method}", lambda method=method: checks.zip_case(method))
        for zip64 in (False, True):
            checks.check(f"zip-cp437-zip64-{zip64}", lambda zip64=zip64: checks.zip_cp437(zip64))
        for version in (1, 2, 3):
            checks.check(f"gma-version-{version}", lambda version=version: checks.gma_case(version))
        malformed = [(x, x) for x in ("signature", "version", "payload", "duplicate", "path")]
        malformed += [("utf8-start", b"models/\xff.mdl"), ("utf8-short", b"models/\xe2\x82"),
                      ("utf8-surrogate", b"models/\xed\xa0\x80.mdl"), ("utf8-continuation", b"models/\xe2\x82X.mdl")]
        for name, kind in malformed:
            checks.check(f"gma-invalid-{name}", lambda name=name, kind=kind: checks.bad_gma(name, kind))
        for kind in ("gma", "zip"):
            checks.check(f"crc-{kind}", lambda kind=kind: checks.crc_failure(kind))
        for version in (1, 2):
            for split in (False, True):
                for preload in (0, 5, 2000):
                    checks.check(f"vpk-{version}-split-{split}-preload-{preload}",
                                 lambda version=version, split=split, preload=preload: checks.vpk_case(version, split, preload))
        checks.check("vpk-package-1.4.0", checks.vpk_package)
        checks.check("content-order-addons-workshop-vpks-folders", checks.precedence)
        checks.check("unicode-filesystem-labels-and-path-order", checks.unicode_filesystem)
        report = {"cases": len(checks.results), "matches": sum(r["match"] for r in checks.results),
                  "results": checks.results}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f'{report["matches"]}/{report["cases"]} content cases MATCH')
        return 0 if report["matches"] == report["cases"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

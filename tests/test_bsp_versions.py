"""Game-independent BSP v20/v21 fixtures with known collision geometry."""
from pathlib import Path
from tempfile import TemporaryDirectory
import lzma
import struct
import unittest

import numpy as np

from navgen.bsp import BSP


ENTITIES = (b'{\n"classname" "worldspawn"\n}\n'
            b'{\n"classname" "info_player_start"\n"origin" "32 64 16"\n}\n\0')
LAYOUTS = ((20, False), (21, False), (21, True))
# liblzma 5.8.1, LZMA_FILTER_LZMA1EXT with ext_flags=0, dictionary=1 MiB,
# lc=3/lp=0/pb=2. This is a complete range-coded stream with no end marker,
# as written by Source, rather than a truncated marker-bearing stream.
NO_END_MARKER = bytes.fromhex(
    '003d8280452210f174dbd4fb9bd48080a9a839f63212b9b099241da89ed04d173498bb8d'
    'b3a462679e086a7e5dc9dc5f5810a3f6fde0809facf47afece571cb0e3868d0e15d2000000')
LZMA_FILTERS = [{'id': lzma.FILTER_LZMA1, 'dict_size': 1 << 20, 'lc': 3, 'lp': 0, 'pb': 2}]


def directory_entry(data, number, offset, size, version=0, fourcc=0, l4d2=False):
    fields = (version, offset, size, fourcc) if l4d2 else (offset, size, version, fourcc)
    struct.pack_into('<4I', data, 8 + 16 * number, *fields)


def bsp_header(version):
    data = bytearray(1036)
    struct.pack_into('<4sI', data, 0, b'VBSP', version)
    return data


def geometry_bsp(version=21, l4d2=True, compressed=False, entities=True, end_marker=True):
    """A 128-square floor, its raised displacement, and two L4D2 v9 props.

    The world model addresses the second 32-byte leaf: interpreting the leaf
    lump version as its offset therefore cannot accidentally pass this fixture.
    """
    data = bsp_header(version)

    def add(number, payload, lump_version=0, fourcc=0):
        data.extend(bytes((-len(data)) % 4))
        directory_entry(data, number, len(data), len(payload), lump_version, fourcc, l4d2)
        data.extend(payload)

    if entities:
        if compressed:
            packed = lzma.compress(ENTITIES, format=lzma.FORMAT_RAW, filters=LZMA_FILTERS) if end_marker else NO_END_MARKER
            payload = b'LZMA' + struct.pack('<II', len(ENTITIES), len(packed)) + b'\x5d' + struct.pack('<I', 1 << 20) + packed
            add(0, payload, fourcc=len(ENTITIES))
        else:
            add(0, ENTITIES)

    planes = ((1, 0, 0, 128), (-1, 0, 0, 0), (0, 1, 0, 128),
              (0, -1, 0, 0), (0, 0, 1, 0), (0, 0, -1, 16))
    add(1, b''.join(struct.pack('<4fi', *p, axis // 2) for axis, p in enumerate(planes)))
    corners = ((0, 0, 0), (0, 128, 0), (128, 128, 0), (128, 0, 0))
    add(3, b''.join(struct.pack('<3f', *p) for p in corners))
    texinfo = bytearray(72)
    struct.pack_into('<I', texinfo, 64, 0x20)
    add(6, texinfo)
    face = bytearray(56)
    struct.pack_into('<HBBihhh', face, 0, 4, 0, 1, 0, 4, 0, 0)
    add(7, face)
    leaves = bytearray(64)
    struct.pack_into('<HH', leaves, 32 + 24, 0, 1)
    add(10, leaves, lump_version=1)
    add(12, struct.pack('<8H', 0, 1, 1, 2, 2, 3, 3, 0))
    add(13, struct.pack('<4i', 0, 1, 2, 3))
    add(14, struct.pack('<9f3i', 0, 0, -16, 128, 128, 4, 0, 0, 0, -2, 0, 1))
    add(17, struct.pack('<H', 0))
    add(18, struct.pack('<iiI', 0, 6, 1))
    add(19, b''.join(struct.pack('<HhhBB', p, 0, -1, 0, 0) for p in range(6)))
    disp = bytearray(176)
    struct.pack_into('<4ifiH', disp, 12, 0, 0, 2, 0, 0., 1, 0)
    add(26, disp)
    add(33, struct.pack('<5f', 0, 0, 1, 4, 0) * 25)
    add(48, struct.pack('<32H', 1, 32, *([1] * 30)))

    # L4D2 sprp v9 adds CPU/GPU levels, diffuse modulation and disableX360;
    # the total record size is 72 bytes. Game-lump offsets stay file-absolute.
    props = bytearray(struct.pack('<I', 1) + b'Models\\Props\\Test.mdl'.ljust(128, b'\0'))
    props.extend(struct.pack('<I2HI', 2, 0, 1, 2))
    for origin, angles, solid in (((8, 16, 32), (0, 90, 0), 6), ((80, 96, 128), (0, 0, 45), 2)):
        record = bytearray(72)
        struct.pack_into('<6f3H2Bi', record, 0, *origin, *angles, 0, 0, 1, solid, 0, 0)
        struct.pack_into('<f4B4B4B', record, 56, 1., 1, 3, 2, 4, 255, 128, 64, 255, 1, 0, 0, 0)
        props.extend(record)
    game_offset = (len(data) + 3) & ~3
    game = struct.pack('<IIHHII', 1, 0x73707270, 0, 9, game_offset + 20, len(props)) + props
    add(35, game)
    return data


class BSPVersionTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'fixture.bsp'

    def read(self, data):
        self.path.write_bytes(data)
        return BSP(self.path)

    def test_geometry_in_all_layouts(self):
        for version, l4d2 in LAYOUTS:
            with self.subTest(version=version, l4d2=l4d2):
                bsp = self.read(geometry_bsp(version, l4d2))
                self.assertEqual(bsp.version, version)
                self.assertEqual(bsp.leaf_stride, 32)
                self.assertEqual(bsp.lumps[10][2], 1)
                self.assertEqual(bsp.model_brushes(0), [0])
                np.testing.assert_array_equal(bsp.seeds(), [[32, 64, 16]])
                planes, flags, lo, hi, sides = bsp.brush(0)
                self.assertEqual(len(planes), 6)
                self.assertEqual(len(sides), 6)
                np.testing.assert_array_equal(flags, [0x20] * 6)
                np.testing.assert_array_equal(lo, [0, 0, -16])
                np.testing.assert_array_equal(hi, [128, 128, 0])
                vertices, normal = bsp.face_vertices(0)
                np.testing.assert_array_equal(vertices, [[0, 0, 0], [0, 128, 0], [128, 128, 0], [128, 0, 0]])
                np.testing.assert_array_equal(normal, [0, 0, 1])
                triangles = list(bsp.displacements())
                self.assertEqual(len(triangles), 31)
                np.testing.assert_array_equal(triangles[0][0], [[0, 0, 4], [32, 32, 4], [0, 32, 4]])
                self.assertEqual(triangles[0][1:], (1, 1))
                props = list(bsp.static_props())
                self.assertEqual(len(props), 2)
                self.assertEqual([p[0] for p in props], ['models/props/test.mdl'] * 2)
                self.assertEqual([p[1] for p in props], [6, 2])
                np.testing.assert_array_equal([p[2] for p in props], [[8, 16, 32], [80, 96, 128]])
                np.testing.assert_array_equal([p[3] for p in props], [[0, 90, 0], [0, 0, 45]])
                self.assertEqual([p[4] for p in props], [1., 1.])

    def test_compressed_entities_keep_fourcc(self):
        for version, l4d2 in LAYOUTS:
            for end_marker in (False, True):
                with self.subTest(version=version, l4d2=l4d2, end_marker=end_marker):
                    bsp = self.read(geometry_bsp(version, l4d2, compressed=True, end_marker=end_marker))
                    self.assertEqual(bsp.lump(0), ENTITIES)
                    self.assertEqual(bsp.lumps[0][3], len(ENTITIES))
                    np.testing.assert_array_equal(bsp.seeds(), [[32, 64, 16]])
                    self.assertEqual(bsp.model_brushes(0), [0])

    def test_no_end_marker_fixture_is_complete(self):
        decoder = lzma.LZMADecompressor(format=lzma.FORMAT_RAW, filters=LZMA_FILTERS)
        self.assertEqual(decoder.decompress(NO_END_MARKER), ENTITIES)
        self.assertFalse(decoder.eof)

    def test_compressed_lump_lengths_and_empty_streams(self):
        for version, l4d2 in LAYOUTS:
            def read_payload(payload):
                data = geometry_bsp(version, l4d2)
                offset = len(data)
                data.extend(payload)
                directory_entry(data, 0, offset, len(payload), l4d2=l4d2)
                return self.read(data)

            # Even an empty raw LZMA stream needs the five-byte range state.
            for packed in (0, 1, 4, 5):
                with self.subTest(version=version, l4d2=l4d2, empty_bytes=packed):
                    payload = b'LZMA' + struct.pack('<II', 0, packed) + b'\x5d' + struct.pack('<I', 1 << 20) + bytes(packed)
                    if packed < 5:
                        with self.assertRaisesRegex(ValueError, 'compressed BSP lump'):
                            read_payload(payload)
                    else:
                        bsp = read_payload(payload)
                        self.assertEqual(bsp.entities, [])
                        self.assertEqual(bsp.model_brushes(0), [0])

            for actual, packed in ((len(ENTITIES) - 1, len(NO_END_MARKER)),
                                   (len(ENTITIES) + 1, len(NO_END_MARKER)),
                                   (len(ENTITIES), len(NO_END_MARKER) + 1)):
                with self.subTest(version=version, l4d2=l4d2, actual=actual, packed=packed):
                    payload = b'LZMA' + struct.pack('<II', actual, packed) + b'\x5d' + struct.pack('<I', 1 << 20) + NO_END_MARKER
                    with self.assertRaisesRegex(ValueError, 'compressed BSP lump'):
                        read_payload(payload)

    def test_layout_detection_without_entities(self):
        for version, l4d2 in LAYOUTS:
            with self.subTest(version=version, l4d2=l4d2):
                bsp = self.read(geometry_bsp(version, l4d2, entities=False))
                self.assertEqual(bsp.entities, [])
                self.assertEqual(bsp.leaf_stride, 32)
                self.assertEqual(bsp.model_brushes(0), [0])

    def test_hdr_faces_when_ldr_faces_are_empty(self):
        for version, l4d2 in LAYOUTS:
            with self.subTest(version=version, l4d2=l4d2):
                data = geometry_bsp(version, l4d2)
                data[8 + 16 * 58:8 + 16 * 59] = data[8 + 16 * 7:8 + 16 * 8]
                data[8 + 16 * 7:8 + 16 * 8] = bytes(16)
                bsp = self.read(data)
                np.testing.assert_array_equal(bsp.face_vertices(0)[0],
                                              [[0, 0, 0], [0, 128, 0], [128, 128, 0], [128, 0, 0]])
                self.assertEqual(len(list(bsp.displacements())), 31)

    def test_empty_directory_and_empty_lump_at_eof(self):
        for version, l4d2 in LAYOUTS:
            with self.subTest(version=version, l4d2=l4d2):
                self.assertEqual(self.read(bsp_header(version)).entities, [])
                data = bsp_header(version)
                directory_entry(data, 10, len(data), 0, version=1, l4d2=l4d2)
                bsp = self.read(data)
                self.assertEqual(bsp.leaf_stride, 32)
                self.assertEqual(bsp.lump(10), b'')

    def test_reject_invalid_directory_including_unused_lumps(self):
        for version, l4d2 in LAYOUTS:
            # Lump 63 is not consumed by generation: still validate its span.
            source = geometry_bsp(version, l4d2)
            for offset, size in ((100, 1), (len(source), 1), (len(source) + 1, 0), (0xfffffffc, 16)):
                with self.subTest(version=version, l4d2=l4d2, offset=offset, size=size):
                    data = bytearray(source)
                    directory_entry(data, 63, offset, size, l4d2=l4d2)
                    with self.assertRaisesRegex(ValueError, 'BSP lump directory'):
                        self.read(data)

    def test_ambiguous_v21_directory_is_rejected(self):
        data = bsp_header(21) + bytes(1036)
        # Both [offset=1036, size=1036, version=16] and the reordered
        # [offset=1036, size=16, version=1036] fit, but describe different data.
        struct.pack_into('<4I', data, 8 + 16 * 63, 1036, 1036, 16, 0)
        with self.assertRaisesRegex(ValueError, 'Ambiguous BSP v21'):
            self.read(data)

    def test_invalid_magic_truncated_header_and_unsupported_versions(self):
        for data in (b'VBSP', bsp_header(21)[:-1], b'NOPE' + bsp_header(21)[4:]):
            with self.subTest(size=len(data), magic=data[:4]):
                with self.assertRaisesRegex(ValueError, 'Not a Source BSP'):
                    self.read(data)
        for version in (19, 22):
            with self.subTest(version=version):
                with self.assertRaisesRegex(ValueError, str(version)):
                    self.read(bsp_header(version))
        with self.assertRaisesRegex(ValueError, 'BSP lump directory'):
            self.read(geometry_bsp(20, True))


if __name__ == '__main__':
    unittest.main()

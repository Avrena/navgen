"""Read and write Source / GMod .nav files (version 16, subversion 0), layout as in Source SDK 2013 nav_file.cpp."""
import struct, sys

class Reader:
    def __init__(self, data): self.d, self.o = data, 0
    def get(self, fmt):
        v = struct.unpack_from('<' + fmt, self.d, self.o); self.o += struct.calcsize('<' + fmt)
        return v[0] if len(v) == 1 else v
    def ids(self, n): return list(self.get(f'{n}I')) if n > 1 else ([self.get('I')] if n else [])

def read_nav(data):
    r = Reader(data)
    nav = {'magic': r.get('I'), 'version': r.get('I')}
    assert nav['magic'] == 0xFEEDFACE and nav['version'] == 16, nav
    nav['subversion'], nav['bsp_size'], nav['analyzed'] = r.get('I'), r.get('I'), r.get('B')
    places = []
    for _ in range(r.get('H')):
        n = r.get('H'); places.append(data[r.o:r.o + n]); r.o += n
    nav['places'], nav['has_unnamed'] = places, r.get('B')
    areas = []
    for _ in range(r.get('I')):
        a = {'id': r.get('I'), 'attr': r.get('I'), 'nw': r.get('3f'), 'se': r.get('3f'), 'ne_z': r.get('f'), 'sw_z': r.get('f')}
        a['links'] = [r.ids(r.get('I')) for _ in range(4)]
        a['hiding'] = [(r.get('I'), r.get('3f'), r.get('B')) for _ in range(r.get('B'))]
        enc = []
        for _ in range(r.get('I')):
            head = r.get('IBIB')
            enc.append((head, [r.get('IB') for _ in range(r.get('B'))]))
        a['encounters'], a['place'] = enc, r.get('H')
        a['ladders'] = [r.ids(r.get('I')) for _ in range(2)]
        a['occupy'], a['light'] = r.get('2f'), r.get('4f')
        a['visible'] = [r.get('IB') for _ in range(r.get('I'))]
        a['inherit_vis'] = r.get('I')
        areas.append(a)
    nav['areas'] = areas
    nav['ladders'] = [{'id': r.get('I'), 'width': r.get('f'), 'top': r.get('3f'), 'bottom': r.get('3f'), 'length': r.get('f'),
                       'dir': r.get('I'), 'areas': r.get('5I')} for _ in range(r.get('I'))]
    nav['trailing'] = data[r.o:]
    return nav

def write_nav(nav):
    out = bytearray(); p = lambda fmt, *v: out.extend(struct.pack('<' + fmt, *v))
    p('IIIIB', nav['magic'], nav['version'], nav['subversion'], nav['bsp_size'], nav['analyzed'])
    p('H', len(nav['places']))
    for s in nav['places']: p('H', len(s)); out.extend(s)
    p('B', nav['has_unnamed']); p('I', len(nav['areas']))
    for a in nav['areas']:
        p('II3f3fff', a['id'], a['attr'], *a['nw'], *a['se'], a['ne_z'], a['sw_z'])
        for l in a['links']: p(f'I{len(l)}I', len(l), *l)
        p('B', len(a['hiding']))
        for sid, pos, fl in a['hiding']: p('I3fB', sid, *pos, fl)
        p('I', len(a['encounters']))
        for head, spots in a['encounters']:
            p('IBIB', *head); p('B', len(spots))
            for s in spots: p('IB', *s)
        p('H', a['place'])
        for l in a['ladders']: p(f'I{len(l)}I', len(l), *l)
        p('2f4f', *a['occupy'], *a['light'])
        p('I', len(a['visible']))
        for v in a['visible']: p('IB', *v)
        p('I', a['inherit_vis'])
    p('I', len(nav['ladders']))
    for L in nav['ladders']:
        p('If3f3ffI5I', L['id'], L['width'], *L['top'], *L['bottom'], L['length'], L['dir'], *L['areas'])
    out.extend(nav['trailing'])
    return bytes(out)

if __name__ == '__main__':
    for path in sys.argv[1:]:
        data = open(path, 'rb').read()
        nav = read_nav(data)
        same = write_nav(nav) == data
        a = nav['areas']
        print(f"{path.split('/')[-1]}: areas={len(a)} ladders={len(nav['ladders'])} trailing={len(nav['trailing'])} B "
              f"links={sum(len(l) for x in a for l in x['links'])} hiding={sum(len(x['hiding']) for x in a)} "
              f"encounters={sum(len(x['encounters']) for x in a)} visible={sum(len(x['visible']) for x in a)} "
              f"lit={sum(1 for x in a if x['light'] != (1.0,) * 4)} attrs={len({x['attr'] for x in a})} byte_identical={same}")

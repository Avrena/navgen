"""BSP v20 geometry, spawn-state entities and deterministic content resolution."""
from __future__ import annotations

import io
import hashlib
import itertools
import lzma
import math
import re
import struct
import zipfile
from collections import Counter
from pathlib import Path

import numpy as np
import vpk
from scipy.spatial import ConvexHull
from .gma import GMA,content_path

DEFAULT_GAME = Path(r'C:\Program Files (x86)\Steam\steamapps\common\GarrysMod\garrysmod')
BRUSH_MASK = 0x2400B
NPC_MASK = BRUSH_MASK | 0x2000000
BRUSH_SIDE_DTYPE = np.dtype([('plane','<u2'),('tex','<i2'),('disp','<i2'),('bevel','u1'),('thin','u1')])


def vector(value: str, default=(0., 0., 0.)) -> np.ndarray:
    values=[float(x) for x in value.split()[:3]] if value else list(default)
    return np.asarray(values+[0.]*(3-len(values)),dtype=np.float64)


def entity_angles(entity):
    if entity.get('angles'):return vector(entity['angles'])
    yaw=float(entity.get('angle','0'))
    if yaw==-1:return np.array([-90.,0.,0.])
    if yaw==-2:return np.array([90.,0.,0.])
    return np.array([0.,yaw,0.])


def rotation(angles):
    p, y, r = np.radians(angles)
    cp, sp, cy, sy, cr, sr = math.cos(p), math.sin(p), math.cos(y), math.sin(y), math.cos(r), math.sin(r)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def box_vertices(lo, hi):
    return np.array(list(itertools.product(*zip(lo, hi))), dtype=np.float64)


class BSP:
    def __init__(self, path):
        self.path = Path(path)
        self.data = self.path.read_bytes()
        if len(self.data) < 1036 or self.data[:4] != b'VBSP':
            raise ValueError('Not a Source BSP')
        self.version = struct.unpack_from('<I', self.data, 4)[0]
        if self.version != 20:
            raise ValueError(f'Expected BSP v20, got {self.version}')
        self.lumps = [struct.unpack_from('<4I', self.data, 8+16*i) for i in range(64)]
        self.entities = [dict(re.findall(r'"([^"\n]*)"\s+"([^"\n]*)"', block))
                         for block in re.findall(r'\{([^{}]*)\}', self.lump(0).decode('latin1'))]
        self.planes = np.frombuffer(self.lump(1), dtype=[('n','<f4',3),('d','<f4'),('type','<i4')])
        self.brushes = np.frombuffer(self.lump(18), dtype=[('first','<i4'),('count','<i4'),('contents','<u4')])
        self.sides = np.frombuffer(self.lump(19), dtype=BRUSH_SIDE_DTYPE)
        self.models = list(struct.iter_unpack('<9f3i', self.lump(14)))
        self.nodes, self.leaves = self.lump(5), self.lump(10)
        self.leaf_stride = 32 if self.lumps[10][2] == 1 else 56
        self.leafbrushes = np.frombuffer(self.lump(17), dtype='<u2')
        self.texinfo = self.lump(6)
        self.faces = self.lump(7) or self.lump(58)
        self.vertices = np.frombuffer(self.lump(3),dtype='<f4').reshape(-1,3)
        self.edges = np.frombuffer(self.lump(12),dtype='<u2').reshape(-1,2)
        self.surfedges = np.frombuffer(self.lump(13),dtype='<i4')
        self.pak = zipfile.ZipFile(io.BytesIO(self.lump(40))) if self.lump(40) else None

    def lump(self, index):
        off, size, _, _ = self.lumps[index]
        if off+size > len(self.data):
            raise ValueError(f'Truncated BSP lump {index}')
        data = self.data[off:off+size]
        if data[:4] == b'LZMA':
            actual, packed = struct.unpack_from('<II', data, 4)
            prop = data[12]
            filters = [{'id': lzma.FILTER_LZMA1, 'dict_size': struct.unpack_from('<I',data,13)[0],
                        'lc': prop % 9, 'lp': (prop//9) % 5, 'pb': prop//45}]
            data = lzma.decompress(data[17:17+packed], format=lzma.FORMAT_RAW, filters=filters)
            if len(data) != actual:
                raise ValueError('Invalid compressed BSP lump length')
        return data

    def model_brushes(self, model):
        todo, visited, found = [self.models[model][9]], set(), set()
        while todo:
            node = todo.pop()
            if node in visited:
                continue
            visited.add(node)
            if node >= 0:
                todo.extend(struct.unpack_from('<2i', self.nodes, node*32+4))
            else:
                first, count = struct.unpack_from('<2H',self.leaves,(-1-node)*self.leaf_stride+24)
                found.update(int(i) for i in self.leafbrushes[first:first+count])
        return sorted(found)

    def brush(self, index):
        row = self.brushes[index]
        sides = self.sides[row['first']:row['first']+row['count']]
        p = self.planes[sides['plane']]
        planes = np.column_stack([p['n'], p['d']]).astype(np.float64)
        flags = np.array([struct.unpack_from('<I',self.texinfo,int(t)*72+64)[0] if t>=0 else 0 for t in sides['tex']],dtype=np.int64)
        lo, hi = np.full(3,-np.inf), np.full(3,np.inf)
        for axis in range(3):
            axial = np.all(np.abs(planes[:,[a for a in range(3) if a!=axis]])<1e-7,axis=1)
            plus = axial & (planes[:,axis]>.99999)
            minus = axial & (planes[:,axis]<-.99999)
            if plus.any(): hi[axis] = planes[plus,3].min()
            if minus.any(): lo[axis] = -planes[minus,3].min()
        if not np.isfinite(np.r_[lo,hi]).all():
            raise ValueError(f'Brush {index} has no axial bounds')
        return planes, flags, lo, hi, sides

    def face_vertices(self, index):
        pn, side, _, first, count, tex, disp = struct.unpack_from('<HBBihhh',self.faces,index*56)
        se = self.surfedges[first:first+count]
        vertices = self.vertices[self.edges[np.abs(se),(se<0).astype(np.int64)]].astype(np.float64)
        normal = self.planes[pn]['n'].astype(np.float64) * (-1 if side else 1)
        return vertices, normal

    def displacements(self):
        info = self.lump(26)
        verts = np.frombuffer(self.lump(33), dtype=[('n','<f4',3),('d','<f4'),('alpha','<f4')])
        tags = np.frombuffer(self.lump(48),dtype='<u2')
        for off in range(0,len(info),176):
            start = np.array(struct.unpack_from('<3f',info,off))
            vi, ti, power, _, _, contents, face = struct.unpack_from('<4ifiH',info,off+12)
            corners, _ = self.face_vertices(face)
            if len(corners)!=4 or not 2 <= power <= 4:
                raise ValueError('Invalid displacement base face/power')
            corners = np.roll(corners,-int(np.argmin(np.linalg.norm(corners-start,axis=1))),axis=0)
            n = (1<<power)+1
            # Source disp space: corner 0->3 is X; corner 0->1 is Y (bspfile.h).
            y,x = np.meshgrid(np.linspace(0,1,n),np.linspace(0,1,n),indexing='ij')
            points = ((1-x)[...,None]*(1-y)[...,None]*corners[0] +
                      x[...,None]*(1-y)[...,None]*corners[3] +
                      x[...,None]*y[...,None]*corners[2] + (1-x)[...,None]*y[...,None]*corners[1]).reshape(-1,3)
            v = verts[vi:vi+n*n]
            points += v['n']*v['d'][:,None]
            k = ti
            for yy in range(n-1):
                for xx in range(n-1):
                    a = yy*n+xx
                    if (xx+yy)&1:
                        pairs = ((a,a+n,a+1),(a+1,a+n,a+n+1))
                    else:
                        pairs = ((a,a+n,a+n+1),(a,a+n+1,a+1))
                    for indices in pairs:
                        # SDK CalcPlane uses (v2-v0) x (v1-v0), opposite our kernel.
                        # Swap once: neither face.side nor a sculpted fold changes winding.
                        tri = points[[indices[0],indices[2],indices[1]]]
                        tag = int(tags[k]); k+=1
                        if not(tag & 32):
                            yield tri, int(contents), tag

    def static_props(self):
        gl = self.lump(35)
        if not gl: return
        for i in range(struct.unpack_from('<i',gl)[0]):
            gid, flags, version, off, size = struct.unpack_from('<IHHII',gl,4+16*i)
            if gid != int.from_bytes(b'sprp','big'): continue
            if flags & 1: raise ValueError('Compressed static-prop game lump unsupported')
            d = self.data[off:off+size]
            count = struct.unpack_from('<i',d)[0]
            names = [d[4+128*k:4+128*(k+1)].split(b'\0')[0].decode('latin1').lower().replace('\\','/') for k in range(count)]
            o = 4+128*count
            leaves = struct.unpack_from('<i',d,o)[0]; o += 4+2*leaves
            count = struct.unpack_from('<i',d,o)[0]; o+=4
            if not count: return
            stride, remainder = divmod(len(d)-o,count)
            if remainder or stride<56: raise ValueError('Unsupported static-prop record size')
            for k in range(count):
                pos = o+k*stride
                org = np.array(struct.unpack_from('<3f',d,pos))
                angles = np.array(struct.unpack_from('<3f',d,pos+12))
                model = names[struct.unpack_from('<H',d,pos+24)[0]]
                solid = d[pos+30]
                # GMod v10 is 72 bytes; scaled v11 extends the standard 80-byte record.
                scale = struct.unpack_from('<f',d,pos+80)[0] if version==11 and stride>=84 else 1.
                yield model, solid, org, angles, scale

    def seeds(self, file=None):
        targets = {e.get('target') for e in self.entities if e.get('classname')=='trigger_teleport'}
        result = [vector(e.get('origin','')) for e in self.entities
                  if e.get('classname')=='info_player_start' or (e.get('targetname') and e['targetname'] in targets)]
        if file:
            for lineno,line in enumerate(Path(file).read_text(encoding='utf-8-sig').splitlines(),1):
                line = line.split('#',1)[0].strip()
                if line:
                    value = vector(line)
                    if len(line.split())!=3 or not np.isfinite(value).all():
                        raise ValueError(f'Invalid seed at line {lineno}')
                    result.append(value)
        if not result: raise ValueError('No info_player_start, teleport destination or supplied seed')
        return np.unique(np.asarray(result),axis=0)


class Content:
    def __init__(self, bsp, game):
        self.game = Path(game)
        self.pak = bsp.pak
        self.paknames = {n.lower().replace('\\','/'):n for n in self.pak.namelist()} if self.pak else {}
        self.sources = []
        self.used = Counter()
        self.missing = set()
        self.conflicts = []
        self.archive_errors = []
        addons = self.game/'addons'
        for addon in sorted(addons.iterdir()) if addons.exists() else []:
            if addon.is_dir(): self.sources.append(('folder',addon,None,str(addon)))
        steamapps=next((p for p in self.game.parents if p.name.lower()=='steamapps'),None)
        archives=list(addons.glob('*.gma'))
        if steamapps:archives+=list((steamapps/'workshop/content/4000').glob('*/*.gma'))
        for path in sorted(archives):
            try:archive=GMA(path)
            except (OSError,ValueError,UnicodeError) as exc:
                self.archive_errors.append(str(exc));continue
            self.sources.append(('gma',archive,archive.entries,str(path)))
        paths = [self.game/'garrysmod_dir.vpk']
        engine = self.game.parent/'sourceengine'
        known = ['hl2_misc_dir.vpk','content_hl2_dir.vpk','content_cstrike_dir.vpk']
        paths += [engine/n for n in known]
        paths += [p for p in sorted(engine.glob('*_dir.vpk')) if p.name not in known]
        for path in paths:
            if path.is_file():
                archive = vpk.open(str(path))
                self.sources.append(('vpk',archive,{k.lower():k for k in archive},str(path)))
        for path in (self.game,engine,self.game/'download'):
            self.sources.append(('folder',path,None,str(path)))

    def read(self,name):
        name = content_path(name)
        selected=None;selected_source=None;alternatives=[];selected_hash=None
        if name in self.paknames:
            selected=self.pak.read(self.paknames[name]);selected_source='BSP pakfile'
        for kind,source,index,label in self.sources:
            data=None
            if kind=='folder':
                p = source/name
                if p.is_file():data=p.read_bytes()
            elif name in index:
                data=source.read(name) if kind=='gma' else source.get_file(index[name]).read()
            if data is None:continue
            if selected is None:selected=data;selected_source=label
            elif data!=selected:
                if selected_hash is None:selected_hash=hashlib.sha256(selected).hexdigest()
                alternatives.append({'source':label,'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)})
        if selected is not None:
            self.used[selected_source]+=1
            if alternatives:self.conflicts.append({'path':name,'selected':selected_source,'sha256':selected_hash,'bytes':len(selected),'different_sources':alternatives})
            return selected
        self.missing.add(name)
        return None


def parse_phy(data):
    hsize, _, nsolids, _ = struct.unpack_from('<4i',data)
    o, pieces = hsize, []
    for _ in range(nsolids):
        size = struct.unpack_from('<i',data,o)[0]; start=o+4
        if size<=0 or start+size>len(data): raise ValueError('Truncated PHY solid')
        if data[start:start+4]==b'VPHY':
            _,kind,_ = struct.unpack_from('<hhi',data,start+4)
            if kind!=0: raise ValueError(f'Unsupported PHY surface type {kind}')
            surf=start+28
        else: surf=start
        tree = struct.unpack_from('<i',data,surf+32)[0]
        lo,end = surf+48,surf+tree
        while lo<end:
            point_off,_,flags,ntri = struct.unpack_from('<iiIh',data,lo)
            if ntri<=0 or point_off<=0: raise ValueError('Invalid PHY ledge')
            end=min(end,lo+point_off)
            if flags&3==0:
                ids=set()
                for t in range(ntri):
                    for e in range(3):
                        ids.add(struct.unpack_from('<I',data,lo+20+t*16+e*4)[0]&0xffff)
                p=np.array([struct.unpack_from('<3f',data,lo+point_off+16*i) for i in sorted(ids)])
                points=p[:,[0,2,1]]*np.array([1,1,-1])/0.0254
                if not len(points) or not np.isfinite(points).all(): raise ValueError('Invalid PHY convex points')
                pieces.append(points)
            lo+=16+ntri*16
        o=start+size
    if not pieces: raise ValueError('PHY contains no convex pieces')
    return pieces


def convex_planes(points):
    centered=points-points[0]
    rank=np.linalg.matrix_rank(centered,tol=1e-7)
    if rank<3:
        # VPHY terminal ledges can be flat triangles rather than solid tetrahedra.
        _,_,basis=np.linalg.svd(centered,full_matrices=True)
        axes=[*np.eye(3),*basis[rank:]]
        if rank==2:
            hull=ConvexHull(centered@basis[:2].T)
            boundary=points[hull.vertices]
            edges=np.roll(boundary,-1,axis=0)-boundary
        elif rank==1:edges=np.array([points[-1]-points[0]])
        else:edges=np.empty((0,3))
        for edge in edges:
            for axis in np.eye(3):
                n=np.cross(edge,axis);length=np.linalg.norm(n)
                if length>1e-9:axes.append(n/length)
        return np.array([np.r_[sign*n,np.max(points@(sign*n))] for n in axes for sign in (1,-1)])
    hull = ConvexHull(points)
    planes = [np.r_[eq[:3],-eq[3]] for eq in hull.equations]
    for axis in np.eye(3):
        planes += [np.r_[axis,np.max(points@axis)],np.r_[-axis,np.max(points@-axis)]]
    edges = {tuple(sorted((int(t[a]),int(t[b])))) for t in hull.simplices for a,b in ((0,1),(1,2),(2,0))}
    for a,b in edges:
        edge=points[b]-points[a]
        for axis in np.eye(3):
            n=np.cross(edge,axis); length=np.linalg.norm(n)
            if length<1e-8: continue
            n/=length; d=np.dot(n,points[a]); distances=points@n-d
            if distances.max()<1e-5: planes.append(np.r_[n,d])
            elif distances.min()>-1e-5: planes.append(np.r_[-n,-d])
    planes=np.asarray(planes)
    _,indices=np.unique(np.round(planes,6),axis=0,return_index=True)
    return planes[np.sort(indices)]


def brush_vertices(planes, sides):
    p=planes[sides['bevel']==0]
    points=[]
    for ids in itertools.combinations(range(len(p)),3):
        rows=p[list(ids)]
        if abs(np.linalg.det(rows[:,:3]))<1e-9: continue
        point=np.linalg.solve(rows[:,:3],rows[:,3])
        if np.all(p[:,:3]@point<=p[:,3]+1e-4): points.append(point)
    if len(points)<4: raise ValueError('Cannot reconstruct rotated brush')
    return np.unique(np.round(points,6),axis=0)

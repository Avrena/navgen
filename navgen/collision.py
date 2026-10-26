"""Contents-aware swept AABB collision over convex solids and one-sided terrain."""
from __future__ import annotations

import math
import struct
import time
from collections import Counter
from dataclasses import dataclass

import numpy as np
from .runtime import njit, prange, get_thread_id, config

from .bsp import BRUSH_MASK, NPC_MASK, Content, box_vertices, brush_vertices, convex_planes, entity_angles, parse_phy, rotation, vector
from .planes import triangle_kernel, triangle_batch

EPS = .03125
THIN_MIN = np.array([-.45,-.45,0.])
THIN_MAX = np.array([.45,.45,55.])


@njit
def clip(planes, start, end, mins, maxs, first, count):
    enter, leave, hitplane = -1e30, 1e30, -1
    startout, endout = False, False
    for k in range(first,first+count):
        d1,d2=-planes[k,3],-planes[k,3]
        for axis in range(3):
            n=planes[k,axis]
            offset=mins[axis] if n>=0 else maxs[axis]
            d1+=n*(start[axis]+offset)
            d2+=n*(end[axis]+offset)
        if d1>0: startout=True
        if d2>0: endout=True
        if d1>0 and d2>=d1: return 1.,False,False,-1
        if d1<=0 and d2<=0: continue
        if d1>d2:
            f=(d1-EPS)/(d1-d2)
            if f>enter: enter=f;hitplane=k
        else:
            leave=min(leave,(d1+EPS)/(d1-d2))
    # Source traces may leave a solid; startsolid alone must not force fraction 0.
    if not startout:
        return (0. if not endout else 1.),True,not endout,-1
    if enter<=leave+1e-12 and enter>-1 and enter<1:
        return max(0.,enter),False,False,hitplane
    return 1.,False,False,-1


@njit
def trace(scene, start, end, mins, maxs, mask=BRUSH_MASK, walk=True, brute=False):
    planes,pflags,first,count,lo,hi,meta,front,blo,bhi,escape,leaf,order,stats = scene
    stats[get_thread_id()*8]+=1
    best=1.;nx=0.;ny=0.;nz=0.;ss=False;als=False;contents=0;disp=False;surf=0;hit=-1
    qlo=np.minimum(start,end)+mins-EPS
    qhi=np.maximum(start,end)+maxs+EPS
    node=0
    limit=len(order) if brute else len(blo)
    while node<limit:
        if brute:
            begin,finish=node,node+1
            node+=1
        else:
            if (blo[node,0]>qhi[0] or blo[node,1]>qhi[1] or blo[node,2]>qhi[2] or
                bhi[node,0]<qlo[0] or bhi[node,1]<qlo[1] or bhi[node,2]<qlo[2]):
                node=escape[node];continue
            begin,finish=leaf[node,0],leaf[node,1]
            node+=1
            if begin<0: continue
        for ii in range(begin,finish):
            bid=order[ii]
            if not(meta[bid,0]&mask): continue
            if walk and meta[bid,1]: continue
            if meta[bid,2] and not(mask&0x2000000): continue
            if meta[bid,3] and not(mask&2): continue
            if meta[bid,4] and not(mask&0x4000): continue
            if (lo[bid,0]>qhi[0] or lo[bid,1]>qhi[1] or lo[bid,2]>qhi[2] or
                hi[bid,0]<qlo[0] or hi[bid,1]<qlo[1] or hi[bid,2]<qlo[2]): continue
            isdisp=meta[bid,5]==2
            if isdisp:
                motion=0.;back=-front[bid,3]
                for axis in range(3):
                    n=front[bid,axis]
                    motion+=n*(end[axis]-start[axis])
                    back+=n*(start[axis]+(maxs[axis] if n>=0 else mins[axis]))
                if motion>1e-9 or back < -EPS: continue
            fraction,startsolid,allsolid,plane=clip(planes,start,end,mins,maxs,first[bid],count[bid])
            ss=ss or startsolid
            als=als or allsolid
            if fraction<best:
                best=fraction;contents=meta[bid,0];disp=isdisp;hit=bid
                if plane>=0:
                    nx,ny,nz=planes[plane,0],planes[plane,1],planes[plane,2]
                    surf=pflags[plane]
                elif isdisp:
                    nx,ny,nz=front[bid,0],front[bid,1],front[bid,2]
            # Fractions cannot improve on zero, ties keep the first hit, and
            # both accumulated solid flags are already true.
            if best==0. and ss and als:
                return best,nx,ny,nz,ss,als,contents,disp,surf,hit
    return best,nx,ny,nz,ss,als,contents,disp,surf,hit


@njit(parallel=True)
def batch_trace(scene,starts,ends,mins,maxs,mask=BRUSH_MASK,walk=True,brute=False):
    out=np.empty((len(starts),10))
    for i in prange(len(starts)):
        r=trace(scene,starts[i],ends[i],mins[i],maxs[i],mask,walk,brute)
        out[i,0]=r[0];out[i,1]=r[1];out[i,2]=r[2];out[i,3]=r[3]
        out[i,4]=r[4];out[i,5]=r[5];out[i,6]=r[6];out[i,7]=r[7];out[i,8]=r[8];out[i,9]=r[9]
    return out


@njit
def blocked_vertical_span(scene,start,end,mins,maxs,hit,limit):
    """Prove a contiguous range blocked by one convex, without epsilon guesses.

    A fixed interior point of the expanded convex witnesses a collision along
    every translated segment. Only strictly interior witnesses are accepted;
    touching, one-sided terrain and degenerate intersections fall back to trace.
    """
    planes,pflags,first,count,lo,hi,meta,front,blo,bhi,escape,leaf,order,stats=scene
    if hit<0 or meta[hit,5]==2:return 0
    enter=0.;leave=1.
    for k in range(first[hit],first[hit]+count[hit]):
        a=-planes[k,3];b=a
        for axis in range(3):
            n=planes[k,axis];offset=mins[axis] if n>=0 else maxs[axis]
            a+=n*(start[axis]+offset);b+=n*(end[axis]+offset)
        if a>0 and b>=a:return 0
        if a<=0 and b<=0:continue
        if a>b:enter=max(enter,a/(a-b))
        else:leave=min(leave,a/(a-b))
    if leave-enter<1e-8:return 0
    witness=start+(end-start)*((enter+leave)*.5)
    # Saved PHY bounds can be tighter than the convex planes. The original
    # broad phase is authoritative too, including its epsilon padding.
    span=min(float(limit),hi[hit,2]-(min(start[2],end[2])+mins[2]-EPS)-1e-7)
    for k in range(first[hit],first[hit]+count[hit]):
        distance=-planes[k,3]
        for axis in range(3):
            n=planes[k,axis];offset=mins[axis] if n>=0 else maxs[axis]
            distance+=n*(witness[axis]+offset)
        # clip moves leaving fractions by EPS too. Merely being physically
        # inside a thin corner does not prove the epsilon-adjusted intervals
        # overlap; the witness must lie at least EPS inside every plane.
        if distance>=-EPS-1e-7:return 0
        if planes[k,2]>0:span=min(span,(-distance-EPS-1e-7)/planes[k,2])
    result=max(0,int(math.floor(span)))
    if result==0:return 0
    if min(start[2]+result,end[2]+result)+mins[2]-EPS>hi[hit,2]:return 0
    witness[2]+=result
    # Recheck the upper endpoint with the same summation as clip. Convexity
    # proves every intermediate height; this check contains division rounding.
    for k in range(first[hit],first[hit]+count[hit]):
        distance=-planes[k,3]
        for axis in range(3):
            n=planes[k,axis];offset=mins[axis] if n>=0 else maxs[axis]
            distance+=n*(witness[axis]+offset)
        if distance>=-EPS-1e-7:return 0
    return result


def triangle_planes(points):
    planes,front=triangle_kernel(points)
    return (planes,front) if len(planes) else (None,None)


def build_bvh(lo,hi):
    boundslo=[];boundshi=[];escape=[];leaf=[];order=[]
    centers=(lo+hi)*.5
    def build(ids):
        i=len(boundslo)
        boundslo.append(lo[ids].min(0));boundshi.append(hi[ids].max(0));escape.append(0);leaf.append((-1,-1))
        if len(ids)<=8:
            first=len(order);order.extend(ids);leaf[i]=(first,len(order))
        else:
            axis=int(np.argmax(np.ptp(centers[ids],axis=0)))
            ids=ids[np.argsort(centers[ids,axis],kind='stable')];mid=len(ids)//2
            build(ids[:mid]);build(ids[mid:])
        escape[i]=len(boundslo)
    if len(lo): build(np.arange(len(lo),dtype=np.int32))
    return (np.asarray(boundslo,dtype=np.float64).reshape(-1,3),np.asarray(boundshi,dtype=np.float64).reshape(-1,3),
            np.asarray(escape,dtype=np.int32),np.asarray(leaf,dtype=np.int32).reshape(-1,2),np.asarray(order,dtype=np.int32))


class SceneBuilder:
    def __init__(self):
        self.planes=[];self.flags=[];self.first=[];self.count=[];self.lo=[];self.hi=[];self.meta=[];self.front=[]
        self.nplanes=0

    def add(self,planes,lo,hi,contents=1,walkable=False,monster=False,transparent=False,push=False,kind=0,flags=None,front=None):
        if not len(planes): return
        if not np.isfinite(planes).all(): raise ValueError('Non-finite collision plane')
        self.first.append(self.nplanes);self.count.append(len(planes));self.nplanes+=len(planes)
        self.planes.append(planes)
        self.flags.append(np.zeros(len(planes),np.int64) if flags is None else flags)
        self.lo.append(lo);self.hi.append(hi)
        self.meta.append((contents,int(walkable),int(monster),int(transparent),int(push),kind))
        self.front.append(np.zeros(4) if front is None else front)

    def pack(self):
        lo=np.asarray(self.lo,dtype=np.float64).reshape(-1,3);hi=np.asarray(self.hi,dtype=np.float64).reshape(-1,3)
        return (np.concatenate(self.planes) if self.planes else np.empty((0,4)),
                np.concatenate(self.flags) if self.flags else np.empty(0,dtype=np.int64),
                np.asarray(self.first,np.int64),np.asarray(self.count,np.int64),lo,hi,
                np.asarray(self.meta,np.int64).reshape(-1,6),np.asarray(self.front,np.float64).reshape(-1,4),*build_bvh(lo,hi),np.zeros(config.NUMBA_NUM_THREADS*8,np.int64))


# SDK bmodels.cpp, buttons.cpp, doors.h, func_movelinear.cpp and trains.h:
# class -> (spawnflags making it nonsolid, ignored by generation's walkable filter).
BRUSH_CLASSES={
    'func_wall':(0,False),'func_wall_toggle':(1,False),'func_conveyor':(2,False),
    'func_rotating':(64,False),'func_movelinear':(8,False),'momentary_door':(8,False),
    'func_button':(0,False),'func_rot_button':(1,False),'momentary_rot_button':(1,False),
    'func_door':(8,True),'func_door_rotating':(8,True),'func_water':(0,False),
    'func_water_analog':(0,False),'func_plat':(0,False),'func_platrot':(0,False),
    'func_train':(8,False),'func_tracktrain':(8,False),'func_trackchange':(0,False),
    'func_trackautochange':(0,False),'func_physbox':(0,True),'func_pushable':(0,True),
    'func_playerinfected_clip':(0,True),
}
NONSOLID_CLASSES={'func_illusionary','func_viscluster','func_detail','func_occluder',
    'func_smokevolume','func_dustmotes','func_dustcloud','func_precipitation',
    'func_precipitation_blocker','func_vehicleclip','func_clip_vphysics','func_traincontrols'}


def entity_rules(e):
    e={k.lower():v for k,v in e.items()};cls=e.get('classname','');flags=int(e.get('spawnflags','0'))
    if cls.startswith(('trigger_','func_areaportal')) or cls in NONSOLID_CLASSES:return False,False
    if cls in ('func_brush','func_reflective_glass','func_monitor'):
        solidity=int(e.get('solidity','0'))
        active=solidity!=1 and (solidity==2 or not int(e.get('startdisabled','0')))
        return active,cls=='func_brush' and solidity==0 and active
    if cls=='func_lod':
        # func_lod.cpp:105: Solid != 0 adds FSOLID_NOT_SOLID (inverse of its name).
        return not int(e.get('solid','0')),False
    if cls in ('func_breakable','func_breakable_surf'):
        return True,cls=='func_breakable_surf' or (float(e.get('health','0'))!=0 and not(flags&1))
    if cls in BRUSH_CLASSES:
        nonsolid,walk=BRUSH_CLASSES[cls]
        return not bool(flags&nonsolid),walk
    return None,False


@dataclass
class Collision:
    scene: tuple
    counts: dict
    unresolved: list
    content_audit: dict=None

    def trace(self,start,end,mins=THIN_MIN,maxs=THIN_MAX,mask=BRUSH_MASK,walk=True):
        return trace(self.scene,np.asarray(start,dtype=np.float64),np.asarray(end,dtype=np.float64),
                     np.asarray(mins,dtype=np.float64),np.asarray(maxs,dtype=np.float64),mask,walk)

    @classmethod
    def load(cls,bsp,game,missing_phy='skip',progress=print):
        begin=time.perf_counter();builder=SceneBuilder();counts=Counter();unresolved=[];unknown=Counter()
        for bid in bsp.model_brushes(0):
            contents=int(bsp.brushes[bid]['contents'])
            planes,flags,lo,hi,sides=bsp.brush(bid)
            builder.add(planes,lo,hi,contents,flags=flags);counts['world_brushes']+=1
        for e in bsp.entities:
            model=e.get('model','')
            if not model.startswith('*'): continue
            active,walkable=entity_rules(e)
            if active is None:unknown[e.get('classname','')]+=1
            if not active: continue
            org=vector(e.get('origin',''));angles=entity_angles(e);rot=rotation(angles)
            for bid in bsp.model_brushes(int(model[1:])):
                planes,flags,lo,hi,sides=bsp.brush(bid)
                if np.abs(angles).max()>1e-8:
                    points=brush_vertices(planes,sides)@rot.T+org
                    planes=convex_planes(points);lo=points.min(0);hi=points.max(0);flags=None
                else:
                    planes[:,3]+=planes[:,:3]@org;lo+=org;hi+=org
                contents=int(bsp.brushes[bid]['contents'])
                builder.add(planes,lo,hi,contents,walkable,False,int(e.get('rendermode','0'))!=0,True,flags=flags)
                counts['entity_brushes']+=1
        progress(f'collision brushes: {counts["world_brushes"]} world, {counts["entity_brushes"]} entity ({time.perf_counter()-begin:.2f}s)',flush=True)
        content=Content(bsp,game)
        counts['unknown_brush_entities']=sum(unknown.values())
        unresolved.extend(f'unknown brush class skipped ({n} entities): {name}' for name,n in sorted(unknown.items()))
        unresolved.extend(content.archive_errors)
        props=list(bsp.static_props())
        models={}
        for name in sorted({p[0] for p in props}):
            mdl=content.read(name);phy=content.read(name[:-4]+'.phy')
            if mdl is None:
                unresolved.append(f'missing MDL: {name}');models[name]=(None,None);counts['missing_models']+=1;continue
            if len(mdl)<128 or mdl[:4]!=b'IDST': raise ValueError(f'Invalid MDL: {name}')
            bounds=(np.array(struct.unpack_from('<3f',mdl,104)),np.array(struct.unpack_from('<3f',mdl,116)))
            try:pieces=parse_phy(phy) if phy is not None else None
            except (ValueError,struct.error) as exc:raise ValueError(f'PHY {name}: {exc}') from exc
            models[name]=(bounds,pieces)
            if pieces is not None: counts['phy_models']+=1;counts['phy_convex_pieces']+=len(pieces)
            else: counts['models_without_phy']+=1
        progress(f'collision models: {len(models)}, PHY parsed {counts["phy_models"]} ({time.perf_counter()-begin:.2f}s)',flush=True)
        missing_instances=Counter()
        oriented={}
        for name,solid,org,angles,scale in props:
            counts['static_props']+=1
            if solid==0: continue
            bounds,pieces=models[name]
            if bounds is None: continue
            if solid==2:
                # SOLID_BBOX is world-axis-aligned; entity angles do not rotate its hull.
                points=box_vertices(bounds[0]*scale+org,bounds[1]*scale+org)
                builder.add(convex_planes(points),points.min(0),points.max(0),kind=1);counts['bbox_props']+=1
                continue
            if solid!=6:
                unresolved.append(f'unsupported static prop solid {solid}: {name}');continue
            if pieces is None:
                missing_instances[name]+=1
                if missing_phy=='skip': continue
                pieces=[box_vertices(*bounds)]
            # Map A measured 123 s rebuilding repeated prop hulls. Reuse orientations within this run only.
            key=(name,float(scale),*map(float,angles))
            if key not in oriented:
                rot=rotation(angles);shapes=[]
                for piece in pieces:
                    points=(piece*scale)@rot.T
                    shapes.append((convex_planes(points),points.min(0),points.max(0)))
                oriented[key]=shapes
            for local_planes,lo,hi in oriented[key]:
                planes=local_planes.copy();planes[:,3]+=planes[:,:3]@org
                builder.add(planes,lo+org,hi+org,kind=1)
                counts['prop_convex_instances']+=1
        counts['unique_prop_orientations']=len(oriented)
        counts['missing_mdl_solid_instances']=sum(1 for name,solid,*_ in props if solid and models[name][0] is None)
        counts['excluded_nonbrush_model_entities']=sum(1 for e in bsp.entities if e.get('model','').lower().endswith('.mdl'))
        counts['missing_phy_solid_instances']=sum(missing_instances.values())
        counts['missing_phy_skipped_instances']=counts['missing_phy_solid_instances'] if missing_phy=='skip' else 0
        counts['missing_phy_bbox_instances']=counts['missing_phy_solid_instances'] if missing_phy=='bbox' else 0
        unresolved.extend(f'missing PHY ({missing_instances[name]} solid instances, {missing_phy}): {name}' for name in sorted(missing_instances))
        progress(f'collision static props: {counts["prop_convex_instances"]} convex instances, missing PHY {counts["missing_phy_solid_instances"]} ({missing_phy}; {time.perf_counter()-begin:.2f}s)',flush=True)
        terrain=list(bsp.displacements())
        triangles=np.asarray([t[0] for t in terrain],np.float64).reshape(-1,3,3)
        planes,fronts,sizes=triangle_batch(triangles)
        for i,(points,contents,tags) in enumerate(terrain):
            if not sizes[i]:continue
            builder.add(planes[i,:sizes[i]],points.min(0),points.max(0),contents,kind=2,front=fronts[i])
            counts['displacement_triangles']+=1
        progress(f'collision terrain: {counts["displacement_triangles"]} triangles; building BVH',flush=True)
        scene=builder.pack()
        counts['collision_objects']=len(scene[2]);counts['collision_planes']=len(scene[0])
        counts['build_seconds']=time.perf_counter()-begin
        counts['content_conflicts']=len(content.conflicts)
        audit={'selected_reads':dict(content.used),'conflicts':content.conflicts,'archive_errors':content.archive_errors,
               'gma_archives':sum(kind=='gma' for kind,*_ in content.sources),'unknown_brush_classes':dict(unknown)}
        return cls(scene,dict(counts),unresolved,audit)

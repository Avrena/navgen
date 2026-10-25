"""GMod nav_quicksave=2 hiding analysis, after final area/link construction.

SDK 2013 nav_area.cpp, confirmed in the installed x64 server.dll:
ComputeHidingSpots 0x99d50, FindPositionInArea 0x9f110, cover 0xa18a0.
Positions retain the corner's Z; the inset does not interpolate area height.
"""
import math
from collections import Counter

import numpy as np
from .runtime import njit, prange, trace_count

from .bsp import BRUSH_MASK
from .collision import trace

IN_COVER,EXPOSED=1,8
JUMP,DONT_HIDE=2,0x200


def spot_position(area,corner):
    """SDK fallback order: inset, Y center, X center, both, 1 unit, corner."""
    nw,se=np.asarray(area['nw'],np.float32),np.asarray(area['se'],np.float32)
    corners=(nw,np.array([se[0],nw[1],area['ne_z']],np.float32),
             se,np.array([nw[0],se[1],area['sw_z']],np.float32))
    origin=corners[corner];sx,sy=((1,1),(-1,1),(-1,-1),(1,-1))[corner]
    half=(se-nw)*np.float32(.5)
    for dx,dy in ((12.5,12.5),(12.5,half[1]),(half[0],12.5),(half[0],half[1]),(1,1),(0,0)):
        pos=origin+np.array([dx*sx,dy*sy,0],np.float32)
        if nw[0]<=pos[0]<=se[0] and nw[1]<=pos[1]<=se[1]:return pos
    return origin.copy()


def area_spots(area,by_id):
    """Candidate positions depend on reciprocal topology, not wall rays."""
    if area['attr']&(JUMP|DONT_HIDE):return []
    counts=[0]*4
    for d,links in enumerate(area['links']):
        axis=0 if d%2==0 else 1
        lo,hi=np.float32(999999.9),np.float32(-999999.9)
        for target in links:
            other=by_id[target]
            if other['attr']&JUMP or area['id'] not in other['links'][(d+2)%4]:continue
            lo=min(lo,np.float32(other['nw'][axis]));hi=max(hi,np.float32(other['se'][axis]))
        first,last=((0,1),(1,2),(3,2),(0,3))[d]
        if np.float32(lo-np.float32(area['nw'][axis]))>=20:counts[first]+=1
        if np.float32(np.float32(area['se'][axis])-hi)>=20:counts[last]+=1
    spots=[]
    for corner,count in enumerate(counts):
        if count!=2:continue
        pos=spot_position(area,corner)
        # SDK uses a 3D distance and a strict <30 test, within this area only.
        if corner and any(np.sum((other-pos)**2,dtype=np.float32)<900 for other in spots):continue
        spots.append(pos)
    return spots


@njit(parallel=True)
def cover_flags(scene,positions):
    """Point rays; ordinary entity filtering, not the generation walk-through filter."""
    flags=np.empty(len(positions),np.uint8);zero=np.zeros(3)
    inc=np.float32(math.pi/8)
    for i in prange(len(positions)):
        start=np.empty(3);end=np.empty(3)
        start[0]=positions[i,0];start[1]=positions[i,1]
        start[2]=np.float32(positions[i,2]+np.float32(35.5))
        end[:]=start;end[2]=np.float32(start[2]+np.float32(20))
        if trace(scene,start,end,zero,zero,BRUSH_MASK,False)[0]!=1:
            flags[i]=IN_COVER;continue
        count=0;angle=np.float32(0)
        while angle<2*math.pi:
            end[0]=np.float32(np.float32(start[0])+np.float32(100)*np.float32(math.cos(float(angle))))
            end[1]=np.float32(np.float32(start[1])+np.float32(100)*np.float32(math.sin(float(angle))))
            end[2]=np.float32(start[2]+np.float32(35.5))
            if trace(scene,start,end,zero,zero,BRUSH_MASK,False)[0]!=1:count+=1
            angle=np.float32(angle+inc)
        flags[i]=IN_COVER if count>=8 else EXPOSED
    return flags


def compute_hiding_spots(nav,collision):
    """Recompute deterministically in saved area order, with IDs starting at zero."""
    by_id={a['id']:a for a in nav['areas']}
    per_area=[area_spots(a,by_id) for a in nav['areas']]
    positions=np.asarray([p for spots in per_area for p in spots],np.float32).reshape(-1,3)
    before=trace_count(collision.scene);flags=cover_flags(collision.scene,positions)
    next_id=0
    for area,spots in zip(nav['areas'],per_area):
        area['hiding']=[]
        for pos in spots:
            area['hiding'].append((next_id,tuple(map(float,pos)),int(flags[next_id])));next_id+=1
    # Full generation in GMod sets this after the quicksave analysis states.
    nav['analyzed']=1
    return {'total':next_id,'in_cover':int(np.count_nonzero(flags==IN_COVER)),
            'exposed':int(np.count_nonzero(flags==EXPOSED)),
            'areas_with_spots':sum(bool(s) for s in per_area),
            'spots_per_area':dict(sorted(Counter(map(len,per_area)).items())),
            'traces':trace_count(collision.scene)-before,'analysis_mode':'GMod nav_quicksave 2'}

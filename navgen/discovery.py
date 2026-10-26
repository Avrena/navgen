"""Bounded, deterministic BSP entity-floor seed discovery."""
from collections import Counter
import struct

import numpy as np
from .bsp import vector,entity_angles,rotation,box_vertices
from .collision import trace
from .geometry import MINS,DUCK


def ladder_specs(bsp):
    result=[];skipped=[]
    for entity in bsp.entities:
        e={k.lower():v for k,v in entity.items()};cls=e.get('classname','')
        if cls not in ('info_ladder','func_useableladder'):continue
        source={'class':cls,'hammerid':e.get('hammerid')}
        if int(e.get('startdisabled','0')) or int(e.get('spawnflags','0')):
            skipped.append({**source,'reason':'disabled or non-default ladder flags'});continue
        if e.get('parentname'):
            skipped.append({**source,'reason':'parented ladder requires runtime transform'});continue
        try:
            if cls=='info_ladder':
                lo=np.array([float(e['mins.'+axis]) for axis in 'xyz']);hi=np.array([float(e['maxs.'+axis]) for axis in 'xyz'])
                if not np.isfinite(np.r_[lo,hi]).all() or np.any(hi<=lo):raise ValueError('invalid bounds')
                axis=int(np.argmin(hi[:2]-lo[:2]));bottom=(lo+hi)*.5;top=bottom.copy();bottom[2]=lo[2];top[2]=hi[2]
                result.append({**source,'bottom':bottom,'top':top,'width':float(hi[1-axis]-lo[1-axis]),'axis':axis,'lo':lo,'hi':hi})
            else:
                # SDK CFuncLadder::Spawn subtracts origin AFTER reading these
                # fields. BSP point0/point1 are already world coordinates.
                points=sorted([vector(e['point0']),vector(e['point1'])],key=lambda p:p[2])
                if not np.isfinite(points).all() or points[1][2]<=points[0][2]:raise ValueError('invalid endpoints')
                result.append({**source,'bottom':points[0],'top':points[1],'width':25.,'axis':None})
        except (KeyError,ValueError):skipped.append({**source,'reason':'invalid ladder geometry'})
    return result,skipped


def entity_sources(bsp,base):
    sources=[(np.asarray(p), 'base') for p in base]
    for e in bsp.entities:
        cls=e.get('classname','');origin=vector(e.get('origin',''))
        if cls.startswith('info_player_') or cls=='info_ladder_dismount':sources.append((origin,cls))
        elif cls in ('func_door','func_door_rotating','prop_door_rotating','func_button','func_rot_button','momentary_rot_button'):
            model=e.get('model','')
            if model.startswith('*'):
                record=bsp.models[int(model[1:])];points=box_vertices(record[:3],record[3:6])@rotation(entity_angles(e)).T+origin
                lo,hi=points.min(0),points.max(0)
            else:
                # The SDK collision model excludes non-brush model entities.
                # Probe around the pivot; do not invent dynamic MDL collision.
                lo=origin-np.array([24.,24.,0.]);hi=origin+np.array([24.,24.,64.])
            center=(lo+hi)*.5
            for axis in (0,1):
                for sign in (-1,1):
                    p=center.copy();p[axis]=(lo[axis]-24) if sign<0 else (hi[axis]+24);sources.append((p,cls))
    specs,skipped=ladder_specs(bsp)
    for spec in specs:
        for name in ('bottom','top'):
            for dx,dy in ((32.,0.),(-32.,0.),(0.,32.),(0.,-32.)):
                p=spec[name]+np.array([dx,dy,36.]);sources.append((p,spec['class']+'_'+name))
    return sources,specs,skipped


def leaf_area(bsp,point):
    node=bsp.models[0][9]
    while node>=0:
        plane,front,back=struct.unpack_from('<3i',bsp.nodes,node*32);p=bsp.planes[plane]
        node=front if np.dot(p['n'],point)>p['d'] else back
    return struct.unpack_from('<H',bsp.leaves,(-1-node)*bsp.leaf_stride+6)[0]&511


def discover_seeds(bsp,collision,base,slope=.7,max_seeds=4096,max_probes=200000,progress=print):
    sources,specs,skipped=entity_sources(bsp,base);scene=collision.scene;accepted=[];records=[]
    probes=set();buckets={};counts=Counter();lo=np.asarray(bsp.models[0][:3]);hi=np.asarray(bsp.models[0][3:6])
    def nearby(index,p,xy,z):
        key=tuple(np.floor(p/[xy,xy,z]).astype(int))
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                for dz in (-1,0,1):
                    for other in index.get((key[0]+dx,key[1]+dy,key[2]+dz),()):
                        if abs(other[2]-p[2])<=z and np.linalg.norm(other[:2]-p[:2])<=xy:return True
        return False
    def index_add(index,p,xy,z):index.setdefault(tuple(np.floor(p/[xy,xy,z]).astype(int)),[]).append(p)
    def probe(p,source):
        if len(accepted)>=max_seeds or counts['probes']>=max_probes:return
        if np.any(p<lo) or np.any(p>hi):counts['outside_bounds']+=1;return
        key=tuple(np.floor(p/[32.,32.,16.]).astype(int))
        if key in probes:return
        probes.add(key);counts['probes']+=1
        floor=None
        for rise in (2.,0.):
            start=p.copy();start[2]+=rise;end=p.copy();end[2]-=3000.
            hit=trace(scene,start,end,MINS,DUCK)
            if hit[4] or hit[0]>=1 or hit[3]<slope or hit[8]&134:continue
            point=start+(end-start)*hit[0];point[2]+=.01
            body=trace(scene,point,point,MINS,DUCK)
            if body[4] or body[5]:continue
            floor=point;break
        if floor is None:counts['rejected_floor']+=1;return
        if nearby(buckets,floor,24.,18.):counts['duplicate_floor']+=1;return
        index_add(buckets,floor,24.,18.);accepted.append(floor);records.append({'point':floor.tolist(),'source':source,'leaf_area':leaf_area(bsp,floor)})
    for point,source in sources:probe(point,source)
    all_seeds=np.unique(np.vstack([np.asarray(base).reshape(-1,3),np.asarray(accepted).reshape(-1,3)]),axis=0)
    sky=[{'origin':vector(e.get('origin','')).tolist(),'leaf_area':leaf_area(bsp,vector(e.get('origin','')))} for e in bsp.entities if e.get('classname')=='sky_camera']
    summary={'counts':dict(counts),'accepted':len(accepted),'total_seeds':len(all_seeds),'max_seeds':max_seeds,'max_probes':max_probes,'nearby_floors':False,
             'limit_reached':len(accepted)>=max_seeds or counts['probes']>=max_probes,
             'sources':dict(Counter(r['source'] for r in records)),'height_bands':dict(Counter(str(round(r['point'][2]/128)*128) for r in records)),
             'accepted_floors':records,'sky_cameras':sky,'ladder_sources':len(specs),'skipped_ladder_sources':skipped}
    progress(f'expanded seeds: {len(accepted):,} floors from {counts["probes"]:,} probes; limit reached={summary["limit_reached"]}',flush=True)
    return all_seeds,summary,specs

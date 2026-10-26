"""Conservative rectangle contraction of an SDK navigation mesh."""
from collections import Counter,deque
import copy,time,struct
import numpy as np
from .areas import corner_points,surface_normals,portal_probe
from .compare import boxes
from .pathing import native_portal
from .bsp import NPC_MASK,box_vertices,rotation,entity_angles,vector
from .collision import trace

BLOCKED_FLAGS=8192|256|0x80000000

def union_box(a,b,tolerance=.5,max_size=800.):
    """Complete common edge, outer corners and cumulative-height compatibility."""
    if a[1]==b[1] and a[3]==b[3] and (a[2]==b[0] or b[2]==a[0]):
        if b[2]==a[0]:a,b=b,a
        if max(abs(a[5]-b[4]),abs(a[6]-b[7]))>min(.5,tolerance):return None
        result=np.array([a[0],a[1],b[2],a[3],a[4],b[5],b[6],a[7]])
    elif a[0]==b[0] and a[2]==b[2] and (a[3]==b[1] or b[3]==a[1]):
        if b[3]==a[1]:a,b=b,a
        if max(abs(a[7]-b[4]),abs(a[6]-b[5]))>min(.5,tolerance):return None
        result=np.array([a[0],a[1],a[2],b[3],a[4],a[5],b[6],b[7]])
    else:return None
    width,height=result[2:4]-result[:2]
    if max(width,height)>max_size or width*height>300000:return None
    na=surface_normals(a);nb=surface_normals(b)
    if min(x@y for x in na for y in nb)<.99:return None
    return result

def fits(box,points,tolerance):
    u=(points[:,0]-box[0])/(box[2]-box[0]);v=(points[:,1]-box[1])/(box[3]-box[1])
    z=(box[4]+u*(box[5]-box[4]))*(1-v)+(box[7]+u*(box[6]-box[7]))*v
    return bool(np.max(np.abs(z-points[:,2]))<=tolerance+1e-8)

def edge_class(a,b,d):
    p,_=native_portal(a,b,d);q,_=native_portal(b,a,(d+2)%4)
    dz=np.float32(q[2]-p[2]);return dz>=18.,dz>=58.,dz < -200.

def merge_nav(nav,water=None,locked_ids=(),tolerance=.5,max_size=125.,scene=None,slope=.7):
    if not np.isfinite(tolerance) or tolerance<0:raise ValueError('Merge tolerance must be finite and nonnegative')
    if not np.isfinite(max_size) or max_size<=0:raise ValueError('Merge size must be finite and positive')
    start=time.perf_counter();result=copy.deepcopy(nav);areas={a['id']:a for a in result['areas']}
    bounds={a['id']:b for a,b in zip(result['areas'],boxes(result))};points={i:corner_points(b) for i,b in bounds.items()}
    outgoing={i:[set(ls) for ls in a['links']] for i,a in areas.items()};incoming={i:set() for i in areas}
    for i,links in outgoing.items():
        for d,ls in enumerate(links):
            for j in ls:incoming[j].add((i,d))
    groups={i:[i] for i in areas};mapping={i:i for i in areas};locked=set(locked_ids);wet={};rejected=Counter();merged=0
    def center(b):return np.array([(b[0]+b[2])/2,(b[1]+b[3])/2,(b[4]+b[6])/2])
    for i,a in areas.items():
        if a['attr']&BLOCKED_FLAGS or any(a['ladders']) or a.get('blocked') or a.get('damaging'):locked.add(i)
        wet[i]=water(center(bounds[i])) if water else a.get('underwater',False)
        if wet[i] is None:locked.add(i)
    queue=deque(sorted(areas));queued=set(queue)
    while queue:
        i=queue.popleft();queued.discard(i)
        if i not in areas or i in locked:continue
        a=areas[i]
        for j in sorted(set.union(*outgoing[i])):
            if j==i or j not in areas or j in locked:continue
            b=areas[j]
            if a['attr']!=b['attr'] or a['place']!=b['place'] or wet[i]!=wet[j]:continue
            if not any(i in links for links in outgoing[j]):continue
            if a['light']!=b['light']:continue
            box=union_box(bounds[i],bounds[j],tolerance,max_size)
            if box is None:continue
            ai,bj=bounds[i],bounds[j]
            d=1 if ai[2]==bj[0] else (3 if bj[2]==ai[0] else (2 if ai[3]==bj[1] else 0))
            if j not in outgoing[i][d] or i not in outgoing[j][(d+2)%4]:continue
            if scene is not None and (portal_probe(scene,ai,bj,d,slope,True)[0] not in (1,2,3) or
                                      portal_probe(scene,bj,ai,(d+2)%4,slope,True)[0] not in (1,2,3)):
                rejected['shared_edge_probe']+=1;continue
            old=np.concatenate((points[i],points[j]))
            if not fits(box,old,tolerance):rejected['original_corner_error']+=1;continue
            compatible=True
            for oldid in (i,j):
                for d,ls in enumerate(outgoing[oldid]):
                    for k in ls-{i,j}:
                        if edge_class(bounds[oldid],bounds[k],d)!=edge_class(box,bounds[k],d):compatible=False;break
                    if not compatible:break
                if not compatible:break
                for k,d in incoming[oldid]:
                    if k not in (i,j) and edge_class(bounds[k],bounds[oldid],d)!=edge_class(bounds[k],box,d):compatible=False;break
                if not compatible:break
            if not compatible:rejected['native_height_class']+=1;continue
            if scene is not None:
                for oldid in (i,j):
                    for d,ls in enumerate(outgoing[oldid]):
                        for k in ls-{i,j}:
                            if portal_probe(scene,box,bounds[k],d,slope,True)[0] not in (1,2,3):compatible=False;break
                        if not compatible:break
                    if not compatible:break
                    for k,d in incoming[oldid]:
                        if k not in (i,j) and portal_probe(scene,bounds[k],box,d,slope,True)[0] not in (1,2,3):compatible=False;break
                    if not compatible:break
                if not compatible:rejected['frontier_probe']+=1;continue
            if water and water(center(box))!=wet[i]:rejected['merged_water']+=1;continue
            # Preserve each directed link and its stored compass direction.
            newout=[(outgoing[i][d]|outgoing[j][d])-{i,j} for d in range(4)]
            newin={(k,d) for k,d in incoming[i]|incoming[j] if k not in (i,j)}
            for oldid in (i,j):
                for d,ls in enumerate(outgoing[oldid]):
                    for k in ls:incoming[k].discard((oldid,d))
                for k,d in list(incoming[oldid]):outgoing[k][d].discard(oldid)
            outgoing[i]=newout;incoming[i]=newin
            for d,ls in enumerate(newout):
                for k in ls:incoming[k].add((i,d))
            for k,d in newin:outgoing[k][d].add(i)
            bounds[i]=box;points[i]=corner_points(box) if np.ptp(old[:,2])==0 else old
            for k in groups[j]:mapping[k]=i
            groups[i]+=groups.pop(j);areas.pop(j);outgoing.pop(j);incoming.pop(j);bounds.pop(j);points.pop(j);wet.pop(j)
            a['nw']=tuple(box[[0,1,4]]);a['se']=tuple(box[[2,3,6]]);a['ne_z']=float(box[5]);a['sw_z']=float(box[7])
            a['hiding']=[];a['encounters']=[];a['visible']=[];a['inherit_vis']=0
            merged+=1
            affected={i}|set.union(*newout)|{k for k,d in newin}
            for k in sorted(affected):
                if k not in queued:queue.append(k);queued.add(k)
            break
    expected={(mapping[a['id']],mapping[j],d) for a in nav['areas'] for d,links in enumerate(a['links']) for j in links if mapping[a['id']]!=mapping[j]}
    actual={(i,j,d) for i,links in outgoing.items() for d,ls in enumerate(links) for j in ls}
    if expected!=actual:raise ValueError('Merge changed the directed graph contraction')
    changed=sum(len(groups[i])>1 or len(groups[j])>1 for i,j,d in actual)
    link_validation={'directed_contraction_verified':True,'changed_links':changed,'changed_links_geometrically_verified':changed if scene is not None else 0,
                     'inherited_unchanged_links':len(actual)-changed,'policy':'Changed frontier links require SDK movement/drop/portal witnesses; unchanged SDK exceptions are retained.'}
    result['areas']=[areas[i] for i in sorted(areas)]
    for a in result['areas']:a['links']=[sorted(ls) for ls in outgoing[a['id']]]
    return result,{'source_areas':len(nav['areas']),'merged_areas':len(areas),'merges':merged,'locked':len(locked),
        'tolerance':tolerance,'max_size':max_size,'rejected':dict(rejected),'seconds':time.perf_counter()-start,
        'mapping':mapping,'groups':groups,'link_validation':link_validation}


def ground_height(scene,point):
    """CNavMesh::GetGroundHeight trace sequence against the offline scene."""
    point=np.asarray(point,float);start=point.copy();end=point.copy();start[2]+=35.501;end[2]-=10000.
    zero=np.zeros(3)
    while end[2]-point[2]<100.:
        hit=trace(scene,start,end,zero,zero,NPC_MASK,True);z=start[2]+(end[2]-start[2])*hit[0]
        if not hit[4] and (hit[0]==1 or start[2]-z>=35.5):return float(z)
        end[2]=start[2] if hit[4] else z;start[2]=end[2]+35.501
    return None

def world_contents(bsp,point):
    node=bsp.models[0][9]
    while node>=0:
        plane,front,back=struct.unpack_from('<3i',bsp.nodes,node*32);p=bsp.planes[plane]
        node=front if np.dot(p['n'],point)>p['d'] else back
    return struct.unpack_from('<I',bsp.leaves,(-1-node)*bsp.leaf_stride)[0]

def merge_environment(bsp,collision,nav):
    cache={}
    def water(center):
        key=tuple(center)
        if key not in cache:
            z=ground_height(collision.scene,center)
            if z is None:cache[key]=None
            else:
                point=np.asarray(center,float).copy();point[2]=z+1.
                cache[key]=bool(world_contents(bsp,point)&0x4030)
        return cache[key]
    locked=set();records=[];bounds=boxes(nav)
    for entity in bsp.entities:
        cls=entity.get('classname','')
        if cls not in ('trigger_hurt','func_nav_blocker','func_nav_avoid','func_nav_cost','func_water','func_water_analog'):continue
        model=entity.get('model','')
        if not model.startswith('*'):continue
        record=bsp.models[int(model[1:])];points=box_vertices(record[:3],record[3:6])@rotation(entity_angles(entity)).T+vector(entity.get('origin',''))
        lo,hi=points.min(0),points.max(0)
        # Conservatively keep all areas touching a runtime hazard/cost/water
        # volume, including disabled sources whose live state is not knowable.
        keep=(bounds[:,0]<=hi[0])&(bounds[:,2]>=lo[0])&(bounds[:,1]<=hi[1])&(bounds[:,3]>=lo[1])
        keep&=(bounds[:,4:].min(1)<=hi[2])&(bounds[:,4:].max(1)+71>=lo[2])
        ids=[nav['areas'][i]['id'] for i in np.flatnonzero(keep)];locked.update(ids)
        records.append({'class':cls,'hammerid':entity.get('hammerid'),'bounds':[lo.tolist(),hi.tolist()],'locked_area_ids':ids})
    return water,locked,{'static_volume_sources':records,'runtime_state_verified':False,'water_policy':'SDK ground probe and world BSP leaf contents; runtime water entity volumes are locked'}

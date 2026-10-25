"""Compare coverage and directed grid transitions independently of area numbering."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from numba import njit
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from .areas import probe_portal
from .bsp import BSP,DEFAULT_GAME
from .collision import Collision,batch_trace
from .navio import read_nav


def boxes(nav):
    return np.array([[a['nw'][0],a['nw'][1],a['se'][0],a['se'][1],a['nw'][2],a['ne_z'],a['se'][2],a['sw_z']] for a in nav['areas']],dtype=np.float64).reshape(-1,8)


def cells(nav):
    points=[];owners=[]
    for i,b in enumerate(boxes(nav)):
        xs=np.arange(np.ceil((b[0]-12.5)/25),np.ceil((b[2]-12.5)/25))*25+12.5
        ys=np.arange(np.ceil((b[1]-12.5)/25),np.ceil((b[3]-12.5)/25))*25+12.5
        if not len(xs) or not len(ys):continue
        xx,yy=np.meshgrid(xs,ys);x=xx.ravel();y=yy.ravel()
        u=(x-b[0])/(b[2]-b[0]);v=(y-b[1])/(b[3]-b[1])
        z=(b[4]+u*(b[5]-b[4]))*(1-v)+(b[7]+u*(b[6]-b[7]))*v
        points.append(np.column_stack([x,y,z]));owners.append(np.full(len(x),i,np.int64))
    if not points:return np.empty((0,3)),np.empty(0,np.int64)
    points=np.concatenate(points);owners=np.concatenate(owners)
    key=np.column_stack([points[:,:2],np.round(points[:,2])])
    _,index=np.unique(key,axis=0,return_index=True)
    return points[index],owners[index]


def graph_arrays(nav):
    ids={a['id']:i for i,a in enumerate(nav['areas'])};n=len(ids);rows=[];cols=[];keys=[]
    for i,a in enumerate(nav['areas']):
        for d,links in enumerate(a['links']):
            for target in links:
                if target not in ids:raise ValueError(f'Dangling nav link {a["id"]}->{target}')
                j=ids[target];rows.append(i);cols.append(j);keys.append((i*4+d)*n+j)
    graph=csr_matrix((np.ones(len(rows)),(rows,cols)),shape=(n,n))
    weak,wlabels=connected_components(graph,directed=True,connection='weak')
    strong,slabels=connected_components(graph,directed=True,connection='strong')
    return np.unique(np.asarray(keys,np.int64)),{'areas':n,'directed_links':len(rows),'weak_components':int(weak),
           'strong_components':int(strong),'largest_weak_areas':int(np.bincount(wlabels).max()) if n else 0,
           'largest_strong_areas':int(np.bincount(slabels).max()) if n else 0,
           'isolated_areas':int(np.count_nonzero(np.asarray(graph.getnnz(0)+graph.getnnz(1))==0))},wlabels


@njit
def linked(keys,n,source,d,target):
    if source==target:return True
    key=(source*4+d)*n+target;i=np.searchsorted(keys,key)
    return i<len(keys) and keys[i]==key


@njit
def transition_counts(points,owners,refowners,keys,offsets,ours_links,ours_n,ref_links,ref_n):
    counts=np.zeros(6,np.int64)
    for i in range(len(points)):
        if refowners[i]<0:continue
        for d in range(4):
            key=keys[i]+offsets[d];begin=np.searchsorted(keys,key);end=np.searchsorted(keys,key,side='right')
            for j in range(begin,end):
                if refowners[j]<0:continue
                dz=points[j,2]-points[i,2]
                if dz>200 or dz<=-400:continue
                if owners[i]==owners[j] and refowners[i]==refowners[j]:continue
                a=linked(ours_links,ours_n,owners[i],d,owners[j])
                b=linked(ref_links,ref_n,refowners[i],d,refowners[j])
                counts[0]+=1
                if a and b:counts[1]+=1
                elif a:counts[2]+=1
                elif b:counts[3]+=1
                else:counts[4]+=1
    return counts


def compare(ours_path,reference_path,map_path,game=DEFAULT_GAME,missing_phy='skip',progress=print,slope_limit=.7):
    started=time.perf_counter()
    ours=read_nav(Path(ours_path).read_bytes());reference=read_nav(Path(reference_path).read_bytes());bsp=BSP(map_path)
    if not ours['areas'] or not reference['areas']:raise ValueError('Cannot compare an empty mesh')
    ours_points,ours_owners=cells(ours);ref_points,ref_owners=cells(reference)
    odist,ref_index=cKDTree(ref_points).query(ours_points,distance_upper_bound=18.00001)
    rdist,_=cKDTree(ours_points).query(ref_points,distance_upper_bound=18.00001)
    ours_match=np.isfinite(odist);ref_match=np.isfinite(rdist)
    ours_bands=np.rint(ours_points[:,2]/128).astype(int)*128;ref_bands=np.rint(ref_points[:,2]/128).astype(int)*128
    ours_links,ours_graph,_=graph_arrays(ours);ref_links,ref_graph,ref_components=graph_arrays(reference)
    xy=np.rint((ours_points[:,:2]-12.5)/25).astype(np.int64)
    keys=(xy[:,0]+65536)*2**32+xy[:,1]+65536
    order=np.argsort(keys,kind='stable');mapped=np.full(len(ours_points),-1,np.int64)
    mapped[ours_match]=ref_owners[ref_index[ours_match]]
    counts=transition_counts(ours_points[order],ours_owners[order],mapped[order],keys[order],
                             np.array([-1,2**32,1,-2**32],np.int64),ours_links,len(ours['areas']),ref_links,len(reference['areas']))
    result={'map':str(map_path),'ours':str(ours_path),'reference':str(reference_path),
            'bsp_bytes':len(bsp.data),'ours_bsp_size_matches':ours['bsp_size']==len(bsp.data),
            'reference_bsp_size_matches':reference['bsp_size']==len(bsp.data),
            'coverage':{'sampling':'25-unit XY cell centers; height match <=18; duplicate floors rounded to 1 unit',
                        'ours_cells':len(ours_points),'reference_cells':len(ref_points),'ours_matched':int(ours_match.sum()),
                        'reference_matched':int(ref_match.sum()),'precision':float(ours_match.mean()),'recall':float(ref_match.mean())},
            'ours_graph':ours_graph,'reference_graph':ref_graph,
            'directed_transitions':{'scope':'Cardinal neighbor cells on common coverage, including any boundary in either mesh; excludes shared internal subdivisions and long jump/drop gaps',
                                    'candidates':int(counts[0]),'both_connected':int(counts[1]),'ours_only':int(counts[2]),
                                    'reference_only':int(counts[3]),'both_disconnected':int(counts[4]),
                                    'precision':float(counts[1]/max(1,counts[1]+counts[2])),
                                    'recall':float(counts[1]/max(1,counts[1]+counts[3]))}}
    component_cells=np.bincount(ref_components[ref_owners],minlength=ref_graph['weak_components'])
    missed_cells=np.bincount(ref_components[ref_owners[~ref_match]],minlength=len(component_cells))
    area_counts=np.bincount(ref_components,minlength=len(component_cells))
    top=np.argsort(missed_cells)[::-1][:12]
    result['missing_coverage_by_reference_component']=[{'component':int(i),'areas':int(area_counts[i]),
        'cells':int(component_cells[i]),'missed_cells':int(missed_cells[i])} for i in top if missed_cells[i]>0]
    progress('comparison: coverage/graph complete; loading collision for wall and reference-floor checks',flush=True)
    collision=Collision.load(bsp,game,missing_phy,progress)
    our_boxes=boxes(ours);idmap={a['id']:i for i,a in enumerate(ours['areas'])};bad=[];checked=0;statuses={}
    for i,a in enumerate(ours['areas']):
        for d,links in enumerate(a['links']):
            for target in links:
                checked+=1;j=idmap[target]
                status,_=probe_portal(collision.scene,our_boxes[i],our_boxes[j],d,slope_limit,True)
                statuses[status]=statuses.get(status,0)+1
                if status in ('no_portal','unverified'):bad.append([a['id'],target,d,status])
    result['wall_check']={'checked_links':checked,'failed_links':len(bad),'examples':bad[:30],
                          'statuses':statuses,'slope_limit':slope_limit,
                          'scope':'Midpoint, drop and portal-sample probes with our collision; failed_links are unverified, not proven wall crossings; not an engine oracle'}
    rng=np.random.default_rng(20261025);selected=rng.choice(len(ref_points),min(10000,len(ref_points)),replace=False)
    start=ref_points[selected].copy();end=start.copy();start[:,2]+=18;end[:,2]-=18
    zero=np.zeros_like(start);r=batch_trace(collision.scene,start,end,zero,zero)
    supported=(r[:,0]<1)&(r[:,4]==0)
    result['reference_floor_probe']={'samples':len(start),'supported_within_18':int(supported.sum()),
                                     'unsupported_fraction':float(1-supported.mean()),
                                     'scope':'Bounded floor-ray check; unsupported samples can be hand edits, collision-model differences or BSP geometry changes'}
    missed=np.flatnonzero(~ref_match)
    selected=rng.choice(missed,min(10000,len(missed)),replace=False)
    start=ref_points[selected].copy();end=start.copy();start[:,2]+=18;end[:,2]-=18
    zero=np.zeros_like(start);r=batch_trace(collision.scene,start,end,zero,zero)
    supported=(r[:,0]<1)&(r[:,4]==0)
    result['missed_reference_floor_probe']={'samples':len(start),'supported_within_18':int(supported.sum()),
        'scope':'Only missed reference cells; supported floors can still fail hull, clearance, reachability or generator tests'}
    bands=[]
    for band in np.union1d(ours_bands,ref_bands):
        om=ours_bands==band;rm=ref_bands==band;missing=np.flatnonzero(rm&~ref_match)
        selected=rng.choice(missing,min(300,len(missing)),replace=False)
        start=ref_points[selected].copy();end=start.copy();start[:,2]+=18;end[:,2]-=18;zero=np.zeros_like(start)
        hits=batch_trace(collision.scene,start,end,zero,zero)
        bands.append({'z':int(band),'ours_cells':int(om.sum()),'reference_cells':int(rm.sum()),
                      'ours_matched':int(ours_match[om].sum()),'reference_matched':int(ref_match[rm].sum()),
                      'missed_floor_probes':len(selected),'missed_floor_supported':int(((hits[:,0]<1)&(hits[:,4]==0)).sum())})
    result['height_bands']={'width':128,'assignment':'round(Z/128)*128; matches use global XY/Z coverage', 'bands':bands}
    result['content_audit']=collision.content_audit
    result['seconds']=time.perf_counter()-started
    return result


def main():
    parser=argparse.ArgumentParser(description='Offline 25-unit coverage, graph and self-collision comparison')
    parser.add_argument('ours',type=Path);parser.add_argument('reference',type=Path);parser.add_argument('map',type=Path)
    parser.add_argument('--game',type=Path,default=DEFAULT_GAME);parser.add_argument('--missing-phy',choices=('skip','bbox'),default='skip')
    parser.add_argument('--slope-limit',type=float,default=.7)
    args=parser.parse_args()
    try:result=compare(args.ours,args.reference,args.map,args.game,args.missing_phy,slope_limit=args.slope_limit)
    except Exception as exc:print(f'comparison failed: {type(exc).__name__}: {exc}',file=sys.stderr);return 1
    print(json.dumps(result,indent=2),flush=True);return 0


if __name__=='__main__':sys.exit(main())

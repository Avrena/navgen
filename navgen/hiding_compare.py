"""Measure hiding-spot agreement in both directions, independent of spot IDs."""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .navio import read_nav


def spots(nav):return [s for a in nav['areas'] for s in a['hiding']]


def summary(nav):
    all_spots=spots(nav)
    return {'areas':len(nav['areas']),'analyzed':nav['analyzed'],'total':len(all_spots),
            'flags':dict(sorted(Counter(s[2] for s in all_spots).items())),
            'spots_per_area':dict(sorted(Counter(len(a['hiding']) for a in nav['areas']).items()))}


def match_count(source,target,tolerance=1.):
    count=0
    for flag in {s[2] for s in source}:
        p=np.asarray([s[1] for s in source if s[2]==flag]);q=np.asarray([s[1] for s in target if s[2]==flag])
        if len(q):count+=int(np.count_nonzero(cKDTree(q).query(p)[0]<=tolerance))
    return count


def extent(a):return (*a['nw'],*a['se'],a['ne_z'],a['sw_z'])


def compare_hiding(ours,reference,tolerance=1.):
    op,rp=spots(ours),spots(reference)
    om,rm=match_count(op,rp,tolerance),match_count(rp,op,tolerance)
    groups=[]
    for nav in (ours,reference):
        group=defaultdict(list)
        for a in nav['areas']:group[extent(a)].append(a)
        groups.append(group)
    pairs=[(group[0],groups[1][key][0]) for key,group in groups[0].items()
           if len(group)==1 and len(groups[1].get(key,[]))==1]
    same=nonempty=nonempty_same=0;exact_op=[];exact_rp=[];examples=[]
    for a,b in pairs:
        x,y=a['hiding'],b['hiding'];exact_op+=x;exact_rp+=y
        equal=len(x)==len(y) and match_count(x,y,tolerance)==len(x) and match_count(y,x,tolerance)==len(y)
        same+=equal;nonempty+=bool(x or y);nonempty_same+=bool((x or y) and equal)
        if not equal and len(examples)<20:examples.append({'ours_id':a['id'],'reference_id':b['id'],'ours':x,'reference':y})
    ratio=lambda n,d:n/d if d else None
    return {'tolerance_units':tolerance,'distance':'3D Euclidean, same complete flag byte, non-exclusive nearest match',
            'ours':summary(ours),'reference':summary(reference),
            'ours_matched':om,'ours_match_fraction':ratio(om,len(op)),
            'reference_matched':rm,'reference_match_fraction':ratio(rm,len(rp)),
            'exact_extents':{'definition':'All four corner coordinates exactly equal after NAV float32 serialization; unique extents only',
                             'area_pairs':len(pairs),'same_spot_sets':same,'same_spot_sets_fraction':ratio(same,len(pairs)),
                             'nonempty_pairs':nonempty,'nonempty_same_spot_sets':nonempty_same,
                             'nonempty_agreement':ratio(nonempty_same,nonempty),
                             'ours_spots':len(exact_op),'reference_spots':len(exact_rp),
                             'ours_matched':match_count(exact_op,exact_rp,tolerance),
                             'reference_matched':match_count(exact_rp,exact_op,tolerance),'examples':examples}}


def compare_files(ours,reference):
    a,b=Path(ours).read_bytes(),Path(reference).read_bytes()
    return {'ours_path':str(ours),'reference_path':str(reference),'ours_sha256':hashlib.sha256(a).hexdigest(),
            'reference_sha256':hashlib.sha256(b).hexdigest(),**compare_hiding(read_nav(a),read_nav(b))}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('ours',type=Path);parser.add_argument('reference',type=Path)
    args=parser.parse_args();print(json.dumps(compare_files(args.ours,args.reference),indent=2))


if __name__=='__main__':main()

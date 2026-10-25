"""Compare conflicting PHY convex bounds without changing content precedence."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import struct

import numpy as np
from scipy.spatial.distance import cdist

from .bsp import BSP,Content,DEFAULT_GAME,parse_phy


def compare_bounds(selected,alternate):
    a=np.array([np.r_[p.min(0),p.max(0)] for p in selected])
    b=np.array([np.r_[p.min(0),p.max(0)] for p in alternate])
    # Nearest bounds in both directions avoid depending on serialized piece order.
    distance=cdist(a,b,metric='chebyshev');ab=distance.min(1);ba=distance.min(0)
    global_a=np.r_[a[:,:3].min(0),a[:,3:].max(0)];global_b=np.r_[b[:,:3].min(0),b[:,3:].max(0)]
    delta=float(max(ab.max(),ba.max()))
    return {'selected_pieces':len(a),'alternate_pieces':len(b),'piece_count_differs':len(a)!=len(b),
            'max_nearest_piece_bound_delta':delta,'bounds_differ_over_1_unit':delta>1.,
            'global_bound_delta':float(np.abs(global_a-global_b).max()),
            'selected_global_bounds':global_a.tolist(),'alternate_global_bounds':global_b.tolist(),
            'selected_unmatched_over_1':np.flatnonzero(ab>1).tolist(),
            'alternate_unmatched_over_1':np.flatnonzero(ba>1).tolist()}


def source_bytes(content,name,label):
    if label=='BSP pakfile':return content.pak.read(content.paknames[name])
    for kind,source,index,source_label in content.sources:
        if source_label!=label:continue
        if kind=='folder':return (source/name).read_bytes()
        return source.read(name) if kind=='gma' else source.get_file(index[name]).read()
    raise ValueError(f'Content source no longer available: {label}')


def audit(generation,game=DEFAULT_GAME):
    generation=Path(generation);data=json.loads(generation.read_text(encoding='utf-8-sig'))
    bsp=BSP(data['map'])
    if hashlib.sha256(bsp.data).hexdigest()!=data['map_sha256']:raise ValueError('BSP changed since generation')
    content=Content(bsp,game);rows=[];errors=[]
    conflicts=[c for c in data['content_audit']['conflicts'] if c['path'].endswith('.phy')]
    for conflict in conflicts:
        for alternate in conflict['different_sources']:
            row={'path':conflict['path'],'selected':conflict['selected'],'alternate':alternate['source'],
                 'selected_sha256':conflict['sha256'],'alternate_sha256':alternate['sha256']}
            try:
                selected=source_bytes(content,row['path'],row['selected']);other=source_bytes(content,row['path'],row['alternate'])
                if hashlib.sha256(selected).hexdigest()!=row['selected_sha256'] or hashlib.sha256(other).hexdigest()!=row['alternate_sha256']:
                    raise ValueError('Content bytes changed since generation')
                row.update(compare_bounds(parse_phy(selected),parse_phy(other)))
            except (OSError,ValueError,KeyError,struct.error) as exc:row['error']=str(exc);errors.append(row)
            rows.append(row)
    return {'map':str(bsp.path),'generation':str(generation),'all_conflicting_paths':len(data['content_audit']['conflicts']),
            'conflicting_phy_paths':len(conflicts),'alternate_comparisons':len(rows),
            'paths_differ_over_1_unit':len({r['path'] for r in rows if r.get('bounds_differ_over_1_unit')}),
            'comparisons_differ_over_1_unit':sum(r.get('bounds_differ_over_1_unit',False) for r in rows),
            'errors':errors,'archive_errors':content.archive_errors,'comparisons':rows,
            'priority_changed':False,
            'scope':'Symmetric nearest convex-piece AABB bounds in model-space Source units; matching bounds do not prove identical hulls, and differing partitions need not change their union. Bounds alone do not establish intended runtime mount priority.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('generation',type=Path);parser.add_argument('--game',type=Path,default=DEFAULT_GAME)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    try:
        result=audit(args.generation,args.game);text=json.dumps(result,indent=2)
        if args.out:
            args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(text,encoding='utf-8')
            print(json.dumps({k:v for k,v in result.items() if k!='comparisons'},indent=2))
        else:print(text)
        return int(bool(result['errors'] or result['archive_errors']))
    except Exception as exc:
        print(f'PHY audit failed: {type(exc).__name__}: {exc}',file=sys.stderr);return 1


if __name__=='__main__':sys.exit(main())

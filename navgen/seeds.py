"""Carry a previous NAV's weak components into the next offline generation."""
import argparse
from pathlib import Path
import sys

import numpy as np

from .areas import height_at
from .bsp import BSP,DEFAULT_GAME
from .collision import Collision,THIN_MIN,THIN_MAX,trace
from .compare import boxes,graph_arrays
from .navio import read_nav
from .sampling import endpoint


def component_seeds(nav,collision=None):
    bounds=boxes(nav);_,stats,labels=graph_arrays(nav);seeds=[];details=[]
    for component in range(stats['weak_components']):
        members=np.flatnonzero(labels==component)
        i=max(members,key=lambda i:(np.prod(bounds[i,2:4]-bounds[i,:2]),-nav['areas'][i]['id']))
        box=bounds[i];point=None;reference_z=None
        # The center first, then four interior points of the same largest area.
        for u,v in ((.5,.5),(.25,.25),(.75,.25),(.75,.75),(.25,.75)):
            p=np.r_[box[:2]+np.array([u,v])*(box[2:4]-box[:2]),0.]
            if collision is not None:
                snapped=np.floor(p[:2]/25+.5)*25
                if np.any(snapped<box[:2]) or np.any(snapped>box[2:4]):continue
                p[:2]=snapped
            p[2]=height_at(box,*p[:2]);reference_z=float(p[2])
            if collision is None:point=p;break
            start=p.copy();end=p.copy();start[2]+=35.9;end[2]-=400
            hit=trace(collision.scene,start,end,THIN_MIN,THIN_MAX)
            if not hit[5] and hit[0]<1 and hit[3]>0:
                point=endpoint(start,end,hit);break
        details.append({'component':component,'areas':len(members),'largest_area_id':nav['areas'][i]['id'],
                        'seed':None if point is None else point.tolist(),'reference_z':reference_z})
        if point is not None:seeds.append(point)
    return np.asarray(seeds).reshape(-1,3),details


def main():
    parser=argparse.ArgumentParser(description='Print one seed per weak NAV component, using its largest area')
    parser.add_argument('nav',type=Path);parser.add_argument('--bsp',type=Path)
    parser.add_argument('--game',type=Path,default=DEFAULT_GAME)
    parser.add_argument('--missing-phy',choices=('skip','bbox'),default='skip')
    args=parser.parse_args()
    try:
        nav=read_nav(args.nav.read_bytes());collision=None
        if args.bsp:
            collision=Collision.load(BSP(args.bsp),args.game,missing_phy=args.missing_phy,progress=lambda *a,**k:print(*a,file=sys.stderr,**k))
        seeds,details=component_seeds(nav,collision)
        if not len(seeds):raise ValueError('No supported component seeds')
        for row in details:
            if row['seed'] is None:
                print(f'component {row["component"]}: no supported seed on largest area {row["largest_area_id"]}; skipped',file=sys.stderr)
                continue
            print(f'# component {row["component"]}, largest area {row["largest_area_id"]}')
            print(' '.join(f'{v:.6f}' for v in row['seed']))
        print(f'{len(seeds)} seeds from {len(details)} weak components',file=sys.stderr)
    except Exception as exc:
        print(f'seed extraction failed: {type(exc).__name__}: {exc}',file=sys.stderr);return 1
    return 0


if __name__=='__main__':sys.exit(main())

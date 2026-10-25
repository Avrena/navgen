"""Offline endpoint classification for retained, unverified NAV connections."""
from collections import Counter

import numpy as np

from navgen.areas import OBSTACLE_TOP,height_at,probe_portal
from navgen.compare import boxes
from navgen.sampling import DIRECTIONS


class Reference:
    def __init__(self,nav):
        self.nav=nav;self.boxes=boxes(nav)
        self.ids={a['id']:i for i,a in enumerate(nav['areas'])}
        self.adj=[{self.ids[j] for links in a['links'] for j in links} for a in nav['areas']]

    def column(self,p):
        b=self.boxes
        return np.flatnonzero((b[:,0]-1<=p[0])&(p[0]<=b[:,2]+1)&(b[:,1]-1<=p[1])&(p[1]<=b[:,3]+1))

    def at(self,p):
        return [int(i) for i in self.column(p) if abs(height_at(self.boxes[i],p[0],p[1])-p[2])<=20]

    def status(self,start,goal):
        a,b=self.at(start),self.at(goal)
        if not a or not b:return 'off_engine_mesh'
        if set(a)&set(b):return 'same_engine_area'
        if any(j in self.adj[i] for i in a for j in b):return 'engine_direct_link'
        mids=set().union(*(self.adj[i] for i in a))
        if any(j in self.adj[k] for k in mids for j in b):return 'engine_two_hops'
        return 'engine_not_linked'


def endpoints(source,target,d,pair):
    p,q=pair;step=DIRECTIONS[d];axis=0 if d in (1,3) else 1
    start=p-step*min(20.,source[axis+2]-source[axis])
    goal=q+step*min(20.,(target[axis+2]-target[axis])/2)
    start[2]=height_at(source,*start[:2]);goal[2]=height_at(target,*goal[:2])
    return start,goal


def classify(nav,reference,scene,slope_limit=.55):
    """Same probes/endpoints as the run4 route script, with the correct SDK flag."""
    ref=Reference(reference);bounds=boxes(nav);ids={a['id']:i for i,a in enumerate(nav['areas'])}
    records=[];counts=Counter()
    for i,a in enumerate(nav['areas']):
        for d,links in enumerate(a['links']):
            for dest in links:
                j=ids[dest];status,pair=probe_portal(scene,bounds[i],bounds[j],d,slope_limit,True)
                if status!='unverified':continue
                start,goal=endpoints(bounds[i],bounds[j],d,pair);dz=float(goal[2]-start[2])
                kind=('fence' if (a['attr']|nav['areas'][j]['attr'])&OBSTACLE_TOP else
                      'drop' if dz < -18 else 'climb' if dz > 18 else 'ordinary')
                status=ref.status(start,goal);counts[kind,status]+=1
                records.append({'from':a['id'],'to':dest,'dir':d,'kind':kind,'dz':round(dz,1),'engine':status,
                                'start':start.tolist(),'goal':goal.tolist()})
    return {'total':len(records),'obstacle_top_areas':sum(bool(a['attr']&OBSTACLE_TOP) for a in nav['areas']),
            'groups':[{'kind':k,'engine':e,'count':v} for (k,e),v in sorted(counts.items())],'records':records}

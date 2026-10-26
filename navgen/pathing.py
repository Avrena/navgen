"""Source CNavArea A* and NextBot portal placement, with explicit limitations.

Directed connections are authoritative, including zero-width/disjoint portals.
This reproduces static area costs and path details, not bot motion or Lua costs.
"""
import heapq
import time
import numpy as np
from .areas import height_at
from .compare import boxes
from .runtime import njit


@njit
def native_portal(a,b,d):
    axis=1 if d in (0,2) else 0;tangent=1-axis
    lo=min(a[tangent+2],max(a[tangent],max(a[tangent],b[tangent])))
    hi=min(a[tangent+2],max(a[tangent],min(a[tangent+2],b[tangent+2])))
    p=np.zeros(3);p[axis]=a[axis] if d in (0,3) else a[axis+2];p[tangent]=(lo+hi)*.5
    p[2]=height_at(a,p[0],p[1])
    return p,(hi-lo)*.5


@njit
def closest_on_area(b,p):
    q=p.copy();q[0]=min(b[2],max(b[0],p[0]));q[1]=min(b[3],max(b[1],p[1]));q[2]=height_at(b,q[0],q[1])
    return q


@njit
def crossing(a,b,d,p,edges):
    axis=1 if d in (0,2) else 0;tangent=1-axis
    lo=max(a[tangent],b[tangent]);hi=min(a[tangent+2],b[tangent+2])
    low_dir=3 if tangent==0 else 0;high_dir=1 if tangent==0 else 2
    low=lo+(25. if edges[low_dir] else 0.);high=hi-(25. if edges[high_dir] else 0.)
    if low>high:low=high=(lo+hi)*.5
    q=p.copy();q[axis]=a[axis] if d in (0,3) else a[axis+2];q[tangent]=min(high,max(low,p[tangent]))
    q[2]=height_at(a,q[0],q[1]);return q


@njit
def area_astar(centers,targets,directions,offsets,lengths,height_changes,source,target,goal,step=18.,jump=58.,drop=200.):
    n=len(centers);cost=np.full(n,np.inf,np.float32);parent=np.full(n,-1,np.int32);how=parent.copy()
    cost[source]=0.;serial=0;expanded=0;closest=source;closest_distance=np.linalg.norm(centers[source]-goal)
    heap=[(np.float32(closest_distance),np.int64(0),np.int64(source),np.float32(0.))]
    reached=False
    while heap:
        score,_,i,known=heapq.heappop(heap)
        if known!=cost[i]:continue
        expanded+=1
        if i==target:closest=i;reached=True;break
        for e in range(offsets[i],offsets[i+1]):
            j=targets[e];dz=height_changes[e]
            if dz>=jump or dz < -drop:continue
            value=np.float32(cost[i]+lengths[e]*(6. if dz>=step else 1.))
            value=max(value,np.float32(cost[i]*np.float32(1.00001)+np.float32(.00001)))
            if value>=cost[j]:continue
            distance=np.float32(np.linalg.norm(centers[j]-goal))
            if distance<closest_distance:closest=j;closest_distance=distance
            cost[j]=value;parent[j]=i;how[j]=directions[e];serial+=1
            heapq.heappush(heap,(np.float32(value+distance),np.int64(-serial),np.int64(j),np.float32(value)))
    return parent,how,closest,reached,expanded,float(cost[closest])


class NativeGraph:
    def __init__(self,nav):
        self.nav=nav;self.bounds=boxes(nav);self.ids={a['id']:i for i,a in enumerate(nav['areas'])}
        b=self.bounds;self.centers=np.column_stack([(b[:,0]+b[:,2])*.5,(b[:,1]+b[:,3])*.5,(b[:,4]+b[:,6])*.5]).astype(np.float32)
        targets=[];directions=[];lengths=[];heights=[];offsets=[0];self.edges=np.ones((len(b),4),np.bool_)
        for i,a in enumerate(nav['areas']):
            for d,links in enumerate(a['links']):
                for ident in links:
                    j=self.ids[ident];p,_=native_portal(b[i],b[j],d);q,_=native_portal(b[j],b[i],(d+2)%4)
                    targets.append(j);directions.append(d);lengths.append(np.linalg.norm(self.centers[j]-self.centers[i]));heights.append(q[2]-p[2])
                    if a['id'] in nav['areas'][j]['links'][(d+2)%4]:self.edges[i,d]=False
            offsets.append(len(targets))
        self.targets=np.asarray(targets,np.int32);self.directions=np.asarray(directions,np.int32);self.offsets=np.asarray(offsets,np.int32)
        self.lengths=np.asarray(lengths,np.float32);self.height_changes=np.asarray(heights,np.float32)

    def nearest(self,p):
        b=self.bounds;x=np.clip(p[0],b[:,0],b[:,2]);y=np.clip(p[1],b[:,1],b[:,3])
        u=(x-b[:,0])/(b[:,2]-b[:,0]);v=(y-b[:,1])/(b[:,3]-b[:,1])
        z=(b[:,4]+u*(b[:,5]-b[:,4]))*(1-v)+(b[:,7]+u*(b[:,6]-b[:,7]))*v
        distance=(x-p[0])**2+(y-p[1])**2+(z-p[2])**2
        return int(np.argmin(distance))

    def path(self,start,goal,step=18.,jump=58.,drop=200.,scene=None,hull_width=26.,settle_start=True):
        start=np.asarray(start,float).copy();goal=np.asarray(goal,float).copy();source=self.nearest(start);target=self.nearest(goal)
        if settle_start:start[2]=height_at(self.bounds[source],start[0],start[1])
        goal[2]=height_at(self.bounds[target],goal[0],goal[1])
        begin=time.perf_counter()
        parent,how,last,complete,expanded,cost=area_astar(self.centers,self.targets,self.directions,self.offsets,self.lengths,self.height_changes,source,target,goal.astype(np.float32),step,jump,drop)
        astar_ms=(time.perf_counter()-begin)*1000
        route=[];i=last
        while i>=0 and len(route)<100000:
            route.append(i)
            if i==source:break
            i=int(parent[i])
        route=route[::-1]
        segments=[{'area':route[0],'pos':start,'how':9,'type':0}]
        for a,b in zip(route,route[1:]):
            d=int(how[b]);p=crossing(self.bounds[a],self.bounds[b],d,segments[-1]['pos'],self.edges[b])
            segments.append({'area':b,'pos':p,'how':d,'type':0})
        segments.append({'area':last,'pos':goal,'how':9,'type':0})
        # Climb/gap details follow NextBotPath::ComputePathDetails. Drop-down
        # collision pushes are applied only when a collision scene is supplied.
        if scene is not None:
            from .collision import trace
            from .bsp import BRUSH_MASK
            i=1
            while i<len(segments):
                prev=segments[i-1];cur=segments[i]
                if cur['how']>3:i+=1;continue
                a=self.bounds[prev['area']];b=self.bounds[cur['area']];p=cur['pos'];floor=height_at(b,p[0],p[1])
                normal=np.cross(np.array([a[2]-a[0],0,a[5]-a[4]]),np.array([0,a[3]-a[1],a[7]-a[4]]));normal/=np.linalg.norm(normal)
                q=p.copy();q[2]=floor;prior=prev['pos'].copy();prior[2]=height_at(a,prior[0],prior[1])
                if -np.dot(q-prior,normal)>step:
                    d=cur['how'];direction=np.array([(0,-1,0),(1,0,0),(0,1,0),(-1,0,0)][d],float);width=hull_width+5.;push=0.
                    for push in np.arange(0,2*width+10.,10.):
                        top=p+direction*push;bottom=top.copy();bottom[2]=floor
                        r=trace(scene,top,bottom,np.array([-width/2,-width/2,step]),np.array([width/2,width/2,71.]))
                        if r[0]>=1:break
                    top=p+direction*push;bottom=top.copy();bottom[2]=floor
                    ray_start=bottom.copy();ray_start[2]+=18.;ray_end=bottom.copy();ray_end[2]-=300.
                    r=trace(scene,ray_start,ray_end,np.zeros(3),np.zeros(3),BRUSH_MASK)
                    if r[0]<1 and not r[4]:
                        ground=ray_start[2]+r[0]*(ray_end[2]-ray_start[2])
                        if top[2]>ground+step:
                            cur['pos']=top;cur['type']=1;bottom[2]=ground
                            segments.insert(i+1,{'area':cur['area'],'pos':bottom,'how':cur['how'],'type':0});i+=1
                i+=1
        i=0
        while i<len(segments)-1:
            a,b=segments[i:i+2]
            if (a['how']!=9 and a['how']>3) or b['how']>3 or b['type']!=0:i+=1;continue
            close_to=closest_on_area(self.bounds[b['area']],a['pos']);close_from=closest_on_area(self.bounds[a['area']],close_to)
            separation=np.linalg.norm((close_to-close_from)[:2])
            if separation>47.5 and separation>.5*abs(close_to[2]-close_from[2]):
                land=closest_on_area(self.bounds[b['area']],b['pos']);launch=closest_on_area(self.bounds[a['area']],land)
                forward=land-launch;forward/=max(np.linalg.norm(forward),1e-9);half=(hull_width+5.)*.5
                b['pos']=land+forward*half;segments.insert(i+1,{'area':a['area'],'pos':launch-forward*half,'how':a['how'],'type':3});i+=1
            elif close_to[2]-close_from[2]>step:
                b['pos']=self.centers[b['area']].astype(float);launch=closest_on_area(self.bounds[a['area']],b['pos'])
                segments.insert(i+1,{'area':a['area'],'pos':launch,'how':a['how'],'type':2});i+=1
            i+=1
        points=np.array([s['pos'] for s in segments]);length=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
        return {'length':length,'complete':bool(complete),'expansions':int(expanded),'astar_ms':astar_ms,'cost':cost,
                'area_ids':[self.nav['areas'][i]['id'] for i in route],'points':points.tolist(),'types':[s['type'] for s in segments],
                'source':self.nav['areas'][source]['id'],'target':self.nav['areas'][target]['id'],
                'limitations':['nearest-area LOS and dynamic blockers are not reconstructed','ladder/elevator edges are omitted','native float/tie behavior may differ']}

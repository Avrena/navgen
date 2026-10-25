"""Directed area graph, Source-style generation cleanup, and v16 records."""
from collections import defaultdict, deque
from dataclasses import dataclass, field
import math
import time

import numpy as np
from numba import njit

from .bsp import BRUSH_MASK, NPC_MASK
from .collision import THIN_MIN, THIN_MAX, trace
from .sampling import DIRECTIONS, adjacent, can_traverse, endpoint, find_node, rectangles

CROUCH,JUMP,STAIRS,NO_MERGE,OBSTACLE_TOP=1,2,4096,8192,16384


@njit
def height_at(box,x,y):
    u=min(1.,max(0.,(x-box[0])/(box[2]-box[0])))
    v=min(1.,max(0.,(y-box[1])/(box[3]-box[1])))
    north=box[4]+u*(box[5]-box[4]);south=box[7]+u*(box[6]-box[7])
    return north+v*(south-north)


def corner_points(box):
    return np.array([[box[0],box[1],box[4]],[box[2],box[1],box[5]],
                     [box[2],box[3],box[6]],[box[0],box[3],box[7]]])


def surface_normals(box):
    c=corner_points(box)
    a=np.cross(c[1]-c[0],c[3]-c[0]);b=np.cross(c[3]-c[2],c[1]-c[2])
    return a/max(np.linalg.norm(a),1e-9),b/max(np.linalg.norm(b),1e-9)


def portal(box,other,d):
    axis=1 if d in (0,2) else 0;tangent=1-axis
    lo=max(box[tangent],other[tangent]);hi=min(box[tangent+2],other[tangent+2])
    if hi-lo<=1e-5:return None
    p=np.zeros(3);q=np.zeros(3);p[tangent]=q[tangent]=(lo+hi)*.5
    p[axis]=box[axis] if d in (0,3) else box[axis+2]
    q[axis]=other[axis+2] if d in (0,3) else other[axis]
    p[2]=height_at(box,p[0],p[1]);q[2]=height_at(other,q[0],q[1])
    return p,q


def probe_portal(scene,box,other,d,slope_limit=.7,alternatives=False):
    pair=portal(box,other,d)
    if pair is None:return 'no_portal',None
    p,q=pair;p-=DIRECTIONS[d]*.05;q+=DIRECTIONS[d]*.05;p[2]+=.05;q[2]+=.05
    if can_traverse(scene,p,q,slope_limit):return 'midpoint',pair
    if not alternatives:return 'unverified',pair
    if jump_down(scene,p,q):return 'drop_probe',pair
    tangent=0 if d in (0,2) else 1
    lo=max(box[tangent],other[tangent]);hi=min(box[tangent+2],other[tangent+2]);inset=min(.5,(hi-lo)/4)
    coords=[lo+inset,hi-inset,*np.arange(math.ceil(lo/25)*25,hi,25)]
    for coordinate in coords:
        if not lo+1e-5<coordinate<hi-1e-5:continue
        a=p.copy();b=q.copy();a[tangent]=b[tangent]=coordinate
        a[2]=height_at(box,*a[:2])+.05;b[2]=height_at(other,*b[:2])+.05
        if can_traverse(scene,a,b,slope_limit) or jump_down(scene,a,b):return 'portal_sample',(a,b)
    return 'unverified',pair


@njit
def jump_down(scene,start,end):
    dz=start[2]-end[2]
    if dz<=64 or dz>=400:return False
    for height in range(1,201):
        up=start.copy();up[2]+=height
        r=trace(scene,start,up,THIN_MIN,THIN_MAX,BRUSH_MASK,False)
        if r[0]<=0 or r[4]:continue
        a=endpoint(start,up,r);a[2]-=.5
        b=end.copy();b[2]=a[2]
        r=trace(scene,a,b,THIN_MIN,THIN_MAX,BRUSH_MASK,False)
        if r[0]!=1 or r[4]:continue
        finish=end.copy();finish[2]+=2
        r=trace(scene,b,finish,THIN_MIN,THIN_MAX,BRUSH_MASK,False)
        return r[0]>0 and not r[4] and endpoint(b,finish,r)[2]<=finish[2]+18
    return False


@njit
def stair_line(scene,start,end,slope_limit=.7):
    if abs(start[2]-end[2])<=18:return 0
    length=np.sqrt(np.sum((start[:2]-end[:2])**2))
    if length<1:return 0
    mins=np.array([-2.5,-2.5,0.]);maxs=np.array([2.5,2.5,1.])
    axis=0 if abs(start[0]-end[0])>abs(start[1]-end[1]) else 1
    mins[axis]=-8;maxs[axis]=8
    previous=0.;seen=False
    for i in range(int(math.ceil(length/5))+1):
        point=start+(end-start)*min(1.,i*5/length)
        a=point.copy();a[2]+=36;b=point.copy();b[2]-=36
        r=trace(scene,a,b,mins,maxs,NPC_MASK,False)
        if r[4] or r[7] or r[0]>=1 or r[3]<.97:return -1
        z=endpoint(a,b,r)[2]
        if i==0:
            if abs(z-start[2])>18:return -1
        else:
            delta=abs(z-previous)
            if delta>18.01:return -1
            if delta>5*math.tan(math.acos(slope_limit)):seen=True
        previous=z
    if abs(previous-end[2])>18:return -1
    return 1 if seen else 0


@dataclass
class Area:
    id: int
    box: np.ndarray
    attr: int
    nodes: np.ndarray
    cells: np.ndarray
    disp: bool=False
    links: list=field(default_factory=lambda:[set() for _ in range(4)])
    incoming: set=field(default_factory=set)

    def z(self,x,y):return height_at(self.box,x,y)


class Spatial:
    def __init__(self,areas,size=128):
        self.bins=defaultdict(list);self.areas=areas;self.size=size
        for a in areas.values():
            b=a.box
            for x in range(math.floor(b[0]/size),math.floor(b[2]/size)+1):
                for y in range(math.floor(b[1]/size),math.floor(b[3]/size)+1):self.bins[x,y].append(a.id)

    def below(self,point,limit=400):
        x,y,z=point;best=None;bestz=-math.inf
        for i in self.bins.get((math.floor(x/self.size),math.floor(y/self.size)),()):
            a=self.areas[i];b=a.box
            if b[0]-.001<=x<=b[2]+.001 and b[1]-.001<=y<=b[3]+.001:
                az=a.z(x,y)
                if z-limit<=az<=z+.001 and az>bestz:best=a;bestz=az
        return best


class AreaGraph:
    def __init__(self,collision,arrays,index,boxes,corners,attrs,slope_limit=.7):
        self.collision=collision;self.scene=collision.scene;self.arrays=arrays;self.index=index
        self.areas={};self.next_id=len(boxes);self.counts={}
        self.slope_limit=slope_limit
        self.link_kinds={};self.link_audit={}
        owner=arrays[-1];ids=np.flatnonzero(owner>=0);ids=ids[np.argsort(owner[ids],kind='stable')]
        groups=np.split(ids,np.flatnonzero(np.diff(owner[ids]))+1) if len(ids) else []
        for group in groups:
            i=int(owner[group[0]])
            self.areas[i]=Area(i,boxes[i],int(attrs[i]),corners[i],group,bool(arrays[12][group].any()))

    def connect(self,a,b,d,kind=None):
        if a==b or a not in self.areas or b not in self.areas:return
        self.areas[a].links[d].add(b);self.areas[b].incoming.add((a,d))
        key=(a,b,d)
        self.link_kinds[key]=kind or self.link_kinds.get(key,'ordinary_step')

    def disconnect(self,a,b,d):
        if a in self.areas:self.areas[a].links[d].discard(b)
        if b in self.areas:self.areas[b].incoming.discard((a,d))
        self.link_kinds.pop((a,b,d),None)

    def remove(self,i):
        a=self.areas[i]
        for d,links in enumerate(a.links):
            for b in list(links):self.disconnect(i,b,d)
        for b,d in list(a.incoming):self.disconnect(b,i,d)
        self.arrays[-1][a.cells]=-1
        del self.areas[i]

    def insert(self,box,attr,cells=None,nodes=None,disp=False):
        i=self.next_id;self.next_id+=1
        if cells is None:cells=np.empty(0,np.int64)
        if nodes is None:nodes=np.array([find_node(self.index,self.arrays,p) for p in corner_points(box)])
        a=Area(i,box.copy(),attr,nodes,cells,disp);self.areas[i]=a
        self.arrays[-1][cells]=i
        return a

    def initial_links(self):
        links=self.arrays[2];owner=self.arrays[-1]
        for d in range(4):
            valid=np.flatnonzero((owner>=0)&(links[:,d]>=0))
            other=links[valid,d]
            keep=(owner[other]>=0)&(owner[other]!=owner[valid])&(links[other,(d+2)%4]==valid)
            for a,b in np.unique(np.column_stack([owner[valid[keep]],owner[other[keep]]]),axis=0):self.connect(int(a),int(b),d)
        spatial=Spatial(self.areas);drops=0;parts=[0,0];pos=self.arrays[0];blocked=self.arrays[7]
        def probe(a,node,d):
            p=pos[node];start=p+DIRECTIONS[d]*12.5;start[2]+=35.5
            for distance in (25.,50.,75.,100.):
                q=start+DIRECTIONS[d]*distance
                tr=trace(self.scene,start,q,THIN_MIN,THIN_MAX,BRUSH_MASK,False)
                if tr[0]<1:break
                b=spatial.below(q)
                if b is None:continue
                if b.id!=a.id:
                    dest=q.copy();dest[2]=b.z(q[0],q[1])
                    if jump_down(self.scene,p,dest):
                        existed=b.id in a.links[d]
                        self.connect(a.id,b.id,d,'drop')
                        return int(not existed)
                break
            return 0
        for a in list(self.areas.values()):
            for d in range(4):
                # SDK ConnectGeneratedAreas: south/east first use the last owned row/column.
                c0,c1,along=((0,1,1),(1,2,2),(3,2,1),(0,3,2))[d]
                for part in range(2 if d in (1,2) else 1):
                    node,end=int(a.nodes[c0]),int(a.nodes[c1])
                    if part==0 and d in (1,2):
                        node=int(links[node,(d+2)%4]) if node>=0 else -1
                        end=int(links[end,(d+2)%4]) if end>=0 else -1
                    seen=set()
                    while node>=0 and node!=end and node not in seen:
                        seen.add(node);adj=int(links[node,d])
                        direct=adj>=0 and owner[adj]>=0 and links[adj,(d+2)%4]==node
                        if part==0:
                            if not direct:parts[0]+=probe(a,node,d)
                        elif owner[node]<0 and not blocked[node].any() and not(adj>=0 and blocked[adj].any()):
                            if adj<0 or owner[adj]<0:parts[1]+=probe(a,node,d)
                        node=int(links[node,along])
        drops=sum(parts)
        self.counts.update(drop_connections_found=drops,drop_part1=parts[0],drop_part2=parts[1])

    def mark_jumps(self):
        count=0
        for a in self.areas.values():
            n1,n2=surface_normals(a.box);nz=min(n1[2],n2[2]);jump=nz<self.slope_limit
            if not jump and nz<self.slope_limit+.1:
                p=np.array([(a.box[0]+a.box[2])*.5,(a.box[1]+a.box[3])*.5,0.]);p[2]=a.z(p[0],p[1])+35.5
                end=p.copy();end[2]-=9999.9
                r=trace(self.scene,p,end,np.zeros(3),np.zeros(3),BRUSH_MASK,False)
                jump=r[0]<1 and abs(r[3]-nz)>.1
            if jump:a.attr|=JUMP|NO_MERGE;count+=1
        self.counts['jump_areas_marked']=count

    def merge(self):
        queue=deque(sorted(self.areas));merged=0
        while queue:
            i=queue.popleft()
            if i not in self.areas:continue
            a=self.areas[i]
            if a.attr&NO_MERGE:continue
            joined=False
            for d in (0,2,3,1):
                for j in sorted(a.links[d]):
                    if j not in self.areas:continue
                    b=self.areas[j]
                    if a.attr!=b.attr or b.attr&NO_MERGE or i not in b.links[(d+2)%4]:continue
                    axis=1 if d in (0,2) else 0;tangent=1-axis
                    if abs(a.box[tangent]-b.box[tangent])>.01 or abs(a.box[tangent+2]-b.box[tangent+2])>.01:continue
                    pair=portal(a.box,b.box,d)
                    if pair is None or np.linalg.norm(pair[0][:2]-pair[1][:2])>.01:continue
                    low=min(a.box[axis],b.box[axis]);high=max(a.box[axis+2],b.box[axis+2])
                    if high-low>1250.01:continue
                    n1,n2=surface_normals(a.box);m1,m2=surface_normals(b.box)
                    tolerance=.7 if a.disp or b.disp else .99
                    if min(n1@m1,n1@m2,n2@m1,n2@m2)<tolerance:continue
                    newbox=a.box.copy();newbox[axis]=low;newbox[axis+2]=high
                    points=corner_points(newbox)
                    for k,p in enumerate(points):
                        source=a if a.box[0]-.01<=p[0]<=a.box[2]+.01 and a.box[1]-.01<=p[1]<=a.box[3]+.01 else b
                        newbox[4+k]=source.z(p[0],p[1])
                    if any(abs(height_at(newbox,*p[:2])-p[2])>5 for old in (a,b) for p in corner_points(old.box)):continue
                    outgoing=[(k,d2,self.link_kinds[(j,k,d2)]) for d2,ls in enumerate(b.links) for k in ls if k!=i]
                    incoming=[(k,d2,self.link_kinds[(k,j,d2)]) for k,d2 in b.incoming if k!=i]
                    cells=np.concatenate([a.cells,b.cells]);self.remove(j)
                    a.box=newbox;a.cells=cells;a.disp|=b.disp
                    a.nodes=np.array([find_node(self.index,self.arrays,p) for p in corner_points(newbox)])
                    self.arrays[-1][cells]=i
                    for k,d2,kind in outgoing:self.connect(i,k,d2,kind)
                    for k,d2,kind in incoming:self.connect(k,i,d2,kind)
                    queue.append(i);queue.extend(k for k,_ in a.incoming);merged+=1;joined=True;break
                if joined:break
        self.counts['merges']=merged

    def split(self,i,axis,coordinate):
        a=self.areas[i];box=a.box
        if not box[axis]+.1<coordinate<box[axis+2]-.1:return None
        out=[(b,d,self.link_kinds[(i,b,d)]) for d,ls in enumerate(a.links) for b in ls]
        inc=[(b,d,self.link_kinds[(b,i,d)]) for b,d in a.incoming]
        cells=a.cells;positions=self.arrays[0][cells,axis]
        boxes=[box.copy(),box.copy()];boxes[0][axis+2]=coordinate;boxes[1][axis]=coordinate
        for b in boxes:
            for k,p in enumerate(corner_points(b)):b[4+k]=a.z(p[0],p[1])
        self.remove(i)
        parts=[self.insert(b,a.attr,cells[positions<coordinate] if k==0 else cells[positions>=coordinate],disp=a.disp) for k,b in enumerate(boxes)]
        for new in parts:
            for j,d,kind in out:
                if j not in self.areas:continue
                dim=1 if d in (0,2) else 0;edge=dim if d in (0,3) else dim+2
                if abs(new.box[edge]-box[edge])<.01 and portal(new.box,self.areas[j].box,d) is not None:self.connect(new.id,j,d,kind)
            for j,d,kind in inc:
                if j not in self.areas:continue
                reverse=(d+2)%4;dim=1 if d in (0,2) else 0;edge=dim if reverse in (0,3) else dim+2
                if abs(new.box[edge]-box[edge])<.01 and portal(self.areas[j].box,new.box,d) is not None:self.connect(j,new.id,d,kind)
        d=1 if axis==0 else 2
        self.connect(parts[0].id,parts[1].id,d);self.connect(parts[1].id,parts[0].id,(d+2)%4)
        return parts

    def split_overhangs(self):
        count=0;queue=deque(sorted(self.areas))
        while queue:
            i=queue.popleft()
            if i not in self.areas:continue
            a=self.areas[i];done=False
            for d in range(4):
                for j in list(a.links[d]):
                    if j not in self.areas:continue
                    b=self.areas[j]
                    if min(a.box[2],b.box[2])-max(a.box[0],b.box[0])<=.1 or min(a.box[3],b.box[3])-max(a.box[1],b.box[1])<=.1:continue
                    lower,upper=(a,b) if np.mean(a.box[4:])<np.mean(b.box[4:]) else (b,a)
                    if min(upper.box[4:])<=max(lower.box[4:])+55:continue
                    axis=1 if d in (0,2) else 0
                    down=d if upper is a else (d+2)%4;up=(down+2)%4
                    if upper.box[axis+2]<lower.box[axis+2]:
                        edge=upper.box[axis+2];length=edge-lower.box[axis]
                    else:
                        edge=upper.box[axis];length=lower.box[axis+2]-edge
                    if length<25:
                        if length<7.5 or lower.box[axis+2]-lower.box[axis]<=50:continue
                        edge+=(25-length)*(-1 if down in (0,3) else 1)
                    if not lower.box[axis]+.1<edge<lower.box[axis+2]-.1:continue
                    from_above=lower.id in upper.links[down];from_below=upper.id in lower.links[up]
                    above_kind=self.link_kinds.get((upper.id,lower.id,down));below_kind=self.link_kinds.get((lower.id,upper.id,up))
                    self.disconnect(upper.id,lower.id,down);self.disconnect(lower.id,upper.id,up)
                    parts=self.split(lower.id,axis,edge)
                    if parts:
                        exposed=parts[0 if down in (0,3) else 1]
                        if from_above:self.connect(upper.id,exposed.id,down,above_kind)
                        if from_below:self.connect(exposed.id,upper.id,up,below_kind)
                        count+=1;queue.extend(p.id for p in parts);queue.append(upper.id);done=True
                    if done:break
                if done:break
        self.counts['overhang_splits']=count

    def square(self):
        queue=deque(sorted(self.areas));count=0
        while queue:
            i=queue.popleft()
            if i not in self.areas:continue
            a=self.areas[i];sizes=a.box[2:4]-a.box[:2]
            if max(sizes)<=3.01*min(sizes):continue
            axis=int(np.argmax(sizes));cut=math.floor((a.box[axis]+a.box[axis+2])/50+.5)*25
            parts=self.split(i,axis,cut)
            if parts:count+=1;queue.extend(p.id for p in parts)
        self.counts['square_splits']=count

    def mark_stairs(self):
        count=0
        for a in self.areas.values():
            b=a.box
            if max(b[2]-b[0],b[3]-b[1])<=25 or np.ptp(b[4:])<=18:continue
            n1,n2=surface_normals(b)
            if n1@n2<.95:continue
            x0,y0,x1,y1=b[:4]+np.array([5,5,-5,-5]);xm=(x0+x1)*.5;ym=(y0+y1)*.5
            pairs=[((x0,y0),(x1,y0)),((x0,y1),(x1,y1)),((x0,y0),(x0,y1)),((x1,y0),(x1,y1)),((xm,y0),(xm,y1)),((x1,ym),(x0,ym))]
            results=[]
            for p,q in pairs:
                start=np.array([*p,a.z(*p)]);end=np.array([*q,a.z(*q)])
                results.append(stair_line(self.scene,start,end,self.slope_limit))
            if min(results)>=0 and max(results)>0:a.attr=STAIRS;count+=1
        self.counts['stair_areas']=count

    def stitch_jumps(self):
        removed=0;added=0
        jumps=[i for i,a in self.areas.items() if a.attr&JUMP]
        for i in jumps:
            a=self.areas[i]
            for source,d in list(a.incoming):
                if source not in self.areas or self.areas[source].attr&JUMP:continue
                for dest in list(a.links[d]):
                    if dest not in self.areas or self.areas[dest].attr&JUMP:continue
                    pair=portal(self.areas[source].box,self.areas[dest].box,d)
                    if pair is None or np.linalg.norm(pair[0][:2]-pair[1][:2])>=75:continue
                    if self.areas[source].attr&STAIRS and pair[1][2]-pair[0][2]>18:continue
                    self.connect(source,dest,d,'stitched_jump');added+=1
        # The SDK stitches all jump areas before deleting any of them.
        for i in jumps:self.remove(i);removed+=1
        self.counts.update(jump_areas_removed=removed,jump_links_added=added)

    def fence_tops(self):
        raised=created=removed=0;pos=self.arrays[0];obsh,obsa,obsb=self.arrays[9:12]
        for a in list(self.areas.values()):
            if np.max(np.abs(a.box[2:4]-a.box[:2]-25))>.01 or min(a.nodes)<0:continue
            best=None;stair_neighbor=False
            for d in range(4):
                ids=[a.nodes[(d+3)%4],a.nodes[(d+2)%4]]
                stair_here=any(self.areas[j].attr&STAIRS for j in a.links[d]);stair_neighbor|=stair_here
                if np.count_nonzero(obsh[ids,d]>18)<(1 if stair_here else 2):continue
                top=pos[ids,2]+obsh[ids,d]
                if best is None or max(top)>max(best[1]):best=(d,top,float(min(obsa[ids,d])),float(max(obsb[ids,d])))
            if best is None:continue
            if stair_neighbor:self.remove(a.id);removed+=1;continue
            d,top,start,end=best
            if end-start<10:start=max(0.,(start+end)*.5-5);end=min(25.,start+10);start=end-10
            axis=1 if d in (0,2) else 0;box=a.box.copy()
            if d in (1,2):box[axis+2]=box[axis]+end;box[axis]+=start
            else:box[axis]=box[axis+2]-end;box[axis+2]-=start
            if d in (2,3):top=top[::-1]
            box[4:]=top[[0,1,1,0]] if axis==1 else top[[0,0,1,1]]
            a.box=box;a.attr=(a.attr|NO_MERGE|OBSTACLE_TOP)&~JUMP;a.nodes=np.full(4,-1)
            for orth in ((d+1)%4,(d+3)%4):
                for j in list(a.links[orth]):self.disconnect(a.id,j,orth)
                for j,di in list(a.incoming):
                    if (di+2)%4==orth:self.disconnect(j,a.id,di)
            raised+=1
        # Require an obstacle across every sampled node of the shared portal.
        for a in list(self.areas.values()):
            if a.attr&(JUMP|OBSTACLE_TOP) or min(a.nodes)<0:continue
            for d in range(4):
                for j in list(a.links[d]):
                    b=self.areas[j]
                    if b.attr&(JUMP|OBSTACLE_TOP):continue
                    if self.create_fence(a,b,d,False) or self.create_fence(a,b,d,True):created+=1
        self.counts.update(obstacle_areas_raised=raised,obstacle_areas_created=created,obstacle_areas_removed=removed)

    def create_fence(self,a,b,d,multi):
        pos=self.arrays[0];links=self.arrays[2];obsh,obsa,obsb=self.arrays[9:12]
        axis=1 if d in (0,2) else 0;tangent=1-axis
        lo=max(a.box[tangent],b.box[tangent]);hi=min(a.box[tangent+2],b.box[tangent+2])
        if hi<=lo:return False
        node=int(a.nodes[d]);last=int(a.nodes[(d+1)%4]);edge=(d+1)%4
        heights=[];levels=[];starts=[];ends=[];seen=set()
        while node>=0 and node not in seen:
            seen.add(node);p=pos[node]
            if lo-1<=p[tangent]<=hi+1:
                height=float(obsh[node,d]);start=float(obsa[node,d]);end=float(obsb[node,d])
                if multi:
                    height=0.;other=int(links[node,d])
                    if other>=0 and pos[other,2]-p[2]>18:
                        q=pos[other];inside=b.box[0]<=q[0]<=b.box[2] and b.box[1]<=q[1]<=b.box[3] and abs(q[2]-b.z(q[0],q[1]))<2
                        further=int(links[other,d])
                        if not inside and further>=0:
                            r=pos[further]
                            if b.box[0]<=r[0]<=b.box[2] and b.box[1]<=r[1]<=b.box[3] and abs(r[2]-b.z(r[0],r[1]))<2 and abs(p[2]-r[2])<=18:
                                height=float(q[2]-p[2]);start=20.;end=30.
                heights.append(height);levels.append(float(p[2]));starts.append(start);ends.append(end)
            if node==last:break
            node=int(links[node,edge])
        if not heights or min(heights)<=18:return False
        first,last=heights[0],heights[-1]
        if max(heights)>max(first,last):first=last=max(heights)
        top=np.array([levels[0]+first,levels[-1]+last])
        if d in (2,3):top=top[::-1]
        start,end=min(starts),max(ends);limit=50. if multi else 25.
        if end-start<10:start=max(0.,(start+end)*.5-5);end=min(limit,start+10);start=end-10
        edge_coord=a.box[axis] if d in (0,3) else a.box[axis+2]
        sign=-1 if d in (0,3) else 1
        bounds=sorted((edge_coord+sign*start,edge_coord+sign*end))
        box=np.zeros(8);box[tangent]=lo;box[tangent+2]=hi;box[axis]=bounds[0];box[axis+2]=bounds[1]
        box[4:]=top[[0,1,1,0]] if axis==1 else top[[0,0,1,1]]
        fence=self.insert(box,a.attr|NO_MERGE|OBSTACLE_TOP,nodes=np.full(4,-1));back=(d+2)%4
        self.disconnect(a.id,b.id,d);self.connect(a.id,fence.id,d,'fence')
        self.connect(fence.id,a.id,back,'fence');self.connect(fence.id,b.id,d,'fence')
        if a.id in b.links[back]:self.disconnect(b.id,a.id,back);self.connect(b.id,fence.id,back,'fence')
        return True

    def remove_overlapping_fences(self):
        fences=[a for a in self.areas.values() if a.attr&OBSTACLE_TOP];remove=set()
        for k,a in enumerate(fences):
            n,m=surface_normals(a.box)
            if min(n[2],m[2])<self.slope_limit:remove.add(a.id)
            for b in fences[k+1:]:
                if np.min(np.minimum(a.box[2:4],b.box[2:4])-np.maximum(a.box[:2],b.box[:2]))<=0:continue
                # SDK obstacle cleanup uses XY overlap; it does not form a union.
                if np.all(a.box[:2]<=b.box[:2]) and np.all(a.box[2:4]>=b.box[2:4]):remove.add(b.id)
                elif np.all(b.box[:2]<=a.box[:2]) and np.all(b.box[2:4]>=a.box[2:4]):remove.add(a.id)
                else:remove.add(b.id if np.prod(a.box[2:4]-a.box[:2])>np.prod(b.box[2:4]-b.box[:2]) else a.id)
        for i in sorted(remove):self.remove(i)
        self.counts['overlapping_or_steep_obstacle_areas_removed']=len(remove)

    def corner_helpers(self):
        # nav_generate.cpp:2407: half-grid helper rectangles at otherwise isolated corners.
        by_corner=defaultdict(list);spatial=Spatial(self.areas);created=0;seen=set()
        for a in self.areas.values():
            for c,p in enumerate(corner_points(a.box)):by_corner[tuple(np.round(p,4))].append((a.id,c))
        for a in list(self.areas.values()):
            for c,point in enumerate(corner_points(a.box)):
                right,left=c,(c+3)%4
                if a.links[right] or a.links[left] or any((d+2)%4 in (right,left) for _,d in a.incoming):continue
                for other,other_corner in by_corner[tuple(np.round(point,4))]:
                    if other==a.id or other_corner!=(c+2)%4:continue
                    b=self.areas[other]
                    for d_other,d_ours in ((left,(left+3)%4),(right,(right+1)%4)):
                        po=point+DIRECTIONS[d_other]*12.5;po[2]=b.z(*po[:2])
                        pu=point+DIRECTIONS[d_ours]*12.5;pu[2]=a.z(*pu[:2])
                        far=point+(DIRECTIONS[d_other]+DIRECTIONS[d_ours])*12.5
                        good,q,_=adjacent(self.scene,point,po,self.slope_limit)
                        if not good or abs(q[2]-point[2])>18:continue
                        good,q,_=adjacent(self.scene,po,far,self.slope_limit)
                        if not good or abs(q[2]-po[2])>18:continue
                        good,r,_=adjacent(self.scene,pu,far,self.slope_limit)
                        if not good or abs(r[2]-pu[2])>18 or abs(r[2]-q[2])>1:continue
                        far=r
                        if spatial.below(far+np.array([0.,0.,.01]),18) is not None:continue
                        points=np.array([point,po,pu,far]);lo=points[:,:2].min(0);hi=points[:,:2].max(0)
                        box=np.r_[lo,hi,np.zeros(4)]
                        for k,p in enumerate(corner_points(box)):box[4+k]=points[np.argmin(np.linalg.norm(points[:,:2]-p[:2],axis=1)),2]
                        key=tuple(np.round(box,4))
                        if key in seen:continue
                        seen.add(key);helper=self.insert(box,a.attr,nodes=np.full(4,-1))
                        self.connect(a.id,helper.id,d_other,'corner_helper');self.connect(helper.id,a.id,(d_other+2)%4,'corner_helper')
                        self.connect(b.id,helper.id,d_ours,'corner_helper');self.connect(helper.id,b.id,(d_ours+2)%4,'corner_helper')
                        created+=1
        self.counts['corner_helpers']=created

    def sdk_connection_fixes(self):
        # nav_generate.cpp:2291: steep stair sides and redundant A->C beside A->B->C.
        stairs_removed=redundant=0;ground=self.arrays[8];positions=self.arrays[0]
        for a in list(self.areas.values()):
            if not(a.attr&STAIRS) or min(a.nodes)<0:continue
            corners=corner_points(a.box)
            for d in range(4):
                edge=(d,(d+1)%4)
                if abs(corners[edge[0],2]-corners[edge[1],2])<18:continue
                for j in list(a.links[d]):
                    b=self.areas[j]
                    if min(b.nodes)<0:continue
                    pair=portal(a.box,b.box,d)
                    if pair is None:continue
                    ni=int(a.nodes[np.argmin(np.linalg.norm(positions[a.nodes]-pair[0],axis=1))])
                    nj=int(b.nodes[np.argmin(np.linalg.norm(positions[b.nodes]-pair[1],axis=1))])
                    other_edge=((d+2)%4,(d+3)%4)
                    if max(ground[ni,list(edge)])>18 or positions[nj,2]+max(ground[nj,list(other_edge)])>positions[ni,2]+18:
                        self.disconnect(a.id,j,d);stairs_removed+=1
        for a in list(self.areas.values()):
            for d in range(4):
                targets=set(a.links[d]);remove=set()
                for b in targets:remove.update(targets.intersection(self.areas[b].links[d]))
                for b in remove:self.disconnect(a.id,b,d);redundant+=1
        self.counts.update(stair_side_links_removed=stairs_removed,redundant_links_removed=redundant)

    def reciprocal_node_pair(self,a,b,d):
        # cells/owner carry the sampled nodes through merge() and split(). A witness
        # must cross this shared edge, not just connect arbitrary nodes in both areas.
        axis=1 if d in (0,2) else 0;tangent=1-axis
        edge=a.box[axis] if d in (0,3) else a.box[axis+2]
        other=b.box[axis+2] if d in (0,3) else b.box[axis]
        lo=max(a.box[tangent],b.box[tangent]);hi=min(a.box[tangent+2],b.box[tangent+2])
        if abs(edge-other)>.01 or hi-lo<=1e-5:return None
        pos,links,owner=self.arrays[0],self.arrays[2],self.arrays[-1]
        source=a.cells;source=source[(owner[source]==a.id)&(links[source,d]>=0)]
        target=links[source,d]
        keep=(owner[target]==b.id)&(links[target,(d+2)%4]==source)
        source,target=source[keep],target[keep]
        p,q=pos[source],pos[target];sign=-1 if d in (0,3) else 1
        keep=((p[:,axis]-edge)*sign<=.01)&((q[:,axis]-edge)*sign>=-.01)
        keep&=(p[:,tangent]>=lo-.01)&(p[:,tangent]<hi-.01)&(np.abs(p[:,tangent]-q[:,tangent])<.01)
        found=np.flatnonzero(keep)
        return (int(source[found[0]]),int(target[found[0]])) if len(found) else None

    def fix_connections(self):
        # Failed movement probes alone cannot disprove special traversal. Ordinary
        # steps must at least retain a reciprocal sampled-node witness at this edge.
        removed=0;counts={};examples=[];evidence=[]
        for a in list(self.areas.values()):
            for d in range(4):
                for j in list(a.links[d]):
                    b=self.areas[j];kind='fence' if (a.attr|b.attr)&OBSTACLE_TOP else self.link_kinds[(a.id,j,d)]
                    c=counts.setdefault(kind,dict(tested=0,midpoint_failed=0,recovered=0,retained_unverified=0,removed_no_portal=0,
                                                  node_backed=0,removed_no_node_evidence=0))
                    c['tested']+=1
                    status,pair=probe_portal(self.scene,a.box,b.box,d,self.slope_limit,True)
                    if status=='midpoint':continue
                    c['midpoint_failed']+=1
                    if status=='no_portal':self.disconnect(a.id,j,d);removed+=1;c['removed_no_portal']+=1
                    elif status=='unverified':
                        witness=self.reciprocal_node_pair(a,b,d) if kind=='ordinary_step' else None
                        if kind=='ordinary_step':
                            evidence.append({'source_generation_id':a.id,'target_generation_id':j,'direction':d,
                                             'node_pair':witness,'positions':None if witness is None else self.arrays[0][list(witness)].tolist()})
                        if kind=='ordinary_step' and witness is None:
                            self.disconnect(a.id,j,d);removed+=1;c['removed_no_node_evidence']+=1;status='no_node_evidence'
                        else:
                            c['retained_unverified']+=1
                            if witness is not None:c['node_backed']+=1
                    else:c['recovered']+=1
                    if sum(e['kind']==kind and e['status']==status for e in examples)<4:
                        examples.append({'kind':kind,'status':status,'source_generation_id':a.id,'target_generation_id':j,
                                         'direction':d,'source_box':a.box.tolist(),'target_box':b.box.tolist(),
                                         'probe':None if pair is None else [p.tolist() for p in pair]})
        self.counts['invalid_links_removed']=removed
        self.link_audit={'policy':'Remove no-portal links and unverified ordinary steps without reciprocal sampled nodes across their shared edge; retain other unverified links',
                         'by_kind':counts,'examples':examples,'ordinary_node_evidence':evidence}

    def nav(self,bsp_size):
        ids={old:i+1 for i,old in enumerate(sorted(self.areas))};areas=[]
        for old in sorted(self.areas):
            a=self.areas[old];b=a.box
            areas.append({'id':ids[old],'attr':a.attr,'nw':(b[0],b[1],b[4]),'se':(b[2],b[3],b[6]),'ne_z':b[5],'sw_z':b[7],
                          'links':[[ids[j] for j in sorted(ls)] for ls in a.links],
                          'hiding':[],'encounters':[],'place':0,'ladders':[[],[]],'occupy':(0.,0.),
                          'light':(1.,1.,1.,1.),'visible':[],'inherit_vis':0})
        return {'magic':0xFEEDFACE,'version':16,'subversion':0,'bsp_size':bsp_size,'analyzed':0,
                'places':[],'has_unnamed':1,'areas':areas,'ladders':[],'trailing':b''}


def build_areas(collision,arrays,index,progress=print,slope_limit=.7):
    timings={};start=time.perf_counter()
    boxes,corners,attrs,valid=rectangles(collision.scene,arrays)
    timings['rectangles']=time.perf_counter()-start
    progress(f'rectangles: {len(boxes):,} areas, {int(valid.sum()):,} closed cells, {timings["rectangles"]:.2f}s',flush=True)
    graph=AreaGraph(collision,arrays,index,boxes,corners,attrs,slope_limit)
    for name,method in [('connections',graph.initial_links),('jump_marking',graph.mark_jumps),('merge',graph.merge),
                        ('overhang_splits',graph.split_overhangs),('square_up',graph.square),('stairs',graph.mark_stairs),
                        ('jump_stitching',graph.stitch_jumps),('fence_tops',graph.fence_tops),('fence_overlap_cleanup',graph.remove_overlapping_fences),('corner_helpers',graph.corner_helpers),
                        ('sdk_connection_fixes',graph.sdk_connection_fixes),('connection_validation',graph.fix_connections)]:
        start=time.perf_counter();method();timings[name]=time.perf_counter()-start
        progress(f'{name}: {len(graph.areas):,} areas, {sum(len(ls) for a in graph.areas.values() for ls in a.links):,} links, {timings[name]:.2f}s',flush=True)
    return graph,timings

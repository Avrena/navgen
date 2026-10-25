"""Source-style cardinal sampling and greedy closed-cell rectangle construction."""
import time

import numpy as np
from numba import types
from numba.typed import Dict
from .runtime import njit, prange, get_num_threads, trace_count

from .collision import BRUSH_MASK, THIN_MIN, THIN_MAX, trace

DIRECTIONS = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,1.,0.],[-1.,0.,0.]])
CORNERS = np.array([[-1.,-1.,0.],[1.,-1.,0.],[1.,1.,0.],[-1.,1.,0.]])
CUBE_MIN = np.full(3,-.5)
CUBE_MAX = np.full(3,.5)


@njit
def endpoint(start,end,r):
    return start+(end-start)*r[0]


@njit
def floor_trace(scene,start,limit=400.):
    end=start.copy();end[2]-=limit
    r=trace(scene,start,end,THIN_MIN,THIN_MAX)
    return r,endpoint(start,end,r)


@njit
def adjacent(scene,origin,target,slope_limit=.7):
    start=origin.copy();end=target.copy();depth=0
    while depth<64:
        r=trace(scene,start,end,THIN_MIN,THIN_MAX)
        point=endpoint(start,end,r)
        if r[4]: return False,point,r
        if abs(point[0]-end[0])<1e-5 and abs(point[1]-end[1])<1e-5:
            r,point=floor_trace(scene,point)
            return not(r[4] or r[0]>=1 or r[3]<slope_limit),point,r
        if depth and np.sum((start[:2]-point[:2])**2)<1:
            return False,point,r
        r,point=floor_trace(scene,point)
        if r[4] or r[0]>=1 or r[3]<slope_limit: return False,point,r
        up=point.copy();up[2]+=18
        r=trace(scene,point,up,THIN_MIN,THIN_MAX)
        start=endpoint(point,up,r);end=target.copy();end[2]=start[2]
        depth+=1
    return False,point,r


@njit
def move_node(scene,origin,target,slope_limit=.7):
    good,point,r=adjacent(scene,origin,target,slope_limit)
    height=0.;obs_start=0.;obs_end=25.
    if not good:
        for height_i in range(18,201):
            start=origin.copy();end=target.copy();start[2]+=height_i;end[2]+=height_i
            tr=trace(scene,start,end,THIN_MIN,THIN_MAX)
            if not tr[4] and tr[0]==1:
                r,point=floor_trace(scene,end)
                if r[4] or r[0]>=1 or r[3]<slope_limit: break
                up=origin.copy();up[2]+=height_i
                ceiling=trace(scene,origin,up,THIN_MIN,THIN_MAX)
                if ceiling[0]<1: break
                good=True;height=float(height_i);break
            else:
                a=endpoint(start,end,tr)
                reverse=trace(scene,end,start,THIN_MIN,THIN_MAX)
                if not reverse[4] and reverse[0]<1:
                    b=endpoint(end,start,reverse)
                    obs_start=np.sqrt(np.sum((a-start)**2))
                    obs_end=np.sqrt(np.sum((b-start)**2))
                    if obs_end==0: obs_end=25.
    if height<18 or point[2]-origin[2]>height-2:
        height=0.;obs_start=0.;obs_end=25.
    return good,point,r,height,obs_start,obs_end


@njit
def overlapped(scene,point):
    # nav_generate.cpp:4305. Full generation has no pre-existing areas.
    total=0
    for c in range(4):
        start=point.copy();start[2]+=35.5
        end=start+CORNERS[c]*25
        r=trace(scene,start,end,CUBE_MIN,CUBE_MAX)
        if r[4] or r[5] or r[0]<.1:
            total+=1;continue
        start=endpoint(start,end,r);end[2]-=71
        r=trace(scene,start,end,CUBE_MIN,CUBE_MAX)
        if r[4] or r[5] or r[0]==1 or r[3]<.7: total+=1
    return total==4


@njit
def embedded_displacement(scene,point):
    up=point.copy();up[2]+=10000
    r=trace(scene,point,up,THIN_MIN,THIN_MAX)
    if r[0]<=0: return False
    start=endpoint(point,up,r)
    r=trace(scene,start,point,THIN_MIN,THIN_MAX)
    return r[0]<1 and endpoint(start,point,r)[2]>point[2]+18


@njit
def crouch_node(scene,point,crouch,blocked,ground):
    # nav_node.cpp:278; GMod x64 RVA 0xd5840: 55 crouch, 71 standing.
    for c in range(4):
        ground[c]=0
        up=point.copy();up[2]+=64
        tr=trace(scene,point,up,THIN_MIN,THIN_MAX)
        ceiling=64*tr[0]
        mins=np.minimum(CORNERS[c]*16,0.)
        maxs=np.maximum(CORNERS[c]*16,0.)
        found=False
        for height in range(int(ceiling)+1):
            start=point.copy();start[2]+=height
            maxs[2]=55
            tr=trace(scene,start,start,mins,maxs)
            if not tr[4]:
                ground[c]=height;maxs[2]=71
                tr=trace(scene,start,start,mins,maxs)
                if tr[4]:crouch[c]=True
                found=True;break
        if not found: ground[c]=64;blocked[c]=True;crouch[c]=True


@njit(parallel=True)
def crouch_nodes(scene,positions,crouch,blocked,ground):
    # CheckCrouch uses the stored node position. Static collision makes its
    # repeated sticky tests equivalent to one evaluation per completed node.
    for i in prange(len(positions)):
        crouch_node(scene,positions[i],crouch[i],blocked[i],ground[i])


@njit
def find_node(index,arrays,point):
    pos,normal,links,parent,visited,nextxy,crouch,blocked,ground,obsh,obsa,obsb,disp,owner=arrays
    key=(int(round(point[0]/25)),int(round(point[1]/25)))
    cur=index[key] if key in index else -1
    while cur>=0:
        if abs(pos[cur,2]-point[2])<11.25: return cur
        cur=nextxy[cur]
    return -1


@njit
def add_node(index,arrays,point,norm,source,count,on_disp):
    pos,normal,links,parent,visited,nextxy,crouch,blocked,ground,obsh,obsa,obsb,disp,owner=arrays
    pos[count]=point;normal[count]=norm;parent[count]=source;disp[count]=on_disp
    key=(int(round(point[0]/25)),int(round(point[1]/25)))
    nextxy[count]=index[key] if key in index else -1
    index[key]=count


@njit
def sample_chunk(scene,seeds,index,arrays,count,current,seedindex,bounds,slope_limit=.7,attempt_limit=10000):
    pos,normal,links,parent,visited,nextxy,crouch,blocked,ground,obsh,obsa,obsb,disp,owner=arrays
    attempts=0
    while attempts<attempt_limit:
        if count+2>=len(pos): return count,current,seedindex,False
        if current<0:
            while seedindex<len(seeds):
                point=seeds[seedindex].copy();seedindex+=1
                point[0]=np.floor(point[0]/25+.5)*25;point[1]=np.floor(point[1]/25+.5)*25
                start=point.copy();start[2]+=35.9
                end=point.copy();end[2]-=400
                r=trace(scene,start,end,THIN_MIN,THIN_MAX)
                if r[5] or r[0]>=1: continue
                point=endpoint(start,end,r)
                if find_node(index,arrays,point)>=0: continue
                add_node(index,arrays,point,np.array([r[1],r[2],r[3]]),-1,count,r[7])
                current=count;count+=1
                break
            if current<0: return count,current,seedindex,True
        direction=-1
        for d in range(4):
            if not(visited[current]&(1<<d)): direction=d;break
        if direction<0:
            current=parent[current];continue
        d=direction;visited[current]|=1<<d;attempts+=1
        start=pos[current].copy();target=start+DIRECTIONS[d]*25
        if (target[0]<bounds[0,0] or target[1]<bounds[0,1] or target[0]>bounds[1,0] or target[1]>bounds[1,1]):continue
        good,point,r,height,oa,ob=move_node(scene,start,target,slope_limit)
        if not good or r[8]&6:continue
        if overlapped(scene,point) or embedded_displacement(scene,point):continue
        other=find_node(index,arrays,point);new=other<0
        if new:
            other=count;add_node(index,arrays,point,np.array([r[1],r[2],r[3]]),current,count,r[7]);count+=1
        links[current,d]=other;obsh[current,d]=height;obsa[current,d]=oa;obsb[current,d]=ob
        delta=start[2]-point[2]
        if abs(delta)<50:
            back=(d+2)%4
            links[other,back]=current;visited[other]|=1<<back
            obsh[other,back]=max(0.,height+delta) if height>0 else 0.
            obsa[other,back]=25-ob;obsb[other,back]=25-oa
        if new: current=other
    return count,current,seedindex,False


def allocate(capacity):
    return (np.zeros((capacity,3)),np.zeros((capacity,3)),np.full((capacity,4),-1,np.int64),
            np.full(capacity,-1,np.int64),np.zeros(capacity,np.uint8),np.full(capacity,-1,np.int64),
            np.zeros((capacity,4),np.bool_),np.zeros((capacity,4),np.bool_),np.zeros((capacity,4)),
            np.zeros((capacity,4)),np.zeros((capacity,4)),np.zeros((capacity,4)),
            np.zeros(capacity,np.bool_),np.full(capacity,-1,np.int64))


def sample(collision,seeds,bounds,progress=print,slope_limit=.7):
    arrays=allocate(262144)
    index=Dict.empty(key_type=types.UniTuple(types.int64,2),value_type=types.int64)
    count,current,seedindex=0,-1,0;bounds=np.asarray(bounds,dtype=np.float64)
    started=time.perf_counter();last=started
    while True:
        count,current,seedindex,done=sample_chunk(collision.scene,seeds,index,arrays,count,current,seedindex,bounds,slope_limit)
        now=time.perf_counter()
        if now-last>5 or done:
            progress(f'sampling: {count:,} nodes, {trace_count(collision.scene):,} traces, seeds {seedindex}/{len(seeds)}, {now-started:.1f}s',flush=True);last=now
        if done: break
        if count+2>=len(arrays[0]):
            grown=allocate(len(arrays[0])*2)
            for a,b in zip(grown,arrays): a[:count]=b[:count]
            arrays=grown
    crouch_nodes(collision.scene,arrays[0][:count],arrays[6][:count],arrays[7][:count],arrays[8][:count])
    # Preserve the M7 diagnostic shape for consumers; there are no speculative probes.
    collision.sampling_speculation={'enabled':False,'lookup_attempts':0,'cache_hits':0,'computed_jobs':0,
        'lookup_hit_rate':0.,'consumed_per_computed_job':0.,'crouch_nodes_tested':count}
    return tuple(a[:count].copy() for a in arrays),index


@njit
def closed_cells(arrays):
    pos,normal,links,parent,visited,nextxy,crouch,blocked,ground,obsh,obsa,obsb,disp,owner=arrays
    valid=np.zeros(len(pos),np.bool_)
    for i in range(len(pos)):
        e,s=links[i,1],links[i,2]
        if e<0 or s<0 or links[e,3]!=i or links[s,0]!=i: continue
        se=links[e,2]
        if se<0 or links[s,1]!=se or links[se,0]!=e or links[se,3]!=s:continue
        valid[i]=True
    return valid


@njit
def obstacles_ok(obsh,node,width,height,x,y):
    # nav_generate.cpp:3173, including the final east and south boundary nodes.
    return (width==1 and height==1) or not (
        (x>0 and obsh[node,3]>18) or (y>0 and obsh[node,0]>18) or
        (x<width-1 and obsh[node,1]>18) or (y<height-1 and obsh[node,2]>18))


@njit
def test_rectangle(scene,arrays,valid,node,width,height):
    pos,normal,links,parent,visited,nextxy,crouch,blocked,ground,obsh,obsa,obsb,disp,owner=arrays
    expected=crouch[node,2]
    n=normal[node];plane=np.dot(n,pos[node]);row=node
    for y in range(height):
        cur=row
        for x in range(width):
            if cur<0 or not valid[cur] or owner[cur]>=0: return False
            if x==0 and y==0: cmask=4
            elif y==0: cmask=12
            elif x==0: cmask=6
            else: cmask=15
            local=False
            for c in range(4):
                if cmask&(1<<c):
                    if blocked[cur,c]: return False
                    local=local or crouch[cur,c]
            if local!=expected: return False
            if not obstacles_ok(obsh,cur,width,height,x,y):return False
            cur=links[cur,1]
            if cur<0: return False
            if (width>1 or height>1) and abs(np.dot(pos[cur],n)-plane)>5: return False
        if not obstacles_ok(obsh,cur,width,height,width,y):return False
        row=links[row,2]
        if row<0: return False
        if (width>1 or height>1) and abs(np.dot(pos[row],n)-plane)>5:return False
    cur=row
    for x in range(width):
        if not obstacles_ok(obsh,cur,width,height,x,height):return False
        cur=links[cur,1]
        if cur<0:return False
        if (width>1 or height>1) and abs(np.dot(pos[cur],n)-plane)>5:return False
    if not obstacles_ok(obsh,cur,width,height,width,height):return False
    if expected:
        mins=np.zeros(3);maxs=np.array([25.,25.,55.]);row=node
        for y in range(height):
            cur=row
            for x in range(width):
                end=pos[cur].copy();end[2]+=64
                r=trace(scene,pos[cur],end,mins,maxs)
                if r[5]:return False
                cur=links[cur,1]
            row=links[row,2]
    return True


@njit
def rectangle_unowned(arrays,node,width,height):
    links,owner=arrays[2],arrays[-1];row=node
    for y in range(height):
        cur=row
        for x in range(width):
            if owner[cur]>=0:return False
            cur=links[cur,1]
        row=links[row,2]
    return True


@njit(parallel=True)
def rectangle_candidates(scene,arrays,valid,width,height):
    result=np.zeros(len(valid),np.bool_);owner=arrays[-1]
    for i in prange(len(valid)):
        if owner[i]<0 and valid[i]:result[i]=test_rectangle(scene,arrays,valid,np.int64(i),width,height)
    return result


@njit
def claim_rectangles(scene,arrays,valid,width,height,candidates,boxes,corners,attrs,nareas):
    pos,normal,links,parent,visited,nextxy,crouch,blocked,ground,obsh,obsa,obsb,disp,owner=arrays
    for i in range(len(pos)-1,-1,-1):
        if owner[i]>=0 or not valid[i]:continue
        if len(candidates):
            if not candidates[i] or not rectangle_unowned(arrays,i,width,height):continue
        elif not test_rectangle(scene,arrays,valid,i,width,height):continue
        row=i;ne=-1
        for y in range(height):
            cur=row
            for x in range(width):
                owner[cur]=nareas;cur=links[cur,1]
            if y==0:ne=cur
            row=links[row,2]
        sw=row;se=row
        for x in range(width):se=links[se,1]
        corners[nareas]=np.array([i,ne,se,sw])
        boxes[nareas]=np.array([pos[i,0],pos[i,1],pos[se,0],pos[se,1],pos[i,2],pos[ne,2],pos[se,2],pos[sw,2]])
        attr=1 if crouch[i,2] else 0
        # Only corner-to-corner directions are internal to this rectangle.
        if (obsh[i,1]>18 or obsh[i,2]>18 or obsh[ne,2]>18 or obsh[ne,3]>18
            or obsh[se,0]>18 or obsh[se,3]>18 or obsh[sw,0]>18 or obsh[sw,1]>18):
            attr|=8192
        attrs[nareas]=attr;nareas+=1
    return nareas


def rectangles(scene,arrays):
    valid=closed_cells(arrays);n=len(valid)
    boxes=np.empty((n,8));corners=np.empty((n,4),np.int64);attrs=np.empty(n,np.int64)
    nareas=0;width=50;height=50;parallel=get_num_threads()>1
    while width>0 and height>0:
        candidates=rectangle_candidates(scene,arrays,valid,width,height) if parallel else np.empty(0,np.bool_)
        nareas=claim_rectangles(scene,arrays,valid,width,height,candidates,boxes,corners,attrs,nareas)
        if width>=height:width-=1
        else:height-=1
    return boxes[:nareas],corners[:nareas],attrs[:nareas],valid


@njit
def can_traverse(scene,start,end,slope_limit=.7):
    if np.sqrt(np.sum((start[:2]-end[:2])**2))<1e-4 and abs(start[2]-end[2])<18:return True
    target=end.copy();target[2]=start[2]
    good,point,r,height,oa,ob=move_node(scene,start,target,slope_limit)
    return good and abs(point[2]-end[2])<=18 and start[2]-end[2]<400 and end[2]-start[2]<=200

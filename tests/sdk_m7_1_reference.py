"""Frozen M7.1 trace/move algorithms for differential tests (1059ee2)."""
import numpy as np
from navgen.runtime import njit, get_thread_id
from navgen.collision import BRUSH_MASK, EPS, THIN_MIN, THIN_MAX, clip
from navgen.sampling import adjacent, floor_trace, endpoint

@njit
def trace(scene, start, end, mins, maxs, mask=BRUSH_MASK, walk=True, brute=False):
    planes,pflags,first,count,lo,hi,meta,front,blo,bhi,escape,leaf,order,stats = scene
    stats[get_thread_id()*8]+=1
    best=1.;nx=0.;ny=0.;nz=0.;ss=False;als=False;contents=0;disp=False;surf=0;hit=-1
    qlo=np.minimum(start,end)+mins-EPS
    qhi=np.maximum(start,end)+maxs+EPS
    node=0
    limit=len(order) if brute else len(blo)
    while node<limit:
        if brute:
            begin,finish=node,node+1
            node+=1
        else:
            if (blo[node,0]>qhi[0] or blo[node,1]>qhi[1] or blo[node,2]>qhi[2] or
                bhi[node,0]<qlo[0] or bhi[node,1]<qlo[1] or bhi[node,2]<qlo[2]):
                node=escape[node];continue
            begin,finish=leaf[node,0],leaf[node,1]
            node+=1
            if begin<0: continue
        for ii in range(begin,finish):
            bid=order[ii]
            if not(meta[bid,0]&mask): continue
            if walk and meta[bid,1]: continue
            if meta[bid,2] and not(mask&0x2000000): continue
            if meta[bid,3] and not(mask&2): continue
            if meta[bid,4] and not(mask&0x4000): continue
            if (lo[bid,0]>qhi[0] or lo[bid,1]>qhi[1] or lo[bid,2]>qhi[2] or
                hi[bid,0]<qlo[0] or hi[bid,1]<qlo[1] or hi[bid,2]<qlo[2]): continue
            isdisp=meta[bid,5]==2
            if isdisp:
                motion=0.;back=-front[bid,3]
                for axis in range(3):
                    n=front[bid,axis]
                    motion+=n*(end[axis]-start[axis])
                    back+=n*(start[axis]+(maxs[axis] if n>=0 else mins[axis]))
                if motion>1e-9 or back < -EPS: continue
            fraction,startsolid,allsolid,plane=clip(planes,start,end,mins,maxs,first[bid],count[bid])
            ss=ss or startsolid
            als=als or allsolid
            if fraction<best:
                best=fraction;contents=meta[bid,0];disp=isdisp;hit=bid
                if plane>=0:
                    nx,ny,nz=planes[plane,0],planes[plane,1],planes[plane,2]
                    surf=pflags[plane]
                elif isdisp:
                    nx,ny,nz=front[bid,0],front[bid,1],front[bid,2]
    return best,nx,ny,nz,ss,als,contents,disp,surf,hit

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

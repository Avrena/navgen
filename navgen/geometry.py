"""Small full-hull helpers for optional entity seeds and ladder witnesses."""
import math
import numpy as np
from .collision import trace
from .runtime import njit
MINS=np.array([-16.,-16.,0.])
DUCK=np.array([16.,16.,55.])
STAND=np.array([16.,16.,71.])

@njit
def clear_move(scene,start,end,height=55.):
    maximum=np.array([16.,16.,height]);r=trace(scene,start,end,MINS,maximum)
    return r[0]==1. and not r[4] and not r[5]


@njit
def hull_move(scene,start,end,jump=64.,drop=400.):
    """Swept full hull: direct, 18-unit step, then bounded up/across/down.

    This is geometric feasibility, not a native physics or NextBot simulation.
    Jump heights are searched at one-unit intervals through the available headroom.
    """
    dz=end[2]-start[2]
    if dz>jump+.01 or dz<=-drop:return 0
    a=start.copy();b=end.copy();a[2]+=.01;b[2]+=.01
    if clear_move(scene,a,b):return 1 if abs(dz)<=18.01 else (3 if dz>0 else 4)
    high=a.copy();high[2]+=jump
    ceiling=trace(scene,a,high,MINS,DUCK)
    if ceiling[4]:return 0
    available=max(0.,jump*ceiling[0]-.05)
    first=max(18.,dz+.05)
    if first>available:return 0
    # The highest usable arc often clears the obstruction in a single probe.
    for trial in range(int(math.ceil(available-first))+2):
        h=available if trial==0 else first+trial-1
        if h>available:continue
        up=a.copy();up[2]+=h;across=b.copy();across[2]=up[2]
        if clear_move(scene,up,across) and clear_move(scene,across,b):
            return 2 if h<=18.01 and abs(dz)<=18.01 else (4 if dz<-18 else 3)
    return 0


@njit
def centre_floor(scene,point,slope):
    zero=np.zeros(3)
    # A fitted NAV surface may lie a few units below a displacement. Starting
    # almost on that surface can miss its one-sided triangles entirely.
    for offset in (18.,36.,72.,2.,.05):
        a=point.copy();a[2]+=offset;b=point.copy();b[2]-=400.
        r=trace(scene,a,b,zero,zero)
        if r[4]:continue
        if r[0]>=1:continue
        return r[3]>=slope and not(r[8]&6),a[2]+(b[2]-a[2])*r[0]-.03125/max(r[3],.01),np.array([r[1],r[2],r[3]])
    return False,np.nan,np.zeros(3)


@njit
def support_origin(scene,point,slope=.55):
    good,z,normal=centre_floor(scene,point,slope)
    if not good:return False,point.copy()
    rise=16.*(abs(normal[0])+abs(normal[1]))/max(normal[2],.01)
    for offset in (.1,rise+.1,2.1,4.1,8.1,12.1,18.1):
        high=point.copy();high[2]=z+offset;low=high.copy();low[2]=z-2.
        r=trace(scene,high,low,MINS,DUCK)
        if r[4] or r[0]>=1 or r[3]<slope or r[8]&6:continue
        p=high+(low-high)*r[0];p[2]+=.01
        r=trace(scene,p,p,MINS,DUCK)
        if not(r[4] or r[5]):return True,p
    return False,point.copy()



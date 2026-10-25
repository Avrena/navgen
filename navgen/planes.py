"""Compiled small-vector plane construction, preserving input and plane order."""
import numpy as np
from .runtime import njit, prange


@njit
def axis_planes(points,axes):
    out=np.empty((len(axes)*2,4));i=0
    for axis in axes:
        for sign in (1.,-1.):
            n=sign*axis
            out[i,:3]=n;out[i,3]=np.max(points@n);i+=1
    return out


@njit
def edge_axes(edges,epsilon):
    axes=np.empty((len(edges)*3,3));count=0
    for edge in edges:
        for axis in np.eye(3):
            n=np.cross(edge,axis);length=np.sqrt(np.dot(n,n))
            if length>epsilon:axes[count]=n/length;count+=1
    return axes[:count]


@njit
def bevel_planes(points,edges):
    out=np.empty((len(edges)*3,4));count=0
    for a,b in edges:
        edge=points[b]-points[a]
        for axis in np.eye(3):
            n=np.cross(edge,axis);length=np.sqrt(np.dot(n,n))
            if length<1e-8:continue
            n/=length;d=np.dot(n,points[a]);distances=points@n-d
            if distances.max()<1e-5:out[count,:3]=n;out[count,3]=d;count+=1
            elif distances.min()>-1e-5:out[count,:3]=-n;out[count,3]=-d;count+=1
    return out[:count]


@njit
def triangle_kernel(points):
    edges=np.empty((3,3))
    for i in range(3):edges[i]=points[(i+1)%3]-points[i]
    normal=np.cross(edges[0],-edges[2]);length=np.sqrt(np.dot(normal,normal))
    if length<1e-8:return np.empty((0,4)),np.zeros(4)
    normal/=length
    extra=edge_axes(edges,1e-8);axes=np.empty((4+len(extra),3))
    axes[0]=normal;axes[1:4]=np.eye(3);axes[4:]=extra
    front=np.empty(4);front[:3]=normal;front[3]=points[0]@normal
    return axis_planes(points,axes),front


@njit(parallel=True)
def triangle_batch(points):
    planes=np.empty((len(points),26,4));fronts=np.empty((len(points),4));counts=np.empty(len(points),np.int64)
    for i in prange(len(points)):
        p,f=triangle_kernel(points[i]);counts[i]=len(p)
        planes[i,:len(p)]=p;fronts[i]=f
    return planes,fronts,counts

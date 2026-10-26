"""Conservative v16 ladder records from static BSP ladder entities.

Records and reciprocal area references are format-checked. Swept hull witnesses
test a possible climb and dismount; they do not simulate native ladder use.
"""
import numpy as np
from .areas import height_at
from .compare import boxes
from .geometry import clear_move,hull_move

DIRECTIONS=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,1.,0.],[-1.,0.,0.]])


def direction(vector):
    if abs(vector[0])>abs(vector[1]):return 1 if vector[0]>0 else 3
    return 2 if vector[1]>0 else 0


def top_slot(facing,delta):
    d=direction(delta)
    return 3 if d==facing else (0 if d==(facing+2)%4 else (1 if d==(facing+3)%4 else 2))


def climb_lines(spec):
    for facing,normal in enumerate(DIRECTIONS):
        if spec['axis'] is None:
            for offset in (0.,16.1,32.1):
                yield facing,spec['bottom']+normal*offset,spec['top']+normal*offset
        else:
            axis=spec['axis'];tangent=1-axis
            if abs(normal[axis])!=1:continue
            face=spec['hi'][axis] if normal[axis]>0 else spec['lo'][axis]
            low=spec['lo'][tangent]+min(16.,spec['width']/4);high=spec['hi'][tangent]-min(16.,spec['width']/4)
            for coordinate in dict.fromkeys([(low+high)*.5,low,high]):
                b=spec['bottom'].copy();t=spec['top'].copy();b[axis]=t[axis]=face+normal[axis]*16.1
                b[tangent]=t[tangent]=coordinate;yield facing,b,t


def build_ladders(nav,collision,specs,support_convention=False,slope=.55):
    bounds=boxes(nav);scene=collision.scene;records=[];rejected=[]
    def attachments(endpoint,upper,mid):
        x=np.clip(endpoint[0],bounds[:,0]+.01,bounds[:,2]-.01)
        y=np.clip(endpoint[1],bounds[:,1]+.01,bounds[:,3]-.01)
        near=np.flatnonzero((np.abs(x-endpoint[0])<=80)&(np.abs(y-endpoint[1])<=80))
        candidates=[]
        for i in near:
            p=np.array([x[i],y[i],height_at(bounds[i],x[i],y[i])])
            if support_convention:
                from .geometry import support_origin
                good,p=support_origin(scene,p,slope)
                if not good:continue
            if abs(p[2]-endpoint[2])>64 or (p[2]>mid)!=upper:continue
            candidates.append((float(np.linalg.norm(p-endpoint)),int(i),p))
        return sorted(candidates,key=lambda item:(item[0],item[1]))[:32]
    for spec in specs:
        source={k:spec.get(k) for k in ('class','hammerid')};bottom=spec['bottom'];top=spec['top'];mid=(top[2]+bottom[2])*.5
        options=[];bottom_count=0;top_count=0;clear_count=0
        for facing,b,t in climb_lines(spec):
            bottoms=attachments(b,False,mid);tops=attachments(t,True,mid)
            bottom_count=max(bottom_count,len(bottoms));top_count=max(top_count,len(tops))
            for bd,bi,bp in bottoms:
                start=b.copy();start[2]=max(bottom[2],bp[2])+.01
                if not hull_move(scene,bp,start) or not hull_move(scene,start,bp):continue
                for td,ti,tp in tops:
                    if ti==bi:continue
                    delta=tp[:2]-t[:2]
                    if np.linalg.norm(delta)<.01:delta=(bounds[ti,:2]+bounds[ti,2:4])*.5-t[:2]
                    slot=top_slot(facing,delta)
                    if slot==3:continue
                    end=t.copy();end[2]=max(top[2],tp[2])+.01
                    if end[2]<=start[2]+18 or not clear_move(scene,start,end):continue
                    clear_count+=1
                    if not hull_move(scene,end,tp) or not hull_move(scene,tp,end):continue
                    options.append((bd+td+np.linalg.norm(start-b)+np.linalg.norm(end-t),facing,bi,ti,slot,start,end,bp,tp))
        if not options:
            rejected.append({**source,'reason':'no clear climb with reciprocal supported top and bottom attachments',
                             'bottom_candidates':bottom_count,'top_candidates':top_count,'clear_climb_trials':clear_count});continue
        _,facing,bi,ti,slot,start,end,bp,tp=min(options,key=lambda v:(v[0],v[1],v[2],v[3]))
        ident=len(nav['ladders'])+1;normal=DIRECTIONS[facing]
        face_bottom=start-normal*16.1;face_top=end-normal*16.1
        areas=[0]*5;areas[slot]=nav['areas'][ti]['id'];areas[4]=nav['areas'][bi]['id']
        nav['ladders'].append({'id':ident,'width':min(spec['width'],32.),'top':tuple(face_top),'bottom':tuple(face_bottom),
                               'length':float(np.linalg.norm(face_top-face_bottom)),'dir':facing,'areas':tuple(areas)})
        nav['areas'][bi]['ladders'][0].append(ident);nav['areas'][ti]['ladders'][1].append(ident)
        nav['areas'][bi]['attr']|=8192;nav['areas'][ti]['attr']|=8192
        records.append({**source,'id':ident,'bottom_area':areas[4],'top_area':areas[slot],'top_slot':slot,
                        'climb_bottom':start.tolist(),'climb_top':end.tolist(),
                        'bottom_witness':bp.tolist(),'top_witness':tp.tolist()})
    return {'sources':len(specs),'written':len(records),'records':records,'skipped':rejected,'native_traversal_verified':False}

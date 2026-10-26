"""Independent union, height and directed-link checks for SDK contractions."""
from collections import Counter,defaultdict
import numpy as np
from .areas import corner_points,height_at,portal_batch,PROBE_STATUS
from .compare import boxes
from .merge import edge_class


def audit_merge(original,merged,mapping,scene=None,slope=.7,tolerance=.5):
    before={a['id']:a for a in original['areas']};after={a['id']:a for a in merged['areas']}
    ob=dict(zip(before,boxes(original)));mb=dict(zip(after,boxes(merged)))
    mapping={int(k):int(v) for k,v in mapping.items()};groups=defaultdict(list)
    assert set(mapping)==set(before)
    for i,j in mapping.items():groups[j].append(i)
    assert set(groups)==set(after)
    maximum=0.;total=0.
    for j,ids in groups.items():
        b=mb[j];parts=[ob[i] for i in ids];area=(b[2]-b[0])*(b[3]-b[1]);total+=area
        assert sum((p[2]-p[0])*(p[3]-p[1]) for p in parts)==area
        for n,i in enumerate(ids):
            p=ob[i];a=before[i]
            assert p[0]>=b[0] and p[1]>=b[1] and p[2]<=b[2] and p[3]<=b[3]
            assert a['attr']==after[j]['attr'] and a['place']==after[j]['place']
            if any(a['ladders']):assert i==j and len(ids)==1 and a['ladders']==after[j]['ladders']
            for q in parts[n+1:]:assert min(p[2],q[2])<=max(p[0],q[0]) or min(p[3],q[3])<=max(p[1],q[1])
            for x,y,z in corner_points(p):maximum=max(maximum,abs(height_at(b,x,y)-z))
    assert maximum<=tolerance+1e-8
    expected=set()
    for i,a in before.items():
        for d,links in enumerate(a['links']):
            for j in links:
                u,v=mapping[i],mapping[j]
                if u==v:continue
                expected.add((u,v,d));assert edge_class(ob[i],ob[j],d)==edge_class(mb[u],mb[v],d)
    edges=sorted((i,j,d) for i,a in after.items() for d,ls in enumerate(a['links']) for j in ls)
    assert expected==set(edges)
    changed=np.array([len(groups[i])>1 or len(groups[j])>1 for i,j,d in edges],np.bool_)
    links={'total':len(edges),'changed':int(changed.sum()),'unchanged':int((~changed).sum()),'directed_contraction_equal':True}
    if scene is not None and edges:
        codes,_=portal_batch(scene,np.array([mb[i] for i,j,d in edges]),np.array([mb[j] for i,j,d in edges]),np.array([d for i,j,d in edges],np.int64),slope)
        verified=np.isin(codes,[1,2,3]);assert verified[changed].all()
        links.update(changed_unverified=int((changed&~verified).sum()),inherited_unchanged_unverified=int((~changed&~verified).sum()),
                     witnesses=dict(Counter(PROBE_STATUS[c] for c in codes)))
    return {'groups':len(groups),'original_areas':len(before),'exact_xy_union':True,'summed_xy_area':float(total),
            'max_original_corner_height_error':float(maximum),'height_bound':tolerance,
            'height_proof':'Difference of bilinear surfaces on each original rectangle reaches its extrema at corners.',
            'coverage_scope':'Same XY union and SDK surface convention, not an independent claim of engine coverage.',
            'links':links}

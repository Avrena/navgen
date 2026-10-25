"""Read-only node-evidence audit of the preserved M2 bbox runs."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

sys.dont_write_bytecode=True
from navgen.areas import AreaGraph as CurrentGraph

root=Path(__file__).resolve().parents[1]
archive=root/'work/navgen-M2.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest()=='47651e2020b71f8cbda0e9c808d209b0a6cbf7813a040e5bfd72ef3da4171875'
folder=root/'work/m3/replay/m2_frozen';folder.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(archive) as package:
    for name in package.namelist():
        if name.startswith('navgen/navgen/') and name.endswith('.py'):
            target=folder/Path(name).name;raw=package.read(name)
            if target.exists():assert target.read_bytes()==raw
            else:target.write_bytes(raw)
sys.path.insert(0,str(folder.parent))
from m2_frozen.__main__ import generate
from m2_frozen.areas import AreaGraph,OBSTACLE_TOP,probe_portal

original=AreaGraph.fix_connections
latest=[]
def audit_links(self):
    original(self);latest.clear();ids={i:k+1 for k,i in enumerate(sorted(self.areas))}
    for a in self.areas.values():
        for d,links in enumerate(a.links):
            for j in sorted(links):
                b=self.areas[j]
                if (a.attr|b.attr)&OBSTACLE_TOP or self.link_kinds[a.id,j,d]!='ordinary_step':continue
                if probe_portal(self.scene,a.box,b.box,d,self.slope_limit,True)[0]!='unverified':continue
                pair=CurrentGraph.reciprocal_node_pair(self,a,b,d)
                latest.append({'source_nav_id':ids[a.id],'target_nav_id':ids[j],'source_generation_id':a.id,
                               'target_generation_id':j,'direction':d,'source_box':a.box.tolist(),'target_box':b.box.tolist(),
                               'node_pair':pair,'positions':None if pair is None else self.arrays[0][list(pair)].tolist()})
AreaGraph.fix_connections=audit_links
evidence=root/'reports/measurements/m3';evidence.mkdir(parents=True,exist_ok=True)
for label in ('map_a-s0.55-extra','map_b-s0.70-bsp'):
    baseline=json.loads((root/'reports/measurements/m2'/f'{label}-generation.json').read_text())
    print('Frozen M2 node audit:',label,flush=True)
    result=generate(baseline['map'],out=folder.parent/f'{label}.nav',missing_phy='bbox',slope_limit=baseline['slope_limit'],
                    seeds=root/'work/m2/component-seeds.txt' if label.endswith('extra') else None,
                    progress=lambda *a,**k:print(*a,**k) if not str(a[0]).startswith(('missing MDL:','missing PHY')) else None)
    assert result['nav_sha256']==baseline['nav_sha256'],label
    expected=baseline['link_audit']['by_kind']['ordinary_step']['retained_unverified'];assert len(latest)==expected
    report={'label':label,'frozen_M2_archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),
            'nav_sha256':result['nav_sha256'],'matches_original_M2':True,'unverified_ordinary_links':len(latest),
            'node_backed':sum(r['node_pair'] is not None for r in latest),
            'without_node_evidence':sum(r['node_pair'] is None for r in latest),'links':list(latest)}
    (evidence/f'{label}-m2-node-audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='links'},indent=2),flush=True)

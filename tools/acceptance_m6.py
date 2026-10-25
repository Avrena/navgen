"""M6 offline generation, corrected link classification, and drop witnesses."""
import argparse
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np

from navgen.__main__ import generate
import navgen.areas as areas
from navgen.collision import Collision
from navgen.hiding_compare import compare_files
from navgen.navio import read_nav,write_nav

sys.path.insert(0,str(Path(__file__).resolve().parent))
from acceptance_m5 import CASES,ROOT,grid_comparison,sha,source_hashes,unverified
from link_audit import classify


def run(names):
    evidence=ROOT/'reports/measurements/m6';evidence.mkdir(parents=True,exist_ok=True)
    source=source_hashes();cached=None;cached_bsp=None
    original_load=Collision.load;original_initial=areas.AreaGraph.initial_links
    original_jump=areas.jump_down;original_below=areas.Spatial.below
    for name in names:
        bsp,slope,seeds,reference,filename=CASES[name]
        output=ROOT/'work/m6'/filename;baseline=ROOT/'work/m5'/filename
        inputs={str(p):sha(p) for p in (bsp,reference,baseline,*([seeds] if seeds else []))}
        drops=[];lookup_differences=[]
        def load(*args,**kwargs):
            nonlocal cached,cached_bsp
            if cached_bsp!=bsp:
                cached=original_load(*args,**kwargs);cached_bsp=bsp
            return cached
        def capture_jump(scene,start,end):
            result=original_jump(scene,start,end)
            if result:drops.append({'start':start.tolist(),'goal':end.tolist()})
            return result
        def capture_below(spatial,point,limit=400,above=.001):
            result=original_below(spatial,point,limit,above)
            if above==5.:
                old=original_below(spatial,point,limit)
                if old is not result:lookup_differences.append({'point':point.tolist(),
                    'old':None if old is None else old.id,'new':None if result is None else result.id})
            return result
        def initial(graph):
            with patch('navgen.areas.jump_down',capture_jump),patch.object(areas.Spatial,'below',capture_below):
                original_initial(graph)
        with patch.object(Collision,'load',load),patch.object(areas.AreaGraph,'initial_links',initial):
            result=generate(bsp,out=output,seeds=seeds,slope_limit=slope)
        assert source==source_hashes(),'Source changed during measurement'
        assert all(sha(p)==value for p,value in inputs.items()),'Input changed during measurement'
        raw=output.read_bytes();nav=read_nav(raw);old=read_nav(baseline.read_bytes());ref=read_nav(reference.read_bytes())
        assert nav['bsp_size']==bsp.stat().st_size and nav['analyzed']==1
        assert write_nav(nav)==raw
        before=grid_comparison(old,ref);after=grid_comparison(nav,ref)
        m5=json.loads((ROOT/f'reports/measurements/m5/{name}-generation.json').read_text(encoding='utf-8'))
        audit=classify(nav,ref,cached.scene,slope)
        metrics={'reference':str(reference),'reference_sha256':sha(reference),
                 'reference_bsp_size_matches':ref['bsp_size']==bsp.stat().st_size,
                 'm5_vs_reference':before,'m6_vs_reference':after,
                 'delta':{section:{k:after[section][k]-before[section][k] for k in before[section]}
                          for section in ('coverage','ours_graph','directed_transitions')},
                 'unverified_links':{'m5':unverified(m5),'m6':unverified(result)},
                 'coverage_nonregression':all(after['coverage'][k]>=before['coverage'][k] for k in ('precision','recall')),
                 'transition_nonregression':all(after['directed_transitions'][k]>=before['directed_transitions'][k] for k in ('precision','recall')),
                 'content_audit_unchanged':result['content_audit']==m5['content_audit']}
        assert metrics['content_audit_unchanged']
        result.update(measured_source_sha256=source,input_sha256=inputs,
                      invocation='generate() public API; collision reused in memory between identical BSP inputs',
                      runner_argv=sys.argv,python=sys.executable)
        payloads={'generation':result,'comparison':metrics,'links':audit,'hiding':compare_files(output,reference),
                  'drop-witnesses':{'accepted_node_drop_probes':drops,'five_unit_lookup_changes':lookup_differences}}
        for suffix,data in payloads.items():
            (evidence/f'{name}-{suffix}.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
        print(json.dumps({'case':name,'areas':result['areas'],'links':result['links'],'hiding':result['hiding_spots']['total'],
                          'obstacle_tops':audit['obstacle_top_areas'],'unverified':audit['total'],'groups':audit['groups'],
                          'coverage':after['coverage'],'transitions':after['directed_transitions'],
                          'lookup_changes':len(lookup_differences)}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('case',choices=[*CASES,'site','all']);a=p.parse_args()
    run(list(CASES) if a.case=='all' else ['map_a-bsp','map_a-seeded'] if a.case=='site' else [a.case])

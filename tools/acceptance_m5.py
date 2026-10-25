"""M5 CLI matrix, coverage/transition deltas, hiding and retained-link evidence."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

from navgen.bsp import DEFAULT_GAME
from navgen.compare import cells,graph_arrays,transition_counts
from navgen.hiding_compare import compare_files
from navgen.navio import read_nav,write_nav

ROOT=Path(__file__).resolve().parents[1]
EXTERNAL=ROOT/'work/external'
SITE=DEFAULT_GAME/'maps/map_a.bsp'
CASES={
    'construct':(DEFAULT_GAME/'maps/gm_construct.bsp',.7,None,DEFAULT_GAME/'maps/gm_construct.nav','gm_construct.nav'),
    'map_b':(DEFAULT_GAME/'maps/map_b.bsp',.7,None,DEFAULT_GAME/'maps/map_b.nav','map_b.nav'),
    'map_a-bsp':(SITE,.55,None,EXTERNAL/'ingame/results/engine_map_a.nav','map_a-bsp.nav'),
    'map_a-seeded':(SITE,.55,ROOT/'work/m2/component-seeds.txt',EXTERNAL/'ingame/results/engine_map_a.nav','map_a.nav')}


def grid_comparison(ours,reference):
    """Same grid and directed-transition definitions as navgen.compare; no collision reload."""
    op,oi=cells(ours);rp,ri=cells(reference)
    odist,index=cKDTree(rp).query(op,distance_upper_bound=18.00001)
    rdist,_=cKDTree(op).query(rp,distance_upper_bound=18.00001)
    om=np.isfinite(odist);rm=np.isfinite(rdist)
    ol,og,_=graph_arrays(ours);rl,rg,_=graph_arrays(reference)
    xy=np.rint((op[:,:2]-12.5)/25).astype(np.int64)
    keys=(xy[:,0]+65536)*2**32+xy[:,1]+65536
    order=np.argsort(keys,kind='stable');mapped=np.full(len(op),-1,np.int64);mapped[om]=ri[index[om]]
    counts=transition_counts(op[order],oi[order],mapped[order],keys[order],np.array([-1,2**32,1,-2**32],np.int64),
                             ol,len(ours['areas']),rl,len(reference['areas']))
    return {'coverage':{'ours_cells':len(op),'reference_cells':len(rp),'ours_matched':int(om.sum()),
                        'reference_matched':int(rm.sum()),'precision':float(om.mean()),'recall':float(rm.mean())},
            'ours_graph':og,'reference_graph':rg,
            'directed_transitions':{'candidates':int(counts[0]),'both_connected':int(counts[1]),'ours_only':int(counts[2]),
                                    'reference_only':int(counts[3]),'both_disconnected':int(counts[4]),
                                    'precision':float(counts[1]/max(1,counts[1]+counts[2])),
                                    'recall':float(counts[1]/max(1,counts[1]+counts[3]))}}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes():
    return {str(p.relative_to(ROOT)).replace('\\','/'):sha(p)
            for folder in ('navgen','tests','tools') for p in sorted((ROOT/folder).glob('*.py'))}


def unverified(result):
    kinds={k:v['retained_unverified'] for k,v in result['link_audit']['by_kind'].items()}
    return {'total':sum(kinds.values()),'by_kind':kinds}


def run(name):
    bsp,slope,seeds,reference,filename=CASES[name]
    output=ROOT/'work/m5'/filename;baseline=ROOT/'work/m4'/filename
    evidence=ROOT/'reports/measurements/m5';evidence.mkdir(parents=True,exist_ok=True)
    if seeds is None and bsp.with_suffix('.navseeds.txt').exists():raise ValueError('BSP-only acceptance requires no sidecar')
    source=source_hashes()
    inputs={str(p):sha(p) for p in (bsp,reference,baseline,*([seeds] if seeds else []))}
    command=[sys.executable,'-I','-B','-m','navgen',str(bsp),'--out',str(output),'--slope-limit',str(slope)]
    if seeds:command+=['--seeds',str(seeds)]
    started=time.perf_counter();result=None
    with subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace') as process:
        for line in process.stdout:
            if line.startswith('{'):result=json.loads(line)
            elif not line.startswith(('missing MDL:','missing PHY')):print(line,end='',flush=True)
        if process.wait():raise RuntimeError(f'Generation failed: {process.returncode}')
    if result is None:raise RuntimeError('Missing CLI measurement')
    assert source==source_hashes(),'Source changed during measurement'
    assert all(sha(p)==value for p,value in inputs.items()),'Input changed during measurement'
    result.update(command_argv=command,command_line=subprocess.list2cmdline(command),cli_seconds=time.perf_counter()-started,
                  measured_source_sha256=source,input_sha256=inputs)
    raw=output.read_bytes();generated=read_nav(raw);old=read_nav(baseline.read_bytes());ref=read_nav(reference.read_bytes())
    assert generated['bsp_size']==bsp.stat().st_size and generated['analyzed']==1
    assert write_nav(read_nav(raw))==raw
    result['m4_byte_identical']=raw==baseline.read_bytes()
    if name=='construct':assert result['m4_byte_identical'],'Construct has no affected triangles'
    if name=='map_a-bsp':
        experiment=EXTERNAL/'exp/dispfix2_bspseeds_055.nav';bare=copy.deepcopy(generated);bare['analyzed']=0
        for area in bare['areas']:area['hiding']=[]
        same=write_nav(bare)==experiment.read_bytes()
        result['reference_experiment_without_hiding']={'path':str(experiment),'sha256':sha(experiment),'byte_identical':same}
        assert same,'M5 geometry differs from the independently generated reference experiment'
    (evidence/f'{name}-generation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    hiding=compare_files(output,reference)
    (evidence/f'{name}-hiding.json').write_text(json.dumps(hiding,indent=2),encoding='utf-8')
    m4=json.loads((ROOT/f'reports/measurements/m4/{name}-generation.json').read_text(encoding='utf-8'))
    before=grid_comparison(old,ref);after=grid_comparison(generated,ref)
    before_links=unverified(m4);after_links=unverified(result)
    metrics={'reference':str(reference),'reference_sha256':sha(reference),'m4':str(baseline),'m5':str(output),
             'reference_bsp_size_matches':ref['bsp_size']==bsp.stat().st_size,
             'coverage_method':'25-unit XY cell centers; height match <=18; duplicate floors rounded to 1 unit',
             'transition_scope':'Directed cardinal neighbor cells on common coverage; area boundary in either mesh; excludes shared internal subdivisions and long jump/drop gaps',
             'm4_vs_reference':before,'m5_vs_reference':after,
             'delta':{section:{key:after[section][key]-before[section][key] for key in before[section]}
                      for section in ('coverage','ours_graph','directed_transitions')},
             'm5_vs_m4':grid_comparison(generated,old),
             'unverified_links':{'m4':before_links,'m5':after_links,'total_delta':after_links['total']-before_links['total'],
                                'scope':'Retained unverified generation links from each measured run, classified before serialization; no native route assertion'},
             'content_audit_unchanged':result['content_audit']==m4['content_audit']}
    assert metrics['content_audit_unchanged'],'Content resolution changed from M4'
    (evidence/f'{name}-comparison.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
    print(json.dumps({'case':name,'areas':result['areas'],'links':result['links'],'nodes':result['nodes'],
                      'hiding':result['hiding_spots'],'coverage_m4':before['coverage'],'coverage_m5':after['coverage'],
                      'transitions_m5':after['directed_transitions'],'unverified':metrics['unverified_links'],
                      'hiding_reference_match':hiding['reference_match_fraction']}),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('case',choices=[*CASES,'all']);args=parser.parse_args()
    for name in CASES if args.case=='all' else [args.case]:run(name)


if __name__=='__main__':main()

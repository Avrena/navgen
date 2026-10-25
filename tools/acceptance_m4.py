"""M4 real-CLI runs and hiding-spot comparisons; output remains inside the project."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from navgen.bsp import DEFAULT_GAME
from navgen.hiding_compare import compare_files
from navgen.navio import read_nav,write_nav

ROOT=Path(__file__).resolve().parents[1]
ENGINE=ROOT/'work/ingame/results'
SITE=DEFAULT_GAME/'maps/map_a.bsp'
CASES={
    'construct':(DEFAULT_GAME/'maps/gm_construct.bsp',.7,None,DEFAULT_GAME/'maps/gm_construct.nav',
                 ROOT/'work/m3/gm_construct-s0.70-skip-bsp.nav','gm_construct.nav'),
    'map_b':(DEFAULT_GAME/'maps/map_b.bsp',.7,None,DEFAULT_GAME/'maps/map_b.nav',
                ROOT/'work/m3/map_b-s0.70-skip-bsp.nav','map_b.nav'),
    'map_a-bsp':(SITE,.55,None,ENGINE/'engine_map_a.nav',ENGINE/'ours_bspseeds_055.nav','map_a-bsp.nav'),
    'map_a-seeded':(SITE,.55,ROOT/'work/m2/component-seeds.txt',ENGINE/'engine_map_a.nav',
                  ROOT/'work/m3/map_a.nav','map_a.nav')}


def run(name):
    bsp,slope,seeds,reference,baseline,filename=CASES[name]
    output=ROOT/'work/m4'/filename;evidence=ROOT/'reports/measurements/m4';evidence.mkdir(parents=True,exist_ok=True)
    if seeds is None and bsp.with_suffix('.navseeds.txt').exists():raise ValueError('BSP-only acceptance requires no sidecar')
    command=[sys.executable,'-I','-B','-m','navgen',str(bsp),'--out',str(output),'--slope-limit',str(slope)]
    if seeds:command+=['--seeds',str(seeds)]
    started=time.perf_counter();result=None
    with subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace') as process:
        for line in process.stdout:
            if line.startswith('{'):result=json.loads(line)
            elif not line.startswith(('missing MDL:','missing PHY')):print(line,end='',flush=True)
        if process.wait():raise RuntimeError(f'Generation failed: {process.returncode}')
    if result is None:raise RuntimeError('Missing CLI measurement')
    result.update(command_argv=command,command_line=subprocess.list2cmdline(command),cli_seconds=time.perf_counter()-started)
    generated=read_nav(output.read_bytes());bare=copy.deepcopy(generated)
    bare['analyzed']=0
    for area in bare['areas']:area['hiding']=[]
    base=baseline.read_bytes();same=write_nav(bare)==base
    result['baseline_without_hiding']={'path':str(baseline),'sha256':hashlib.sha256(base).hexdigest(),'byte_identical':same}
    assert same,'M4 unexpectedly changed area geometry, connections or other data'
    assert generated['bsp_size']==bsp.stat().st_size and generated['analyzed']==1
    assert write_nav(read_nav(output.read_bytes()))==output.read_bytes()
    (evidence/f'{name}-generation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    comparison=compare_files(output,reference)
    (evidence/f'{name}-hiding.json').write_text(json.dumps(comparison,indent=2),encoding='utf-8')
    compact={k:comparison[k] for k in ('ours','reference','ours_match_fraction','reference_match_fraction')}
    compact['exact_extents']={k:v for k,v in comparison['exact_extents'].items() if k!='examples'}
    print(json.dumps({'case':name,'timings':result['timings_seconds'],'baseline_identical':same,**compact}),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('case',choices=[*CASES,'all']);args=parser.parse_args()
    for name in CASES if args.case=='all' else [args.case]:run(name)


if __name__=='__main__':main()

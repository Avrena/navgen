"""Measured offline runs; all generated files remain inside this project."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

from navgen.bsp import DEFAULT_GAME
from navgen.compare import compare

ROOT=Path(__file__).resolve().parents[1]
MAPS={'gm_construct':DEFAULT_GAME/'maps/gm_construct.bsp',
      'map_b':DEFAULT_GAME/'maps/map_b.bsp',
      'map_a':DEFAULT_GAME/'maps/map_a.bsp'}

parser=argparse.ArgumentParser()
parser.add_argument('name',choices=MAPS)
parser.add_argument('--compare-only',action='store_true')
parser.add_argument('--slope-limit',type=float,default=.7)
parser.add_argument('--seeds',type=Path)
parser.add_argument('--missing-phy',choices=('skip','bbox'))
parser.add_argument('--out',type=Path)
args=parser.parse_args()
missing=args.missing_phy or 'skip'
label=f'{args.name}-s{args.slope_limit:.2f}-{missing}-'+('extra' if args.seeds else 'bsp')
destination=(args.out or ROOT/'work/m3'/f'{label}.nav').resolve()
if not destination.is_relative_to((ROOT/'work/m3').resolve()):raise ValueError('Acceptance output must stay under work/m3')
evidence=ROOT/'reports/measurements/m3';evidence.mkdir(parents=True,exist_ok=True)
if not args.compare_only:
    command=[sys.executable,'-I','-B','-m','navgen',str(MAPS[args.name]),'--out',str(destination),'--slope-limit',str(args.slope_limit)]
    if args.seeds:command+=['--seeds',str(args.seeds.resolve())]
    if args.missing_phy:command+=['--missing-phy',args.missing_phy]
    started=time.perf_counter();result=None
    with subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace') as process:
        for line in process.stdout:
            if line.startswith('{'):result=json.loads(line)
            elif not line.startswith(('missing MDL:','missing PHY')):print(line,end='',flush=True)
        if process.wait():raise RuntimeError(f'Generation CLI exited {process.returncode}')
    if result is None:raise RuntimeError('Generation CLI returned no measurement')
    assert result['missing_phy_policy']==missing
    result.update(command_argv=command,command_line=subprocess.list2cmdline(command),cli_seconds=time.perf_counter()-started)
    (evidence/f'{label}-generation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('areas','links','nodes','seeds','nav_sha256','timings_seconds')},indent=2),flush=True)
reference=DEFAULT_GAME/'maps'/('map_a_old.nav' if args.name=='map_a' else f'{args.name}.nav')
result=compare(destination,reference,MAPS[args.name],missing_phy=missing,slope_limit=args.slope_limit)
(evidence/f'{label}-comparison.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({k:result[k] for k in ('coverage','ours_graph','reference_graph','directed_transitions','seconds')}|{'unverified_links':result['wall_check']['failed_links']},indent=2),flush=True)

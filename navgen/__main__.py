"""Noninteractive Hammer++ compile-step entry point."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

import numpy as np

from .areas import build_areas
from .bsp import BSP,DEFAULT_GAME
from .collision import Collision
from .hiding import compute_hiding_spots
from .navio import read_nav,write_nav
from .sampling import sample
from .runtime import configure_threads, trace_count, CACHE_FINGERPRINT


def validate_nav(nav):
    areas=nav['areas'];ids={a['id'] for a in areas}
    spot_ids=set()
    if len(ids)!=len(areas) or not ids:raise ValueError('Empty mesh or duplicate area IDs')
    for a in areas:
        values=np.r_[a['nw'],a['se'],a['ne_z'],a['sw_z']]
        if not np.isfinite(values).all() or a['se'][0]<=a['nw'][0] or a['se'][1]<=a['nw'][1]:
            raise ValueError(f'Invalid area geometry: {a["id"]}')
        for links in a['links']:
            if len(links)!=len(set(links)) or a['id'] in links or any(b not in ids for b in links):
                raise ValueError(f'Invalid connection from area {a["id"]}')
        for sid,pos,flags in a['hiding']:
            if sid in spot_ids or not np.isfinite(pos).all() or flags not in (1,8):
                raise ValueError(f'Invalid hiding spot in area {a["id"]}')
            spot_ids.add(sid)
    ladders={item['id']:item for item in nav['ladders']}
    if len(ladders)!=len(nav['ladders']):raise ValueError('Duplicate ladder IDs')
    by_id={a['id']:a for a in areas}
    for ident,item in ladders.items():
        values=np.r_[item['top'],item['bottom'],item['width'],item['length']]
        refs=item['areas']
        if (not np.isfinite(values).all() or item['width']<=0 or item['length']<=0 or
            item['top'][2]<=item['bottom'][2] or item['dir'] not in range(4) or
            len(refs)!=5 or not refs[4] or not any(refs[:3]) or any(a and a not in ids for a in refs)):
            raise ValueError(f'Invalid ladder geometry or area references: {ident}')
        if ident not in by_id[refs[4]]['ladders'][0] or any(ident not in by_id[a]['ladders'][1] for a in refs[:4] if a):
            raise ValueError(f'Nonreciprocal ladder attachment: {ident}')
    for a in areas:
        for d,refs in enumerate(a['ladders']):
            if len(refs)!=len(set(refs)) or any(i not in ladders for i in refs):raise ValueError('Invalid area ladder reference')
            for ident in refs:
                expected=ladders[ident]['areas'][4:] if d==0 else ladders[ident]['areas'][:4]
                if a['id'] not in expected:raise ValueError('Area refers to an unattached ladder')


def resolve_seed_file(map_path,seeds=None,progress=print):
    candidate=Path(seeds) if seeds is not None else Path(map_path).with_suffix('.navseeds.txt')
    if seeds is not None or candidate.exists():
        progress(f'seeds: using {candidate} ({"explicit override" if seeds is not None else "automatic sidecar"})',flush=True)
        return candidate
    progress(f'seeds: no sidecar found at {candidate}; using BSP seeds',flush=True)
    return None


def generate(map_path,game=DEFAULT_GAME,out=None,seeds=None,missing_phy='skip',progress=print,slope_limit=.7,threads=None,profile='sdk',seed_discovery=None,ladders=False):
    threads=configure_threads(threads)
    if profile not in ('sdk','merged'):raise ValueError('Profile must be sdk or merged')
    if seed_discovery is None:seed_discovery='none'
    if seed_discovery not in ('none','entities'):raise ValueError('Seed discovery must be none or entities')
    if not isinstance(ladders,bool):raise ValueError('Ladders must be a boolean')
    if not np.isfinite(slope_limit) or not 0<slope_limit<=1:raise ValueError('Slope limit must be in (0, 1]')
    total=time.perf_counter();timings={}
    start=time.perf_counter();bsp=BSP(map_path);seed_file=resolve_seed_file(bsp.path,seeds,progress)
    seed_positions=bsp.seeds(seed_file)
    seed_input=None if seed_file is None else {'path':str(seed_file.resolve()),'sha256':hashlib.sha256(seed_file.read_bytes()).hexdigest(),
                                             'source':'explicit' if seeds is not None else 'sidecar'}
    timings['input']=time.perf_counter()-start
    progress(f'input: {bsp.path.name}, {len(bsp.data):,} bytes, {len(seed_positions)} seeds',flush=True)
    start=time.perf_counter();collision=Collision.load(bsp,game,missing_phy,progress)
    timings['collision']=time.perf_counter()-start
    effective=seed_positions;discovery=None;ladder_sources=()
    if seed_discovery=='entities':
        from .discovery import discover_seeds
        start=time.perf_counter()
        effective,discovery,ladder_sources=discover_seeds(bsp,collision,seed_positions,slope_limit,progress=progress)
        timings['seed_discovery']=time.perf_counter()-start
    start=time.perf_counter();arrays,index=sample(collision,effective,[bsp.models[0][:3],bsp.models[0][3:6]],progress,slope_limit)
    timings['sampling']=time.perf_counter()-start
    graph,area_timings=build_areas(collision,arrays,index,progress,slope_limit);timings.update(area_timings)
    nav=graph.nav(len(bsp.data));nodes=len(arrays[0]);cleanup=graph.counts;link_audit=graph.link_audit
    ladder_info=None;merge_info=None
    if ladders:
        from .ladders import build_ladders
        start=time.perf_counter()
        if seed_discovery=='none':
            from .discovery import ladder_specs
            ladder_sources,_=ladder_specs(bsp)
        ladder_info=build_ladders(nav,collision,ladder_sources,support_convention=True,slope=slope_limit)
        timings['ladders']=time.perf_counter()-start
    if profile=='merged':
        from .merge import merge_nav,merge_environment
        start=time.perf_counter();water,locked,environment=merge_environment(bsp,collision,nav)
        nav,merge_info=merge_nav(nav,water=water,locked_ids=locked,scene=collision.scene,slope=slope_limit)
        merge_info['environment']=environment;timings['post_merge']=time.perf_counter()-start
        link_audit={'sdk_before_merge':link_audit,'merge':merge_info['link_validation']}
        progress(f'merged: {merge_info["source_areas"]:,} -> {len(nav["areas"]):,} areas, {timings["post_merge"]:.2f}s',flush=True)
    start=time.perf_counter()
    hiding=compute_hiding_spots(nav,collision);timings['hiding_spots']=time.perf_counter()-start
    progress(f'hiding spots: {hiding["total"]:,} ({hiding["in_cover"]:,} cover, {hiding["exposed"]:,} exposed), {timings["hiding_spots"]:.2f}s',flush=True)
    start=time.perf_counter();validate_nav(nav)
    raw=write_nav(nav)
    if write_nav(read_nav(raw))!=raw:raise ValueError('Generated NAV round-trip failed')
    target=Path(out) if out else bsp.path.with_suffix('.nav')
    if target.resolve()==bsp.path.resolve() or (seed_file is not None and target.resolve()==seed_file.resolve()):
        raise ValueError('Output must not replace an input BSP or seed file')
    target.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=target.parent,prefix=target.name+'.',suffix='.tmp',delete=False) as f:
        temp=Path(f.name);f.write(raw)
    try:os.replace(temp,target)
    finally:
        if temp.exists():temp.unlink()
    timings['write_validate']=time.perf_counter()-start;timings['total']=time.perf_counter()-total
    result={'map':str(bsp.path),'map_sha256':hashlib.sha256(bsp.data).hexdigest(),'out':str(target),
            'nav_sha256':hashlib.sha256(raw).hexdigest(),'bsp_size':len(bsp.data),'nav_bytes':len(raw),
            'areas':len(nav['areas']),'links':sum(len(ls) for a in nav['areas'] for ls in a['links']),
            'nodes':nodes,'seeds':len(seed_positions),'traces':trace_count(collision.scene),'profile':profile,'merge':merge_info,'ladders':ladder_info,
            'seed_discovery':{'mode':seed_discovery,'effective_seeds':effective.tolist(),'details':discovery},
            'threads':threads,'compiled_cache_fingerprint':CACHE_FINGERPRINT,
            'sampling_speculation':getattr(collision,'sampling_speculation',None),
            'missing_phy_policy':missing_phy,'slope_limit':slope_limit,'collision':collision.counts,'unresolved':collision.unresolved,
            'content_audit':collision.content_audit,
            'cleanup':cleanup,'link_audit':link_audit,'timings_seconds':timings,'analyzed':nav['analyzed'],
            'hiding_spots':hiding,'seed_file':seed_input,'native_acceptance':False}
    for item in collision.unresolved:progress(item,flush=True)
    if collision.content_audit:
        progress(f'content: {collision.counts["content_conflicts"]} differing source paths (details in content_audit)',flush=True)
    progress(f'completed: {result["areas"]:,} areas, {result["links"]:,} directed links, {timings["total"]:.2f}s, {target}',flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description='Generate a Source v16/0 NAV offline with hiding analysis and optional conservative merging.')
    parser.add_argument('map',type=Path);parser.add_argument('--game',type=Path,default=DEFAULT_GAME)
    parser.add_argument('--out',type=Path);parser.add_argument('--seeds',type=Path,help='Extra seeds; overrides automatic <map>.navseeds.txt pickup beside the BSP')
    parser.add_argument('--slope-limit',type=float,default=.7,help='Minimum walkable ground normal Z; engine default 0.7, optimizer workflow 0.55')
    parser.add_argument('--threads',type=int,help='Numba worker threads (default: all available logical cores)')
    parser.add_argument('--profile',choices=('sdk','merged'),default='sdk',help='sdk preserves engine generation; merged contracts compatible SDK areas')
    parser.add_argument('--seed-discovery',choices=('none','entities'),help='Opt in to validated entity-floor seeds; default none for both profiles')
    parser.add_argument('--ladders',action='store_true',help='Opt in to static BSP ladder records; default off for both profiles')
    parser.add_argument('--missing-phy',choices=('skip','bbox'),default='skip',help='SOLID_VPHYSICS without PHY: omit collision (default), or use a transformed MDL hull box')
    args=parser.parse_args()
    try:
        result=generate(args.map,args.game,args.out,args.seeds,args.missing_phy,slope_limit=args.slope_limit,threads=args.threads,profile=args.profile,seed_discovery=args.seed_discovery,ladders=args.ladders)
    except Exception as exc:
        print(f'navgen failed: {type(exc).__name__}: {exc}',file=sys.stderr,flush=True);return 1
    print(json.dumps(result,sort_keys=True),flush=True)
    return 0


if __name__=='__main__':sys.exit(main())

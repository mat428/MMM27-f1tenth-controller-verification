"""Run the archived tuning and 360-run design without altering published data."""
from __future__ import annotations
import argparse
import contextlib
import csv
import gzip
import hashlib
import importlib.metadata as md
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
COMMIT = '4fdb9c7e6fb7c701290f4dc18377d07c1681724f'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--smoke', action='store_true', help='Four evaluation runs only; not a full reproduction')
    args = parser.parse_args()
    target = ROOT/'results'/('smoke' if args.smoke else 'campaign')
    if target.exists():
        raise SystemExit(f'Refusing to overwrite {target}. Move your existing results before a new run.')
    if not args.smoke and (ROOT/'results'/'tuning').exists():
        raise SystemExit('Refusing to overwrite results/tuning. Move it before a new full run.')
    # Disables only creation of a graphics shadow window. Physics is unchanged.
    import pyglet
    pyglet.options['shadow_window'] = False
    import f110_gym
    simroot = Path(f110_gym.__file__).resolve().parents[2]
    actual = subprocess.check_output(['git','-C',str(simroot),'rev-parse','HEAD'],text=True).strip()
    if actual != COMMIT:
        raise SystemExit(f'Wrong simulator commit: {actual}')
    sys.path.insert(0,str(ROOT/'src'))
    import reference_campaign as reference
    import doe_campaign as campaign
    import pandas as pd
    import numpy as np
    versions = {}
    for name in ('numpy','scipy','pandas','Pillow','gym','numba','llvmlite','pyglet','PyYAML','cloudpickle','gym-notices'):
        versions[name] = md.version(name)
    cpu = 'unrecorded'
    if Path('/proc/cpuinfo').exists():
        cpu = next((l.split(':',1)[1].strip() for l in Path('/proc/cpuinfo').read_text().splitlines()
                    if l.startswith('model name')),cpu)
    metadata = dict(started_utc=datetime.now(timezone.utc).isoformat(),
        python=sys.version, platform=platform.platform(), cpu_model=cpu,
        logical_cpu_count=os.cpu_count(), available_cpu_count=len(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else None,
        simulator_commit=actual, dependencies=versions, smoke_only=args.smoke,
        graphics_shadow_window=False, boundary_observer='passive 100 Hz projection/counter trace; no feedback or state modification',
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'src').glob('*.py'))})
    ROOT.joinpath('results').mkdir(exist_ok=True)
    meta_path=ROOT/'results'/('smoke_execution.json' if args.smoke else 'execution.json')
    meta_path.write_text(json.dumps(metadata,indent=2)+'\n')
    start=time.monotonic()
    try:
        if not args.smoke:
            tuning_dir=ROOT/'results'/'tuning'
            tuning_dir.mkdir()
            traces=[]
            original=reference.run_one
            def record_tuning(*a,**kw):
                summary,trace=original(*a,**kw)
                traces.append(summary)
                trace.to_csv(tuning_dir/(summary['run_id'].replace(' ','_')+'.csv.gz'),index=False,compression='gzip')
                print('Tuning:',summary['run_id'],'success=',summary['success'],flush=True)
                return summary,trace
            reference.run_one=record_tuning
            try:
                chosen,scores=reference.tune_controllers(reference.make_tracks())
            finally:
                reference.run_one=original
            scores.to_csv(tuning_dir/'controller_tuning.csv',index=False)
            pd.DataFrame(traces).to_csv(tuning_dir/'run_summary.csv',index=False)
            (tuning_dir/'selected_tuning.json').write_text(json.dumps(chosen,indent=2)+'\n')
            expected=json.loads((ROOT/'frozen_tuning.json').read_text())
            if chosen != expected:
                raise RuntimeError(f'Retuned settings {chosen} differ from archived frozen settings {expected}; evaluation not started.')
            print('Tuning settings match archived settings:',chosen,flush=True)
        argv=sys.argv[:]
        try:
            sys.argv=['doe_campaign.py']+(['--limit','4'] if args.smoke else [])
            # Observe existing per-physics-step projection calls without changing
            # their return values or replacing the archived departure logic.
            target.mkdir(parents=True,exist_ok=True)
            project=campaign.nearest_state
            run=campaign.run_one
            active={}
            with gzip.open(target/'boundary_checks.csv.gz','wt',newline='') as stream:
                writer=csv.writer(stream)
                writer.writerow(['run_id','time_s','physics_step','x_m','y_m','path_index','lateral_error_m','consecutive_exceedances'])
                def observe_projection(track,x,y):
                    value=project(track,x,y)
                    active['counter']=active['counter']+1 if abs(value[1])>campaign.BOUNDARY_LIMIT else 0
                    writer.writerow([active['run_id'],active['env'].current_time,active['step'],x,y,value[0],value[1],active['counter']])
                    active['step']+=1
                    return value
                def observe_run(env,row,track,record_timeseries=True):
                    active.update(run_id=row.run_id,env=env,step=0,counter=0)
                    return run(env,row,track,record_timeseries=record_timeseries)
                campaign.nearest_state=observe_projection
                campaign.run_one=observe_run
                try:
                    campaign.main()
                finally:
                    campaign.nearest_state=project
                    campaign.run_one=run
        finally:
            sys.argv=argv
        metadata['status']='completed'
    except BaseException as exc:
        metadata.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        metadata.update(finished_utc=datetime.now(timezone.utc).isoformat(),wall_clock_seconds=time.monotonic()-start)
        meta_path.write_text(json.dumps(metadata,indent=2)+'\n')

if __name__=='__main__':
    main()

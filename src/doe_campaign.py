"""Prospectively specified F1TENTH simulation DOE. Run from the project root."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import itertools
import json
import math
import platform
import random
import sys
import time
from collections import deque
from pathlib import Path

import gym
import numpy as np
import pandas as pd
import scipy
import f110_gym

from reference_campaign import (DEFAULT_PARAMS, DT, CONTROL_DT, CONTROL_STEPS,
    BOUNDARY_LIMIT, TRACK_WIDTH, make_tracks, make_env, nearest_state,
    pure_pursuit, stanley, wrap_angle)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results'
FACTORS = ('speed_code', 'delay_code', 'noise_code', 'bias_code', 'mismatch_code')
CONTROLLERS = ('Pure Pursuit', 'Stanley')
TRACKS = ('oval', 'hairpin', 'chicane')
SEEDS = (3101, 3102, 3103)
TUNING = json.loads((ROOT / 'frozen_tuning.json').read_text())


def manifest():
    rows = []
    for track in TRACKS:
        for speed in (0.6, 1.0, 1.5, 2.0):
            for seed in SEEDS:
                for controller in CONTROLLERS:
                    rows.append(dict(phase='A_nominal', design_point=f'N{speed:g}',
                        track=track, seed=seed, controller=controller, speed_mps=speed,
                        delay_requested_s=0., noise_on=0, bias_on=0, mismatch_on=0,
                        **{f: -1 if f != 'speed_code' else (1 if speed == 2.0 else -1)
                           for f in FACTORS}))
    # Regular 2^(5-1) resolution V fraction: E = A*B*C*D, I = ABCDE.
    # All main effects are aliased with four-way effects; two-way with three-way.
    for i, (a,b,c,d) in enumerate(itertools.product((-1,1), repeat=4), 1):
        e = a*b*c*d
        for track in TRACKS:
            for seed in SEEDS:
                for controller in CONTROLLERS:
                    rows.append(dict(phase='B_fraction', design_point=f'F{i:02d}',
                        track=track, seed=seed, controller=controller,
                        speed_mps=0.6 if a == -1 else 2.0,
                        delay_requested_s=0. if b == -1 else 0.075,
                        noise_on=int(c == 1), bias_on=int(d == 1), mismatch_on=int(e == 1),
                        **dict(zip(FACTORS, (a,b,c,d,e)))))
    # Randomize paired controller order, then shuffle pairs within each track block.
    rng = random.Random(20260927)
    grouped = {}
    for row in rows:
        key = (row['phase'],row['track'],row['design_point'],row['seed'])
        grouped.setdefault(key, []).append(row)
    ordered = []
    for phase in ('A_nominal','B_fraction'):
        for track in TRACKS:
            pairs = [v for k,v in grouped.items() if k[0] == phase and k[1] == track]
            rng.shuffle(pairs)
            for pair in pairs:
                rng.shuffle(pair)
                ordered.extend(pair)
    for i,row in enumerate(ordered, 1):
        row['run_id'] = f'DOE{i:03d}'
        row['run_order'] = i
        row['pair_id'] = f"{row['phase']}:{row['track']}:{row['design_point']}:{row['seed']}"
    return pd.DataFrame(ordered)


def run_one(env, row, track, record_timeseries=True):
    # Separate streams make pose and noise identical for paired controllers,
    # irrespective of early termination and of which factors are enabled.
    seed = int(row.seed)
    pose_rng = np.random.default_rng(seed)
    noise_rng = np.random.default_rng(seed + 100000)
    params = dict(DEFAULT_PARAMS)
    if row.mismatch_on:
        params.update(mu=params['mu']*.80, m=params['m']*1.15,
                      C_Sf=params['C_Sf']*.85, C_Sr=params['C_Sr']*.85)
    env.update_params(params)
    jitter = pose_rng.normal(0,.008,2)
    yaw_jitter = pose_rng.normal(0,math.radians(.35))
    pose = np.array([[track['x'][0]+jitter[0], track['y'][0]+jitter[1],
                      track['yaw'][0]+yaw_jitter]])
    obs, _, done, _ = env.reset(pose)
    delay_steps = math.ceil(float(row.delay_requested_s)/CONTROL_DT)
    state_buffer = deque(maxlen=delay_steps+1)
    command = np.array([[0.,float(row.speed_mps)]])
    logs, compute = [], []
    boundary_count = 0
    boundary_departure = collision = completed = False
    max_time = min(100., 2.7*track['length']/max(float(row.speed_mps),.35)+8.)
    step = 0
    while env.current_time < max_time:
        x,y,yaw = (float(obs[k][0]) for k in ('poses_x','poses_y','poses_theta'))
        speed = float(obs['linear_vels_x'][0])
        lap = int(obs['lap_counts'][0])
        idx,ey,refyaw,curv = nearest_state(track,x,y)
        heading = float(wrap_angle(yaw-refyaw))
        boundary_count = boundary_count+1 if abs(ey)>BOUNDARY_LIMIT else 0
        if boundary_count >= round(.10/DT):
            boundary_departure=True
            break
        if step % CONTROL_STEPS == 0:
            mx,my,myaw = x,y,yaw
            if row.noise_on:
                mx += noise_rng.normal(0,.010)
                my += noise_rng.normal(0,.010)
                myaw += noise_rng.normal(0,math.radians(.50))
            state_buffer.append((env.current_time,mx,my,myaw,speed))
            stamp,cx,cy,cyaw,cv = state_buffer[0] if len(state_buffer)>delay_steps else state_buffer[-1]
            actual_age = float(env.current_time-stamp)
            start = time.perf_counter_ns()
            if row.controller == 'Pure Pursuit':
                steer = pure_pursuit(track,cx,cy,cyaw,cv,TUNING['pure_pursuit_lookahead_m'])
            else:
                steer = stanley(track,cx,cy,cyaw,cv,TUNING['stanley_gain'])
            compute_ms = (time.perf_counter_ns()-start)/1e6
            compute.append(compute_ms)
            if row.bias_on:
                steer += math.radians(2.)
            steer = float(np.clip(steer,DEFAULT_PARAMS['s_min'],DEFAULT_PARAMS['s_max']))
            command = np.array([[steer,float(row.speed_mps)]])
            if record_timeseries:
                logs.append((row.run_id,round(env.current_time,4),lap,x,y,yaw,speed,idx,
                             ey,heading,curv,steer,actual_age,compute_ms))
        obs,_,done,_ = env.step(command)
        step+=1
        collision = bool(obs['collisions'][0])
        if done:
            completed = int(obs['lap_counts'][0]) >= 2 and not collision
            break
    cols = ('run_id','time_s','lap_count','x_m','y_m','yaw_rad','speed_mps',
            'path_index','lateral_error_m','heading_error_rad','curvature_1pm',
            'steering_command_rad','state_age_s','compute_ms')
    ts = pd.DataFrame(logs,columns=cols)
    # The first lap is warm-up; a successful two-lap run has a complete second lap.
    steady = ts.loc[ts.lap_count>=1]
    valid = bool(completed and not boundary_departure and not collision and len(steady)>=20)
    lat = steady.lateral_error_m.to_numpy() if valid else np.array([])
    hd = steady.heading_error_rad.to_numpy() if valid else np.array([])
    safe = bool(completed and not boundary_departure and not collision)
    summary = dict(row._asdict(), completed_two_laps=bool(completed),
        boundary_departure=bool(boundary_departure), collision=bool(collision),
        safe_completion=safe, valid_accuracy=valid, duration_s=float(env.current_time),
        logged_samples=len(ts), second_lap_samples=len(steady),
        lateral_rmse_m=float(np.sqrt(np.mean(lat**2))) if valid else np.nan,
        lateral_p95_m=float(np.percentile(abs(lat),95)) if valid else np.nan,
        lateral_max_m=float(np.max(abs(lat))) if valid else np.nan,
        heading_rmse_deg=float(np.degrees(np.sqrt(np.mean(hd**2)))) if valid else np.nan,
        measured_state_age_p50_s=float(ts.state_age_s.median()) if len(ts) else np.nan,
        measured_state_age_max_s=float(ts.state_age_s.max()) if len(ts) else np.nan,
        compute_p95_ms=float(np.percentile(compute,95)) if compute else np.nan,
        stop_reason='boundary' if boundary_departure else ('collision' if collision else
                    ('completed' if completed else 'timeout_or_other')))
    return summary,ts


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--limit',type=int,default=0,help='Development smoke run only; never publish as full campaign')
    args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    schedule=manifest()
    if args.limit:
        schedule=schedule.iloc[:args.limit].copy()
        destination=OUT/'smoke'
    else:
        destination=OUT/'campaign'
    destination.mkdir(exist_ok=True)
    schedule.to_csv(destination/'scenario_manifest.csv',index=False)
    tracks=make_tracks()
    envs={k:make_env(k) for k in TRACKS}
    summary=[]
    started=time.time()
    try:
        with gzip.open(destination/'timeseries.csv.gz','wt',newline='') as stream:
            for i,row in enumerate(schedule.itertuples(index=False),1):
                result,ts=run_one(envs[row.track],row,tracks[row.track])
                summary.append(result)
                ts.to_csv(stream,index=False,header=(i==1))
                if i%24==0: print(f'{i}/{len(schedule)} runs; {time.time()-started:.0f}s',flush=True)
    finally:
        for env in envs.values(): env.close()
    pd.DataFrame(summary).to_csv(destination/'run_summary.csv',index=False)
    metadata=dict(simulator_commit='4fdb9c7e6fb7c701290f4dc18377d07c1681724f',
        python=platform.python_version(),gym=gym.__version__,numpy=np.__version__,
        scipy=scipy.__version__,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        seed_values=SEEDS,run_count=len(summary),seconds=time.time()-started,
        design='A nominal four speeds; B 2^(5-1) I=ABCDE resolution V; controller crossed; track blocked',
        frozen_tuning=TUNING,simulated_only=True)
    (destination/'metadata.json').write_text(json.dumps(metadata,indent=2))
    print(json.dumps({'runs':len(summary),'seconds':round(metadata['seconds'],1)},indent=2))


if __name__=='__main__': main()

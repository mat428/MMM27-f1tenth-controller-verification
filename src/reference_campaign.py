from __future__ import annotations

import gzip
import json
import math
import time
import warnings
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import gym
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw
from scipy.spatial import cKDTree
from scipy import stats

import f110_gym  # registers f110_gym:f110-v0
from f110_gym.envs.base_classes import Integrator


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'f1tenth_simulation_results'
MAP_DIR = OUT / 'maps'
OUT.mkdir(exist_ok=True)
MAP_DIR.mkdir(exist_ok=True)

DT = 0.01
CONTROL_DT = 0.02
CONTROL_STEPS = int(round(CONTROL_DT / DT))
WHEELBASE = 0.3302
TRACK_WIDTH = 1.30
CONSERVATIVE_LATERAL_ENVELOPE = 0.58  # use body length, not narrower body width
BOUNDARY_LIMIT = TRACK_WIDTH / 2 - CONSERVATIVE_LATERAL_ENVELOPE / 2
DEFAULT_PARAMS = {
    'mu': 1.0489, 'C_Sf': 4.718, 'C_Sr': 5.4562,
    'lf': 0.15875, 'lr': 0.17145, 'h': 0.074,
    'm': 3.74, 'I': 0.04712, 's_min': -0.4189,
    's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2,
    'v_switch': 7.319, 'a_max': 9.51, 'v_min': -5.0,
    'v_max': 20.0, 'width': 0.31, 'length': 0.58,
}


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def resample_closed(x, y, n=1200):
    pts = np.c_[x, y]
    pts = np.vstack([pts, pts[0]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    s_new = np.linspace(0, s[-1], n, endpoint=False)
    xr = np.interp(s_new, s, pts[:, 0])
    yr = np.interp(s_new, s, pts[:, 1])
    return xr, yr, s_new, float(s[-1])


def make_tracks():
    t = np.linspace(0, 2*np.pi, 1800, endpoint=False)
    # Smooth stadium: two straights joined by constant-radius 180-degree turns.
    # This supplies repeated hairpin-like corners without the curvature cusp of
    # a signed-power parameterization.
    nseg = 450
    top_x = np.linspace(-3.0, 3.0, nseg, endpoint=False)
    top_y = np.full(nseg, 1.8)
    right_a = np.linspace(np.pi/2, -np.pi/2, nseg, endpoint=False)
    right_x = 3.0 + 1.8*np.cos(right_a)
    right_y = 1.8*np.sin(right_a)
    bottom_x = np.linspace(3.0, -3.0, nseg, endpoint=False)
    bottom_y = np.full(nseg, -1.8)
    left_a = np.linspace(-np.pi/2, -3*np.pi/2, nseg, endpoint=False)
    left_x = -3.0 + 1.8*np.cos(left_a)
    left_y = 1.8*np.sin(left_a)
    raw = {
        'oval': (4.2*np.cos(t), 2.4*np.sin(t)),
        'hairpin': (np.r_[top_x, right_x, bottom_x, left_x],
                    np.r_[top_y, right_y, bottom_y, left_y]),
        'chicane': ((4.1 + 0.48*np.cos(3*t))*np.cos(t),
                    (2.55 + 0.55*np.cos(3*t + 0.4))*np.sin(t)),
        'development': ((3.8 + 0.35*np.sin(4*t))*np.cos(t),
                        (2.65 + 0.30*np.sin(3*t))*np.sin(t)),
    }
    tracks = {}
    for name, (x0, y0) in raw.items():
        x, y, s, length = resample_closed(x0, y0)
        dx = np.roll(x, -1) - np.roll(x, 1)
        dy = np.roll(y, -1) - np.roll(y, 1)
        yaw = np.arctan2(dy, dx)
        ds = length / len(x)
        curvature = wrap_angle(np.roll(yaw, -1) - np.roll(yaw, 1)) / (2*ds)
        tracks[name] = {
            'x': x, 'y': y, 's': s, 'yaw': yaw,
            'curvature': curvature, 'length': length,
            'tree': cKDTree(np.c_[x, y]),
        }
        create_map(name, x, y)
    return tracks


def create_map(name, x, y, resolution=0.04, margin=1.2):
    xmin, xmax = float(x.min()-margin), float(x.max()+margin)
    ymin, ymax = float(y.min()-margin), float(y.max()+margin)
    w = int(math.ceil((xmax-xmin)/resolution)) + 1
    h = int(math.ceil((ymax-ymin)/resolution)) + 1
    image = Image.new('L', (w, h), 0)
    draw = ImageDraw.Draw(image)
    pix = []
    for px, py in zip(x, y):
        cx = int(round((px-xmin)/resolution))
        cy = h - 1 - int(round((py-ymin)/resolution))
        pix.append((cx, cy))
    pix.append(pix[0])
    draw.line(pix, fill=255, width=max(3, int(round(TRACK_WIDTH/resolution))), joint='curve')
    image.save(MAP_DIR / f'{name}.png')
    yaml_text = (
        f'image: {name}.png\nresolution: {resolution}\n'
        f'origin: [{xmin}, {ymin}, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.196\n'
    )
    (MAP_DIR / f'{name}.yaml').write_text(yaml_text)


def nearest_state(track, x, y):
    _, idx = track['tree'].query([x, y])
    idx = int(idx)
    rx, ry = track['x'][idx], track['y'][idx]
    yaw_ref = track['yaw'][idx]
    tx, ty = math.cos(yaw_ref), math.sin(yaw_ref)
    e_y = tx*(y-ry) - ty*(x-rx)
    return idx, float(e_y), float(yaw_ref), float(track['curvature'][idx])


def pure_pursuit(track, x, y, yaw, speed, lookahead):
    idx, _, _, _ = nearest_state(track, x, y)
    advance = max(1, int(round(lookahead / (track['length']/len(track['x'])))))
    tidx = (idx + advance) % len(track['x'])
    dx = track['x'][tidx] - x
    dy = track['y'][tidx] - y
    y_local = -math.sin(yaw)*dx + math.cos(yaw)*dy
    actual_ld = max(0.05, math.hypot(dx, dy))
    steer = math.atan2(2*WHEELBASE*y_local, actual_ld**2)
    return float(np.clip(steer, DEFAULT_PARAMS['s_min'], DEFAULT_PARAMS['s_max']))


def stanley(track, x, y, yaw, speed, gain, soft=0.35):
    _, e_y, yaw_ref, _ = nearest_state(track, x, y)
    heading_error = wrap_angle(yaw_ref - yaw)
    correction = math.atan2(-gain*e_y, abs(speed)+soft)
    steer = heading_error + correction
    return float(np.clip(steer, DEFAULT_PARAMS['s_min'], DEFAULT_PARAMS['s_max']))


@dataclass
class Scenario:
    run_id: str
    group: str
    controller: str
    track: str
    speed: float
    seed: int
    delay_s: float = 0.0
    profile: str = 'nominal'
    confirmation_of: str = ''


def scenario_manifest():
    scenarios = []
    controllers = ['Pure Pursuit', 'Stanley']
    tracks = ['oval', 'hairpin', 'chicane']
    speeds = [0.6, 1.0, 1.5, 2.0]
    n = 1
    for c in controllers:
        for tr in tracks:
            for sp in speeds:
                for seed in range(3):
                    scenarios.append(Scenario(f'R{n:03d}', 'S1 nominal baseline', c, tr, sp, seed)); n += 1
    for c in controllers:
        for tr in tracks:
            for delay in [0.0, 0.025, 0.075]:
                for sp in [1.0, 2.0]:
                    for seed in [10, 11]:
                        scenarios.append(Scenario(f'R{n:03d}', 'S2 timing sensitivity', c, tr, sp, seed, delay_s=delay)); n += 1
    for c in controllers:
        for tr in tracks:
            for profile in ['nominal', 'localization_noise', 'steering_bias', 'model_mismatch']:
                for seed in [20, 21]:
                    scenarios.append(Scenario(f'R{n:03d}', 'S3 noise and mismatch', c, tr, 1.5, seed, profile=profile)); n += 1
    assert len(scenarios) == 192
    return scenarios


def make_env(track_name):
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        env = gym.make(
            'f110_gym:f110-v0', map=str(MAP_DIR / track_name), map_ext='.png',
            num_agents=1, timestep=DT, integrator=Integrator.RK4,
            disable_env_checker=True,
        ).unwrapped
    # f1tenth_gym's scan-based iTTC test projects straight-line motion. On a
    # curved corridor it can stop a car that is safely following the curve.
    # Safety in this campaign is instead evaluated with the conservative
    # geometric centerline boundary below. The longer 0.58 m body dimension is
    # used as a deliberately conservative lateral-clearance envelope.
    for agent in env.sim.agents:
        agent.ttc_thresh = -1.0
    return env


def run_one(env, scenario, track, tuning, keep_timeseries=True):
    rng = np.random.default_rng(scenario.seed)
    params = dict(DEFAULT_PARAMS)
    if scenario.profile == 'model_mismatch':
        params['mu'] *= 0.80
        params['m'] *= 1.15
        params['C_Sf'] *= 0.85
        params['C_Sr'] *= 0.85
    env.update_params(params)

    pos_jitter = rng.normal(0, 0.008, size=2)
    yaw_jitter = rng.normal(0, math.radians(0.35))
    pose = np.array([[track['x'][0]+pos_jitter[0], track['y'][0]+pos_jitter[1], track['yaw'][0]+yaw_jitter]])
    obs, _, done, _ = env.reset(pose)
    delay_steps = int(math.ceil(scenario.delay_s / CONTROL_DT))
    state_buffer = deque(maxlen=max(1, delay_steps+1))
    command = np.array([[0.0, scenario.speed]])
    records = []
    compute_times = []
    boundary_count = 0
    boundary_departure = False
    collision = False
    completed = False
    max_time = min(100.0, 2.7*track['length']/max(scenario.speed, 0.35) + 8.0)
    step = 0
    while env.current_time < max_time:
        x = float(obs['poses_x'][0]); y = float(obs['poses_y'][0]); yaw = float(obs['poses_theta'][0])
        speed_actual = float(obs['linear_vels_x'][0])
        lap_count = int(obs['lap_counts'][0])
        idx, e_y_true, yaw_ref, curvature = nearest_state(track, x, y)
        heading_error = float(wrap_angle(yaw - yaw_ref))
        if abs(e_y_true) > BOUNDARY_LIMIT:
            boundary_count += 1
        else:
            boundary_count = 0
        if boundary_count >= int(round(0.10/DT)):
            boundary_departure = True
            break
        if step % CONTROL_STEPS == 0:
            mx, my, myaw = x, y, yaw
            if scenario.profile == 'localization_noise':
                mx += rng.normal(0, 0.010)
                my += rng.normal(0, 0.010)
                myaw += rng.normal(0, math.radians(0.50))
            state_buffer.append((mx, my, myaw, speed_actual))
            control_state = state_buffer[0] if len(state_buffer) > delay_steps else state_buffer[-1]
            cx, cy, cyaw, cv = control_state
            start_ns = time.perf_counter_ns()
            if scenario.controller == 'Pure Pursuit':
                steer = pure_pursuit(track, cx, cy, cyaw, cv, tuning['pure_pursuit_lookahead_m'])
            else:
                steer = stanley(track, cx, cy, cyaw, cv, tuning['stanley_gain'])
            compute_ms = (time.perf_counter_ns()-start_ns)/1e6
            compute_times.append(compute_ms)
            if scenario.profile == 'steering_bias':
                steer += math.radians(2.0)
            steer = float(np.clip(steer, DEFAULT_PARAMS['s_min'], DEFAULT_PARAMS['s_max']))
            command = np.array([[steer, scenario.speed]])
            if keep_timeseries:
                records.append({
                    'run_id': scenario.run_id, 'time_s': round(env.current_time, 4),
                    'controller': scenario.controller, 'track': scenario.track,
                    'speed_target_mps': scenario.speed, 'profile': scenario.profile,
                    'delay_s': scenario.delay_s, 'seed': scenario.seed,
                    'lap_count': lap_count, 'x_m': x, 'y_m': y, 'yaw_rad': yaw,
                    'speed_mps': speed_actual, 'path_index': idx,
                    'lateral_error_m': e_y_true, 'heading_error_rad': heading_error,
                    'reference_curvature_1pm': curvature,
                    'steering_command_rad': float(command[0, 0]),
                    'controller_compute_ms': compute_ms,
                })
        obs, _, done, _ = env.step(command)
        step += 1
        collision = bool(obs['collisions'][0])
        if done:
            completed = int(obs['lap_counts'][0]) >= 2 and not collision
            break

    df = pd.DataFrame(records)
    if df.empty:
        df = pd.DataFrame([{'run_id': scenario.run_id, 'time_s': 0.0, 'lateral_error_m': np.nan,
                            'heading_error_rad': np.nan, 'steering_command_rad': np.nan,
                            'speed_mps': np.nan, 'controller_compute_ms': np.nan}])
    steady = df[df.get('lap_count', pd.Series(0, index=df.index)) >= 1]
    if len(steady) < 20:
        steady = df[df['time_s'] >= min(1.0, df['time_s'].max()/3)]
    lat = steady['lateral_error_m'].dropna().to_numpy()
    head = steady['heading_error_rad'].dropna().to_numpy()
    steer = steady['steering_command_rad'].dropna().to_numpy()
    speed_vals = steady['speed_mps'].dropna().to_numpy()
    ctimes = np.asarray(compute_times)
    safe = completed and not collision and not boundary_departure
    summary = {
        **scenario.__dict__,
        'simulator': 'official f1tenth_gym upstream',
        'integrator': 'RK4', 'dt_s': DT, 'control_period_s': CONTROL_DT,
        'track_length_m': track['length'], 'boundary_limit_m': BOUNDARY_LIMIT,
        'completed_two_laps': bool(completed), 'collision': bool(collision),
        'boundary_departure': bool(boundary_departure), 'success': bool(safe),
        'duration_s': float(env.current_time), 'samples_logged': int(len(df)),
        'lateral_rmse_m': float(np.sqrt(np.mean(lat**2))) if len(lat) else np.nan,
        'lateral_mae_m': float(np.mean(np.abs(lat))) if len(lat) else np.nan,
        'lateral_p95_m': float(np.percentile(np.abs(lat), 95)) if len(lat) else np.nan,
        'lateral_max_m': float(np.max(np.abs(lat))) if len(lat) else np.nan,
        'heading_rmse_deg': float(np.degrees(np.sqrt(np.mean(head**2)))) if len(head) else np.nan,
        'heading_p95_deg': float(np.degrees(np.percentile(np.abs(head), 95))) if len(head) else np.nan,
        'speed_rmse_mps': float(np.sqrt(np.mean((speed_vals-scenario.speed)**2))) if len(speed_vals) else np.nan,
        'steering_rms_rad': float(np.sqrt(np.mean(steer**2))) if len(steer) else np.nan,
        'steering_total_variation_rad': float(np.sum(np.abs(np.diff(steer)))) if len(steer)>1 else np.nan,
        'steering_saturation_fraction': float(np.mean(np.abs(steer) >= 0.99*DEFAULT_PARAMS['s_max'])) if len(steer) else np.nan,
        'compute_p50_ms': float(np.percentile(ctimes, 50)) if len(ctimes) else np.nan,
        'compute_p95_ms': float(np.percentile(ctimes, 95)) if len(ctimes) else np.nan,
        'compute_p99_ms': float(np.percentile(ctimes, 99)) if len(ctimes) else np.nan,
        'deadline_miss_fraction': float(np.mean(ctimes > CONTROL_DT*1000)) if len(ctimes) else np.nan,
        'acceptance_pass': bool(safe and len(lat) and np.sqrt(np.mean(lat**2)) <= 0.10
                                and np.percentile(np.abs(lat),95) <= 0.20
                                and np.degrees(np.sqrt(np.mean(head**2))) <= 5.0),
    }
    return summary, df


def tune_controllers(tracks):
    env = make_env('development')
    rows = []
    try:
        for controller in ['Pure Pursuit', 'Stanley']:
            values = [0.45, 0.65, 0.85, 1.05] if controller == 'Pure Pursuit' else [0.6, 1.0, 1.4, 1.8]
            for val in values:
                rmses = []; successes = []
                tuning = {'pure_pursuit_lookahead_m': val if controller == 'Pure Pursuit' else 0.65,
                          'stanley_gain': val if controller == 'Stanley' else 1.0}
                for seed in [101, 102]:
                    s = Scenario(f'TUNE_{controller[:2]}_{val}_{seed}', 'tuning', controller, 'development', 1.2, seed)
                    summary, _ = run_one(env, s, tracks['development'], tuning, keep_timeseries=True)
                    rmses.append(summary['lateral_rmse_m']); successes.append(summary['success'])
                score = np.nanmedian(rmses) + (1-np.mean(successes))*1.0
                rows.append({'controller': controller, 'parameter': 'lookahead_m' if controller=='Pure Pursuit' else 'gain',
                             'value': val, 'median_lateral_rmse_m': np.nanmedian(rmses),
                             'success_rate': np.mean(successes), 'selection_score': score})
    finally:
        env.close()
    tune_df = pd.DataFrame(rows)
    best_pp = tune_df[tune_df.controller=='Pure Pursuit'].sort_values('selection_score').iloc[0]['value']
    best_st = tune_df[tune_df.controller=='Stanley'].sort_values('selection_score').iloc[0]['value']
    return {'pure_pursuit_lookahead_m': float(best_pp), 'stanley_gain': float(best_st)}, tune_df


def paired_bootstrap(summary_df, metric, group_filter=None, n_boot=5000, seed=2026):
    d = summary_df.copy()
    if group_filter is not None:
        d = d.query(group_filter)
    key = ['group', 'track', 'speed', 'seed', 'delay_s', 'profile']
    p = d[d.controller=='Pure Pursuit'].set_index(key)[metric]
    s = d[d.controller=='Stanley'].set_index(key)[metric]
    pair = pd.concat({'Pure Pursuit':p, 'Stanley':s}, axis=1).dropna()
    diff = (pair['Pure Pursuit'] - pair['Stanley']).to_numpy()
    if len(diff) == 0:
        return {'metric':metric,'n_pairs':0,'median_diff_pp_minus_stanley':np.nan,'ci_low':np.nan,'ci_high':np.nan,'p_wilcoxon':np.nan}
    rng = np.random.default_rng(seed)
    boots = np.median(rng.choice(diff, size=(n_boot, len(diff)), replace=True), axis=1)
    try:
        pval = stats.wilcoxon(diff).pvalue
    except ValueError:
        pval = 1.0
    return {'metric':metric,'n_pairs':len(diff),'median_diff_pp_minus_stanley':float(np.median(diff)),
            'ci_low':float(np.percentile(boots,2.5)),'ci_high':float(np.percentile(boots,97.5)),
            'p_wilcoxon':float(pval)}


def validate_outputs(summary_df, ts_path):
    checks = []
    checks.append(('summary_rows_200', len(summary_df) == 200, len(summary_df)))
    checks.append(('unique_run_ids', summary_df.run_id.nunique() == 200, summary_df.run_id.nunique()))
    checks.append(('no_missing_core_fields', not summary_df[['controller','track','group','speed','seed']].isna().any().any(),
                   int(summary_df[['controller','track','group','speed','seed']].isna().sum().sum())))
    checks.append(('controller_balance', summary_df.controller.value_counts().to_dict() == {'Pure Pursuit':100,'Stanley':100},
                   summary_df.controller.value_counts().to_dict()))
    checks.append(('finite_success_metrics', np.isfinite(summary_df.loc[summary_df.success,'lateral_rmse_m']).all(),
                   int(summary_df.loc[summary_df.success,'lateral_rmse_m'].isna().sum())))
    checks.append(('metric_ranges', bool((summary_df['lateral_rmse_m'].dropna() >= 0).all() and
                                          (summary_df['deadline_miss_fraction'].dropna().between(0,1)).all()), 'checked'))
    checks.append(('timeseries_exists', ts_path.exists() and ts_path.stat().st_size > 0, ts_path.stat().st_size if ts_path.exists() else 0))
    return pd.DataFrame(checks, columns=['check','passed','observed'])


def main():
    start = time.time()
    tracks = make_tracks()
    tuning, tuning_df = tune_controllers(tracks)
    tuning_df.to_csv(OUT/'controller_tuning.csv', index=False)
    (OUT/'frozen_tuning.json').write_text(json.dumps(tuning, indent=2))
    scenarios = scenario_manifest()
    envs = {name: make_env(name) for name in ['oval','hairpin','chicane']}
    summaries = []
    timeseries_parts = []
    try:
        for i, scenario in enumerate(scenarios, 1):
            summary, ts = run_one(envs[scenario.track], scenario, tracks[scenario.track], tuning)
            summaries.append(summary); timeseries_parts.append(ts)
            if i % 16 == 0:
                print(f'completed {i}/192', flush=True)
        prelim = pd.DataFrame(summaries)
        # Choose four near-boundary matched scenarios per controller using the distance to pass/fail limits.
        prelim['boundary_score'] = np.minimum.reduce([
            np.abs(prelim['lateral_rmse_m'].fillna(9)-0.10),
            np.abs(prelim['lateral_p95_m'].fillna(9)-0.20),
            np.abs(prelim['heading_rmse_deg'].fillna(99)-5.0)/20,
        ])
        selected = prelim.sort_values('boundary_score').groupby('controller', group_keys=False).head(4)
        n = 193
        for j, row in selected.iterrows():
            scenario = Scenario(f'R{n:03d}', 'S4 boundary confirmation', row.controller, row.track,
                                float(row.speed), int(row.seed)+1000, float(row.delay_s), row.profile,
                                confirmation_of=row.run_id)
            summary, ts = run_one(envs[scenario.track], scenario, tracks[scenario.track], tuning)
            summaries.append(summary); timeseries_parts.append(ts); n += 1
    finally:
        for env in envs.values(): env.close()

    summary_df = pd.DataFrame(summaries).drop(columns=['boundary_score'], errors='ignore')
    timeseries_df = pd.concat(timeseries_parts, ignore_index=True)
    summary_path = OUT/'simulation_run_summary.csv'
    manifest_path = OUT/'scenario_manifest.csv'
    ts_path = OUT/'simulation_timeseries.csv.gz'
    summary_df.to_csv(summary_path, index=False)
    summary_df[[c for c in Scenario.__dataclass_fields__]].to_csv(manifest_path, index=False)
    timeseries_df.to_csv(ts_path, index=False, compression='gzip')

    metrics = ['lateral_rmse_m','lateral_p95_m','heading_rmse_deg','steering_total_variation_rad',
               'compute_p99_ms','speed_rmse_mps']
    analysis_rows = [paired_bootstrap(summary_df, m) for m in metrics]
    analysis_df = pd.DataFrame(analysis_rows)
    analysis_df.to_csv(OUT/'paired_statistical_analysis.csv', index=False)

    aggregate = summary_df.groupby(['controller','group'], as_index=False).agg(
        runs=('run_id','count'), success_rate=('success','mean'), acceptance_rate=('acceptance_pass','mean'),
        median_lateral_rmse_m=('lateral_rmse_m','median'), median_lateral_p95_m=('lateral_p95_m','median'),
        median_heading_rmse_deg=('heading_rmse_deg','median'), median_compute_p99_ms=('compute_p99_ms','median'),
        median_steering_total_variation_rad=('steering_total_variation_rad','median'))
    aggregate.to_csv(OUT/'aggregate_results.csv', index=False)

    checks = validate_outputs(summary_df, ts_path)
    checks.to_csv(OUT/'data_quality_checks.csv', index=False)
    metadata = {
        'campaign_generated_utc': pd.Timestamp.utcnow().isoformat(),
        'simulator_source': 'https://github.com/f1tenth/f1tenth_gym',
        'simulator_commit': '4fdb9c7e6fb7c701290f4dc18377d07c1681724f',
        'runs': int(len(summary_df)), 'timeseries_rows': int(len(timeseries_df)),
        'tuning': tuning, 'elapsed_wall_s': time.time()-start,
        'track_width_m': TRACK_WIDTH, 'boundary_limit_m': BOUNDARY_LIMIT,
        'conservative_lateral_envelope_m': CONSERVATIVE_LATERAL_ENVELOPE,
        'limitations': [
            'Simulation-only evidence; no physical F1TENTH or sim-to-real validation.',
            'Synthetic tracks and controller implementations created for this campaign.',
            'Localization noise and timing delay are injected at the controller input.',
            'Computation latency is machine-dependent and does not represent Jetson timing.',
            'The simulator scan-based straight-line iTTC stop is disabled; collision/failure is evaluated using a conservative geometric track-boundary test.',
        ],
    }
    (OUT/'campaign_metadata.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps({'runs':len(summary_df),'rows':len(timeseries_df),'tuning':tuning,
                      'success':summary_df.groupby('controller').success.mean().to_dict(),
                      'elapsed_s':metadata['elapsed_wall_s']}, indent=2), flush=True)


if __name__ == '__main__':
    main()

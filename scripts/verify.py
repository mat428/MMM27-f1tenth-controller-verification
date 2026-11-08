"""Independently audit saved observations and compare a fresh reproduction."""
from __future__ import annotations
import argparse
import ast
import hashlib
import itertools
import json
from pathlib import Path
import sys
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
FACTORS=['speed_code','delay_code','noise_code','bias_code','mismatch_code']
ATOL=1e-8

def accepted(s):
    return s.safe_completion & s.valid_accuracy & (s.lateral_rmse_m<=.1) & (s.lateral_p95_m<=.2) & (s.heading_rmse_deg<=5)

def geometry():
    source=ast.parse((ROOT/'src'/'reference_campaign.py').read_text())
    nodes=[n for n in source.body if isinstance(n,ast.FunctionDef) and n.name in {'wrap_angle','resample_closed','make_tracks'}]
    scope=dict(np=np,cKDTree=cKDTree,create_map=lambda *a:None)
    exec(compile(ast.Module(body=nodes,type_ignores=[]),'archived_geometry','exec'),scope)
    return scope['make_tracks']()

def audit(path):
    s=pd.read_csv(path/'run_summary.csv')
    m=pd.read_csv(path/'scenario_manifest.csv')
    t=pd.read_csv(path/'timeseries.csv.gz')
    meta=json.loads((path/'metadata.json').read_text())
    checks={}
    def check(name,condition,detail=None):
        checks[name]={'passed':bool(condition),'detail':detail}
    check('360_unique_runs',len(s)==360 and s.run_id.nunique()==360)
    check('manifest_matches',all(s[c].equals(m[c]) for c in m))
    check('phase_counts',s.phase.value_counts().to_dict()=={'A_nominal':72,'B_fraction':288})
    check('matched_controller_pairs',s.groupby('pair_id').controller.agg(set).map(lambda x:x=={'Pure Pursuit','Stanley'}).all())
    check('assigned_seeds',set(s.seed)=={3101,3102,3103})
    b=s[s.phase=='B_fraction']
    mat=b.drop_duplicates('design_point')[FACTORS].to_numpy()
    check('resolution_V_generator',mat.shape==(16,5) and np.all(mat.prod(axis=1)==1))
    check('main_two_factor_balance',all(np.prod(mat[:,ix],axis=1).sum()==0
        for n in (1,2) for ix in itertools.combinations(range(5),n)))
    check('log_run_ids',set(t.run_id)==set(s.run_id))
    check('sample_counts',(t.groupby('run_id').size().reindex(s.run_id).to_numpy()==s.logged_samples.to_numpy()).all())
    check('recorded_source_hash',hashlib.sha256((ROOT/'src'/'doe_campaign.py').read_bytes()).hexdigest()==meta['code_sha256'])
    check('simulator_commit',meta['simulator_commit']=='4fdb9c7e6fb7c701290f4dc18377d07c1681724f')
    check('frozen_tuning',meta['frozen_tuning']==json.loads((ROOT/'frozen_tuning.json').read_text()))
    tracks=geometry()
    maximum_projection=maximum_heading=0.
    indices=True
    for name,rows in s.groupby('track'):
        q=t[t.run_id.isin(rows.run_id)]
        track=tracks[name]
        _,idx=track['tree'].query(q[['x_m','y_m']].to_numpy())
        angle=track['yaw'][idx]
        error=np.cos(angle)*(q.y_m.to_numpy()-track['y'][idx])-np.sin(angle)*(q.x_m.to_numpy()-track['x'][idx])
        heading=(q.yaw_rad.to_numpy()-angle+np.pi)%(2*np.pi)-np.pi
        maximum_projection=max(maximum_projection,float(np.max(np.abs(error-q.lateral_error_m.to_numpy()))))
        maximum_heading=max(maximum_heading,float(np.max(np.abs(heading-q.heading_error_rad.to_numpy()))))
        indices=indices and bool((idx==q.path_index.to_numpy()).all())
    check('projection_from_true_xy',maximum_projection<1e-10,maximum_projection)
    check('heading_from_true_yaw',maximum_heading<1e-10,maximum_heading)
    check('nearest_waypoint_indices',indices)
    valid=s[s.valid_accuracy].set_index('run_id')
    r=t[t.run_id.isin(valid.index)&(t.lap_count>=1)]
    calc=r.groupby('run_id').agg(
        lateral_rmse_m=('lateral_error_m',lambda x:np.sqrt(np.mean(x.to_numpy()**2))),
        lateral_p95_m=('lateral_error_m',lambda x:np.percentile(abs(x.to_numpy()),95)),
        lateral_max_m=('lateral_error_m',lambda x:np.max(abs(x.to_numpy()))),
        heading_rmse_deg=('heading_error_rad',lambda x:np.degrees(np.sqrt(np.mean(x.to_numpy()**2)))))
    delta=float(abs(valid[calc.columns]-calc).max().max())
    check('accuracy_recalculated',delta<1e-10,delta)
    check('failed_accuracy_missing',s.loc[~s.valid_accuracy,['lateral_rmse_m','lateral_p95_m','heading_rmse_deg']].isna().all().all())
    check('completion_semantics',(s.safe_completion==(s.completed_two_laps & ~s.boundary_departure & ~s.collision)).all())
    ages=True
    sample_groups=t.groupby('run_id',sort=False)
    for row in s.itertuples(index=False):
        q=sample_groups.get_group(row.run_id)
        expected=np.zeros(len(q))
        if row.delay_requested_s:
            expected[4:]=.08
        ages=ages and bool(np.allclose(q.state_age_s.to_numpy(),expected,rtol=0,atol=1e-8))
    check('buffer_startup_and_steady_age',ages)
    if (path/'boundary_checks.csv.gz').exists():
        trace=pd.read_csv(path/'boundary_checks.csv.gz')
        trace_groups=trace.groupby('run_id',sort=False)
        boundary_valid=True
        projection_valid=True
        for row in s.itertuples(index=False):
            q=trace_groups.get_group(row.run_id)
            count=0;maximum=0
            steps=q.physics_step.to_numpy()
            if not np.array_equal(steps,np.arange(len(q))):boundary_valid=False
            tr=tracks[row.track]
            _,idx=tr['tree'].query(q[['x_m','y_m']].to_numpy())
            angle=tr['yaw'][idx]
            ey=np.cos(angle)*(q.y_m.to_numpy()-tr['y'][idx])-np.sin(angle)*(q.x_m.to_numpy()-tr['x'][idx])
            projection_valid=projection_valid and bool(np.allclose(ey,q.lateral_error_m,rtol=0,atol=1e-10))
            for error,recorded in zip(ey,q.consecutive_exceedances):
                count=count+1 if abs(error)>.36 else 0
                maximum=max(maximum,count)
                if count!=recorded:boundary_valid=False
            if bool(maximum>=10)!=bool(row.boundary_departure):boundary_valid=False
        check('100Hz_projection_from_true_xy',projection_valid)
        check('100Hz_departure_counter',boundary_valid)
    else:
        checks['100Hz_departure_counter']={'passed':None,'detail':'Not independently reconstructible: original archive contains only 50 Hz logs.'}
    s=s.copy();s['accepted']=accepted(s)
    nominal=s[s.phase=='A_nominal']
    pairs=nominal.pivot(index='pair_id',columns='controller',values='lateral_rmse_m')
    diff=pairs['Pure Pursuit']-pairs['Stanley']
    counts=s.groupby(['phase','controller']).agg(runs=('run_id','size'),completed=('safe_completion','sum'),accepted=('accepted','sum')).reset_index()
    summary=dict(raw_rows=len(t),nominal_paired_mean_difference_m=float(diff.mean()),
        nominal_paired_median_difference_m=float(diff.median()),nominal_pairs_pp_lower=int((diff<0).sum()),
        counts=counts.to_dict('records'))
    return {'checks':checks,'summary':summary,'passed':all(v['passed'] is not False for v in checks.values())},s,t

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--published-only',action='store_true',help='Audit published data without a simulator or fresh run')
    args=parser.parse_args()
    out=ROOT/'verification';out.mkdir(exist_ok=True)
    old,old_s,old_t=audit(ROOT/'data'/'published')
    reports={'published':old}
    ok=old['passed']
    if not args.published_only:
        new,new_s,new_t=audit(ROOT/'results'/'campaign');reports['fresh']=new
        comparisons={}
        comparisons['manifest_identical']=pd.read_csv(ROOT/'data'/'published'/'scenario_manifest.csv').equals(pd.read_csv(ROOT/'results'/'campaign'/'scenario_manifest.csv'))
        a=old_s.set_index('run_id').sort_index();b=new_s.set_index('run_id').sort_index()
        cols=[c for c in a if c in b and c not in {'compute_p95_ms'}]
        differences=[]
        for col in cols:
            x,y=a[col],b[col]
            if pd.api.types.is_numeric_dtype(x) and x.dtype!=bool:
                delta=float((x-y).abs().max()) if (x.notna()|y.notna()).any() else 0.
                equal=bool(np.allclose(x,y,rtol=0,atol=ATOL,equal_nan=True))
            else:delta=None;equal=x.equals(y)
            differences.append(dict(field=col,passed=equal,max_absolute_difference=delta))
        pd.DataFrame(differences).to_csv(out/'run_summary_comparison.csv',index=False)
        comparisons['all_nontiming_summary_fields_match']=all(d['passed'] for d in differences)
        raw=[]
        for col in old_t.columns:
            if col=='compute_ms':continue
            if len(old_t)!=len(new_t):equal=False;delta=None
            elif pd.api.types.is_numeric_dtype(old_t[col]):
                delta=float((old_t[col]-new_t[col]).abs().max())
                equal=bool(np.allclose(old_t[col],new_t[col],rtol=0,atol=ATOL,equal_nan=True))
            else:delta=None;equal=old_t[col].equals(new_t[col])
            raw.append(dict(field=col,passed=equal,max_absolute_difference=delta))
        pd.DataFrame(raw).to_csv(out/'timeseries_comparison.csv',index=False)
        comparisons['all_nontiming_raw_fields_match']=all(d['passed'] for d in raw)
        scores=pd.read_csv(ROOT/'results'/'tuning'/'controller_tuning.csv')
        previous=pd.read_csv(ROOT/'provenance'/'historical_tuning'/'controller_tuning.csv')
        numeric=['value','median_lateral_rmse_m','success_rate','selection_score']
        comparisons['tuning_scores_match']=bool(np.allclose(scores[numeric],previous[numeric],rtol=0,atol=ATOL))
        tuning_runs=pd.read_csv(ROOT/'results'/'tuning'/'run_summary.csv')
        tuning_valid=len(tuning_runs)==16 and bool(tuning_runs.success.all())
        tuning_max=0.
        for row in tuning_runs.itertuples(index=False):
            trace=pd.read_csv(ROOT/'results'/'tuning'/(row.run_id.replace(' ','_')+'.csv.gz'))
            steady=trace[trace.lap_count>=1]
            if len(steady)<20:
                steady=trace[trace.time_s>=min(1.,trace.time_s.max()/3)]
            rmse=float(np.sqrt(np.mean(steady.lateral_error_m.to_numpy()**2)))
            tuning_max=max(tuning_max,abs(rmse-row.lateral_rmse_m))
            tuning_valid=tuning_valid and bool(np.isfinite(rmse))
        comparisons['fresh_tuning_traces_recalculate']=bool(tuning_valid and tuning_max<1e-10)
        reports['comparison']={'checks':comparisons,'absolute_tolerance':ATOL,'relative_tolerance':0,
            'excluded':'CPU computation time, wall-clock execution time, gzip container bytes',
            'tuning_max_absolute_score_difference_m':float((scores.selection_score-previous.selection_score).abs().max()),
            'tuning_raw_metric_recalculation_max_difference_m':tuning_max}
        ok=ok and new['passed'] and all(comparisons.values())
        pd.DataFrame(new['summary']['counts']).to_csv(out/'fresh_counts.csv',index=False)
        export_tables(new_s,out)
    reports['passed']=bool(ok)
    (out/'verification_report.json').write_text(json.dumps(reports,indent=2)+'\n')
    print(json.dumps(reports,indent=2))
    if not ok:raise SystemExit('Verification failed: inspect verification_report.json; do not claim exact reproduction.')

def export_tables(s,out):
    nominal=s[s.phase=='A_nominal']
    nominal.groupby(['speed_mps','controller']).lateral_rmse_m.agg(median='median',q1=lambda x:x.quantile(.25),q3=lambda x:x.quantile(.75)).to_csv(out/'nominal_quartiles.csv')
    stress=s[s.phase=='B_fraction'].copy()
    stress.groupby(['speed_mps','delay_requested_s','controller']).agg(runs=('run_id','size'),completed=('safe_completion','sum'),accepted=('accepted','sum'),conditional_median_rmse_m=('lateral_rmse_m','median')).to_csv(out/'stress_cells.csv')
    rows=[]
    for controller,d in stress.groupby('controller'):
        # The PP continuous screen intentionally excludes the entire high-speed aged cell,
        # not just its failures: 108 assigned runs in all-completion cells remain.
        continuous=d if controller=='Stanley' else d[~((d.speed_code==1)&(d.delay_code==1))]
        for order in (1,2):
            for factors in itertools.combinations(FACTORS,order):
                for response,frame in [('lateral_rmse_m',continuous),('safe_completion',d),('accepted',d)]:
                    sign=frame[list(factors)].prod(axis=1)
                    hi=frame.loc[sign==1,response];lo=frame.loc[sign==-1,response]
                    shift=float(hi.mean()-lo.mean())
                    rows.append(dict(controller=controller,term='*'.join(factors),response=response,n_high=len(hi),n_low=len(lo),
                        high_minus_low=shift if response=='lateral_rmse_m' else 100*shift,
                        units='m' if response=='lateral_rmse_m' else 'percentage_points'))
    pd.DataFrame(rows).to_csv(out/'restricted_and_binary_screens.csv',index=False)
    high=stress[(stress.speed_mps==2.)&(stress.delay_requested_s>0)]
    high.groupby(['controller','noise_on','bias_on','mismatch_on']).agg(
        runs=('run_id','size'),completed=('safe_completion','sum'),accepted=('accepted','sum'),
        conditional_median_rmse_m=('lateral_rmse_m','median')).to_csv(out/'high_speed_aged_combinations.csv')
    threshold=[]
    for (phase,controller),frame in s.groupby(['phase','controller']):
        complete=frame[frame.valid_accuracy]
        threshold.append(dict(phase=phase,controller=controller,assigned_runs=len(frame),
            completions=int(frame.safe_completion.sum()),valid_accuracy_runs=len(complete),
            exceeded_lateral_rmse=int((complete.lateral_rmse_m>.1).sum()),
            exceeded_lateral_p95=int((complete.lateral_p95_m>.2).sum()),
            exceeded_heading_rmse=int((complete.heading_rmse_deg>5).sum()),
            note='Overlapping threshold counts; incomplete runs have no accuracy.'))
    pd.DataFrame(threshold).to_csv(out/'threshold_failures.csv',index=False)

if __name__=='__main__':main()

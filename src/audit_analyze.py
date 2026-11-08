"""Independently audit raw logs and prepare transparent descriptive DOE outputs."""
from pathlib import Path
import json
import math
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
DATA=ROOT/'results'/'campaign'
FACTORS=('speed_code','delay_code','noise_code','bias_code','mismatch_code')

def main():
    schedule=pd.read_csv(DATA/'scenario_manifest.csv')
    s=pd.read_csv(DATA/'run_summary.csv')
    t=pd.read_csv(DATA/'timeseries.csv.gz')
    checks={}
    def check(name,condition,detail):
        checks[name]={'passed':bool(condition),'detail':str(detail)}
    check('counts',len(schedule)==360 and len(s)==360 and s.run_id.nunique()==360,
          (len(schedule),len(s),s.run_id.nunique()))
    check('schedule_matches',s.run_id.equals(schedule.run_id) and
          s[list(FACTORS)].equals(schedule[list(FACTORS)]),'IDs and factor codes')
    check('two_controllers_per_pair',(s.groupby('pair_id').controller.nunique()==2).all(),
          s.groupby('pair_id').controller.nunique().value_counts().to_dict())
    check('phase_counts',s.phase.value_counts().to_dict()=={'A_nominal':72,'B_fraction':288},
          s.phase.value_counts().to_dict())
    b=s.query("phase=='B_fraction'")
    matrix=b.drop_duplicates('design_point')[list(FACTORS)].to_numpy()
    check('fraction_generator',len(matrix)==16 and np.all(np.prod(matrix,axis=1)==1),
          'I=ABCDE; 16 distinct points')
    check('main_and_two_factor_balance',all(np.sum(np.prod(matrix[:,ix],axis=1))==0
          for n in (1,2) for ix in __import__('itertools').combinations(range(5),n)),
          'all first- and second-order columns balanced')
    check('all_log_ids_match',set(t.run_id)==set(s.run_id),len(t))
    check('per_run_sample_counts',t.groupby('run_id').size().reindex(s.run_id).to_numpy().tolist()
          ==s.logged_samples.tolist(),'raw log rows versus summary')
    check('state_age_levels',bool((t.state_age_s>=-1e-9).all() and
          (t.state_age_s<=.0800001).all() and
          t.merge(s[['run_id','delay_requested_s']],on='run_id')
           .query('delay_requested_s==0').state_age_s.max()<1e-8),
          (float(t.state_age_s.min()),float(t.state_age_s.max())))
    successful=s[s.valid_accuracy]
    raw=t[t.run_id.isin(successful.run_id) & (t.lap_count>=1)]
    recalculated=raw.groupby('run_id').agg(
        rmse=('lateral_error_m',lambda x:float(np.sqrt(np.mean(x.to_numpy()**2)))),
        p95=('lateral_error_m',lambda x:float(np.percentile(abs(x.to_numpy()),95))),
        heading=('heading_error_rad',lambda x:float(np.degrees(np.sqrt(np.mean(x.to_numpy()**2))))))
    v=successful.set_index('run_id').join(recalculated)
    maxdiff=max(float(np.max(abs(v[a]-v[c]))) for a,c in
        [('lateral_rmse_m','rmse'),('lateral_p95_m','p95'),('heading_rmse_deg','heading')])
    check('independent_metric_recalculation',maxdiff<1e-10,maxdiff)
    check('failure_accuracy_missing',s.loc[~s.valid_accuracy,'lateral_rmse_m'].isna().all(),
          int((~s.valid_accuracy).sum()))
    check('no_invalid_numerics',np.isfinite(s.loc[s.valid_accuracy,'lateral_rmse_m']).all(),
          int(s.valid_accuracy.sum()))
    # A paired comparison is valid only where both controllers finish two laps.
    nominal=s.query("phase=='A_nominal'")
    p=nominal.pivot(index='pair_id',columns='controller',values='lateral_rmse_m').dropna()
    diff=p['Pure Pursuit']-p['Stanley']
    bycell=nominal.groupby(['track','speed_mps','controller'],as_index=False).agg(
        runs=('run_id','size'),safe=('safe_completion','sum'),
        complete_rmse_mean_m=('lateral_rmse_m','mean'))
    bycell.to_csv(DATA/'nominal_cells.csv',index=False)
    # Unstandardized factorial effects calculated on available complete cases.
    # This table is descriptive; outcome-dependent missingness can disrupt orthogonality.
    rows=[]
    from itertools import combinations
    for controller,d in b.groupby('controller'):
        d=d[d.valid_accuracy]
        for n in (1,2):
            for fs in combinations(FACTORS,n):
                sign=d[list(fs)].prod(axis=1)
                hi=d.loc[sign==1,'lateral_rmse_m']; lo=d.loc[sign==-1,'lateral_rmse_m']
                rows.append(dict(controller=controller,term='*'.join(fs),n_high=len(hi),n_low=len(lo),
                    high_minus_low_rmse_m=hi.mean()-lo.mean()))
    pd.DataFrame(rows).to_csv(DATA/'factor_effects_descriptive.csv',index=False)
    # Explicit binary and missing response columns for Minitab GLM / binary regression.
    worksheet=s.copy()
    worksheet['safe_completion_01']=worksheet.safe_completion.astype(int)
    worksheet['accuracy_available_01']=worksheet.valid_accuracy.astype(int)
    worksheet.to_csv(DATA/'minitab_worksheet.csv',index=False)
    report=dict(audit=checks,summary={
        'runs':len(s),'raw_rows':len(t),'safe_by_phase_controller':
          s.groupby(['phase','controller']).safe_completion.agg(['sum','count']).reset_index().to_dict('records'),
        'nominal_complete_pairs':len(diff),'nominal_paired_mean_pp_minus_stanley_m':float(diff.mean()),
        'nominal_paired_median_pp_minus_stanley_m':float(diff.median()),
        'note':'Accuracy comparisons condition on both completing; safety is reported for all runs. No general-population p-values are asserted.'})
    (DATA/'audit_report.json').write_text(json.dumps(report,indent=2))
    if not all(c['passed'] for c in checks.values()):
        raise RuntimeError('Audit failed: '+str([k for k,v in checks.items() if not v['passed']]))
    print(json.dumps(report['summary'],indent=2))

if __name__=='__main__': main()

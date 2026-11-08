"""Compare four installation-smoke runs with their archived counterparts."""
import json
from pathlib import Path
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]

def main():
    s=pd.read_csv(ROOT/'results/smoke/run_summary.csv')
    t=pd.read_csv(ROOT/'results/smoke/timeseries.csv.gz')
    archived=pd.read_csv(ROOT/'data/published/timeseries.csv.gz')
    old=archived[archived.run_id.isin(s.run_id)].reset_index(drop=True)
    checks={};deltas={}
    checks['four_runs_only']=len(s)==4 and s.run_id.nunique()==4
    for c in t:
        if c=='compute_ms':continue
        if pd.api.types.is_numeric_dtype(t[c]):
            checks[c]=bool(len(t)==len(old) and np.allclose(t[c],old[c],rtol=0,atol=1e-8,equal_nan=True))
            deltas[c]=float(abs(t[c]-old[c]).max()) if len(t)==len(old) else None
        else:checks[c]=t[c].equals(old[c])
    result=dict(passed=all(checks.values()),checks=checks,max_absolute_differences=deltas,
                scope='Four installation smoke runs only, not a 360-run reproduction.')
    path=ROOT/'verification/setup_smoke_report.json'
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit('Smoke comparison failed')

if __name__=='__main__':main()

"""Verify file hashes. --write creates a release manifest; it does not prove validity."""
import argparse
import hashlib
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SKIP={'.venv','vendor','.git','__pycache__','f1tenth_simulation_results'}

def files():
    for p in sorted(ROOT.rglob('*')):
        rel=p.relative_to(ROOT)
        if not p.is_file() or any(part in SKIP for part in rel.parts):continue
        if p.name=='SHA256SUMS.txt' or rel.as_posix().startswith('results/smoke'):continue
        yield p,rel.as_posix()

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write',action='store_true')
    args=parser.parse_args()
    target=ROOT/'SHA256SUMS.txt'
    if args.write:
        entries=[f'{digest(p)}  {name}' for p,name in files()]
        target.write_text('\n'.join(entries)+'\n')
        print('Wrote',len(entries),'file hashes. Review this new manifest before publishing.')
    else:
        failed=[];n=0
        for line in target.read_text().splitlines():
            expected,name=line.split('  ',1);p=ROOT/name;n+=1
            if not p.is_file() or digest(p)!=expected:failed.append(name)
        if failed:raise SystemExit('Hash mismatch or missing: '+', '.join(failed))
        print(n,'file hashes verified. Content identity is not evidence of scientific validity.')

if __name__=='__main__':main()

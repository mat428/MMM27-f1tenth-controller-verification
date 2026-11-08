"""Create a Python 3.12 environment and install the pinned simulator source."""
import pathlib
import subprocess
import sys
import venv

ROOT=pathlib.Path(__file__).resolve().parents[1]
COMMIT='4fdb9c7e6fb7c701290f4dc18377d07c1681724f'

def run(*args):
    print('+',' '.join(map(str,args)),flush=True)
    subprocess.run(list(map(str,args)),check=True,cwd=ROOT)

def main():
    if sys.version_info[:2]!=(3,12):
        raise SystemExit('Use Python 3.12 (tested: 3.12.14).')
    env=ROOT/'.venv'
    if not env.exists():venv.EnvBuilder(with_pip=True).create(env)
    python=env/('Scripts/python.exe' if sys.platform=='win32' else 'bin/python')
    run(python,'-m','pip','install','pip==25.0.1','setuptools==84.2.1','wheel==0.48.0','packaging==26.3')
    run(python,'-m','pip','install','-r',ROOT/'requirements.txt')
    source=ROOT/'vendor'/'f1tenth_gym'
    if source.exists():
        actual=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
        if actual!=COMMIT:
            raise SystemExit('Existing vendor/f1tenth_gym is not at the pinned commit; inspect it manually. No files were reset.')
        if subprocess.check_output(['git','-C',str(source),'status','--porcelain'],text=True).strip():
            raise SystemExit('Existing simulator source has local changes; inspect it manually.')
    else:
        source.parent.mkdir(exist_ok=True)
        run('git','clone','https://github.com/f1tenth/f1tenth_gym.git',source)
        run('git','-C',source,'checkout','--detach',COMMIT)
    # Intentional: legacy setup.py pins Gym 0.19 / NumPy <=1.22, which do not
    # describe the archived Python 3.12 experiment. Use requirements.txt instead.
    run(python,'-m','pip','install','--no-deps','--no-build-isolation','-e',source)
    run(python,'-c',"import pyglet; pyglet.options['shadow_window']=False; import numba; import f110_gym; from f110_gym.envs.base_classes import Integrator; print('Simulator imports passed')")
    print('Ready. Run .venv/bin/python scripts/reproduce.py, then scripts/verify.py.')

if __name__=='__main__':main()

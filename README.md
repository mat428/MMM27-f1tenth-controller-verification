# F1TENTH path-tracking verification: code, data and reproduction

Companion materials for **Simulation-Based Verification of F1TENTH Path
Tracking: A 360-Run Comparison of Pure Pursuit and Stanley Control**.

This is a simulation-only experiment, not a physical-robot trial. The two
controllers use the simulator pose origin, not their canonical axle reference
points. Completion uses a geometric departure proxy, not a physical safety
certificate. The project context is SafeAV Use Case 2; the work acknowledges
SafeAV, SekMo/DT-IVV and Astra+ as stated in the accompanying manuscript.

**Fresh reproduction: PASS.** All 16 tuning trials and 360 evaluation runs
were executed on 8 November 2026. Non-timing raw observations and run summaries
matched the archive with zero observed numerical difference. The fresh
100 Hz boundary audit and four offline unit tests passed. See the
[full execution and verification report](docs/REPRODUCTION_REPORT.md),
including the limits of the clean-environment installation test.

## What is supplied

| Location | Contents |
|---|---|
| `src/` | Byte-preserved archived controller, geometry, campaign and original audit code |
| `data/published/` | Original 360-run manifest, summaries and 50 Hz raw observations, unchanged |
| `results/tuning/` | Fresh 16-run development-track tuning, including newly recorded per-seed trajectories |
| `results/campaign/` | Fresh 360-run evaluation and passive 100 Hz boundary-check trace |
| `results/execution.json` | Fresh execution status, dates, software versions and host/CPU provenance |
| `verification/` | Independent audit, published-versus-fresh comparisons and paper-aligned tables |
| `provenance/` | Historical tuning aggregates and upstream licence notice |
| `scripts/` | Installation, reproduction, verification and file-integrity commands |
| `tests/` | Fast offline geometry, steering-sign, preview-index and design tests |
| `docs/` | Data dictionary, protocol and reproduction report |

The original observations and fresh observations are deliberately separated.
They reuse the same scenarios and seeds; **do not pool them as independent
replicates**. Historical tuning did not retain per-seed traces; the fresh
tuning does. Smoke-test runs are excluded from the experiment and release data.

## Quick verification, without running the simulator

After extracting or cloning this repository:

```bash
python3.12 scripts/check_integrity.py
```

This checks release file identity only. Scientific checks need NumPy, pandas
and SciPy. In an environment with the pinned `requirements.txt` packages:

```bash
python3.12 -m unittest discover -s tests -v
python3.12 scripts/verify.py --published-only
python3.12 scripts/verify.py
```

The first verification audits the original observations from their raw poses.
The second additionally audits the supplied fresh observations and compares
them with the original data. See `verification/verification_report.json` for
individual checks. No simulator is imported by these offline checks. Running
verification regenerates its report and CSVs; the original data are not edited.

## Reproduce from the beginning

### 1. Install the tested environment

Tested on Linux x86-64 with Python 3.12.14. On Windows, use an Ubuntu WSL2
environment for these Linux commands; native Windows execution was not tested.
Python 3.12, Git, internet access,
and system OpenGL/GLU libraries are needed; no GPU, desktop window, ROS 2,
Jetson or physical car is used. On Ubuntu/Debian, an administrator can install
the prerequisites with:

```bash
sudo apt-get update
sudo apt-get install git python3.12-venv libgl1 libglu1-mesa
```

Then run:

```bash
python3.12 scripts/bootstrap.py
```

The bootstrap creates `.venv/`, installs the pinned numerical dependencies,
and obtains upstream F1TENTH Gym at commit
`4fdb9c7e6fb7c701290f4dc18377d07c1681724f` under `vendor/`.
Those local dependencies are excluded from the release and from Git tracking.
The simulator's legacy setup metadata pins Gym 0.19 and old NumPy; these are
intentionally overridden by the tested pins, using `--no-deps` for the
simulator installation. Do not resolve the environment using its legacy pins
or replace the simulator with Gymnasium. A `pip check` conflict against those
legacy pins is expected; the executed imports and experiment checks, not that
old metadata, are the compatibility test. The headless launcher disables only
Pyglet's graphics shadow window, not physics or sensor computation.

### 2. Preserve the supplied reproduction before generating another

The runner refuses to overwrite existing experiment results. First check the
supplied release, then move its fresh results to a separate local directory:

```bash
mv results results_supplied
```

Do this only if `results_supplied` does not already exist. `data/published/`
always remains in place. To test installation without a full run:

```bash
.venv/bin/python scripts/reproduce.py --smoke
.venv/bin/python scripts/verify_smoke.py
```

This writes four evaluation runs under `results/smoke/`; it does not tune,
verify the full design or replace the full experiment. A full run is:

```bash
.venv/bin/python -u scripts/reproduce.py
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python scripts/verify.py
```

The full command regenerates maps and track geometry; runs 8 tuning
candidates x 2 development seeds; verifies the selected settings against
`frozen_tuning.json`; generates the randomized 360-row manifest; executes
all 72 nominal plus 288 stress simulations; and writes new raw observations,
summaries and provenance. Verification then independently reconstructs errors,
checks the design, audits the 100 Hz counter and compares with published data.
No archived time series are used to generate simulated states. An unsuccessful
check exits nonzero; a failed or partial execution is not a completed reproduction.

Numba's initial compilation adds startup time. Host speed and package/platform
differences can change wall-clock timings. The release records the measured
runtime rather than promising an embedded or real-time deadline.

### 3. Interpret agreement correctly

Categorical outcomes, run IDs, manifest order and sample counts are checked
directly. Numeric non-timing raw trajectories and run summaries are compared
with absolute tolerance `1e-8`, relative tolerance zero. Controller compute
time, total wall-clock time and gzip container bytes are not deterministic
scientific outcomes and are excluded. The tolerance is in each column's
stated units; consult the per-field discrepancy tables, not just a PASS label.

The published archive has only 50 Hz logs. Its exact ten-step 100 Hz departure
counter cannot be independently reconstructed from those decimated records.
The fresh reproduction records a passive 100 Hz trace for that additional
check without modifying the archived runner or feeding the trace into control.
Reproduction in one simulator is not independent physical validation.

## Experimental protocol

| Component | Executed setting |
|---|---|
| Physics / controller / main logger | 100 / 50 / 50 Hz; RK4 integration |
| Development tuning | Separate development track, 1.2 m/s, seeds 101 and 102 |
| PP candidates | 0.45, 0.65, 0.85, 1.05 m fixed arc preview |
| Stanley candidates | 0.6, 1.0, 1.4, 1.8 s^-1; speed softening 0.35 m/s |
| Selected settings | PP 0.45 m; Stanley 1.8 s^-1, frozen before evaluation |
| Evaluation tracks | Oval, stadium/hairpin, chicane; 1200 waypoints each, 1.30 m corridor |
| Phase A | 2 controllers x 3 tracks x 4 speeds (0.6, 1.0, 1.5, 2.0 m/s) x 3 seeds = 72 |
| Phase B | 16-point resolution-V half fraction x 2 controllers x 3 tracks x 3 seeds = 288 |
| A, speed | 0.6 / 2.0 m/s |
| B, requested delay | 0 / 75 ms, producing 0 / 80 ms steady-state sampled-state age |
| C, localization noise | Off / independent Gaussian x,y SD 0.010 m and yaw SD 0.50 degrees per update |
| D, steering bias | 0 / +2 degrees, between two clipping operations |
| E, bundled plant perturbation | Nominal / friction x0.80, mass x1.15, front/rear cornering parameters x0.85 |
| Fraction | E=ABCD, I=ABCDE; main/four-way and two-way/three-way aliasing |
| Evaluation seeds | 3101, 3102, 3103; separate matched pose/noise streams |
| Scheduling seed | 20260927; paired controller order and pair order randomized within phase/track |
| Completion proxy | Two laps, no collision flag and no 10 consecutive physics checks with absolute lateral error above 0.36 m |
| Accuracy | Completed second lap only; failed runs remain missing, not zero |
| Acceptance | Completion + lateral RMSE <=0.10 m + absolute-error p95 <=0.20 m + heading RMSE <=5 degrees |

Constant setpoints follow geometric centrelines, not optimized racelines.
Axle transformations are deliberately absent, as in the archived experiment.
Both chosen tuning parameters lie at candidate-grid boundaries. Positive bias
is one-sided; plant changes are bundled; tracks are fixed, not random samples.
For PP continuous factor screens, exclude the entire 2.0 m/s/80 ms cell,
leaving 108 assigned stress runs in all-completion cells. Binary rates retain
all 144 runs per controller. These are descriptive, aliased contrasts, not
independent causal effects or evidence of controller-family superiority.

See [protocol details](docs/PROTOCOL.md) and the
[data dictionary](docs/DATA_DICTIONARY.md) for signs, initial conditions,
delay-buffer warm-up, lap semantics, timeout and recorded fields.

## Sharing with reviewers and readers

This package is ready to upload to a repository; **no online repository has
been created by this workflow**. After author/coauthor and institutional
approval, create your GitHub repository and upload these contents preserving
the directory structure. Do not upload `.venv`, `vendor` or caches. Use Git
from your computer for the large compressed time series rather than the
browser uploader. Review `RIGHTS.md` and select code/data licences before
public release; no licence has been assigned on the authors' behalf.

For example, after creating an empty GitHub repository:

```bash
git init
git add .
git commit -m "Release F1TENTH verification code, data and reproduction evidence"
git branch -M main
git remote add origin https://github.com/YOUR_ACCOUNT/YOUR_REPOSITORY.git
git push -u origin main
```

Replace the example URL with your actual repository. Large files are kept
compressed and must remain unchanged; check your hosting service's current
file-size rules. For a blinded review, follow the venue's anonymity policy
rather than exposing an identifying public account. For public publication,
tag an immutable release, archive it in a DOI-granting repository if required,
and update `CITATION.cff` and the manuscript with the actual URL and release
identifier. A GitHub branch link alone does not freeze the reviewed evidence.

After intentional documentation/licensing edits, regenerate the release hash
manifest with `python scripts/check_integrity.py --write` and check it again.
Do not rewrite checksums to conceal unexpected numerical-data or source changes;
investigate those and rerun the scientific checks first.

AI assistance was used for coding, execution orchestration, analysis and
documentation. Python and the pinned simulator, not language-model-generated
numerical estimates, produced the fresh trajectories. Authors remain
responsible for the scientific claims, approvals and release permissions.

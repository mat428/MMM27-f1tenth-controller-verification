# Fresh reproduction report

Executed on 8 November 2026 in an assistant-provided Linux x86-64 workspace,
using CPU execution on an AMD EPYC 9V74 host. This report describes commands
that actually executed, not estimated or invented simulator outcomes.

## Scope and source

Upstream F1TENTH Gym commit:
`4fdb9c7e6fb7c701290f4dc18377d07c1681724f`.
The archived `doe_campaign.py`, `reference_campaign.py` and original
`audit_analyze.py` are retained unchanged. The new launcher handles headless
imports, provenance, fresh tuning trace retention and passive boundary
observation; it does not change controller equations, plant parameters,
random seeds, map geometry, the experimental design or acceptance rules.

Python 3.12.14, NumPy 2.3.5, SciPy 1.17.0, pandas 2.2.3, Gym 0.26.2,
Numba 0.63.1, llvmlite 0.46.0, Pillow 13.0.0 and Pyglet 1.4.11 were used.
Other exact packages are pinned in requirements.txt. Full host/software
details are in results/execution.json. Execution was sequential, without
ROS 2, a GPU, a physical car or an embedded processor.

## Execution ledger

| Stage | Actual execution / outcome |
|---|---|
| Simulator retrieval | Retrieved upstream source and checked the archived commit |
| Initial installation smoke | Four runs completed; excluded from the experiment and release data |
| Fresh development tuning | 8 candidates x 2 seeds = 16 completed trials; all successful |
| Selected settings | PP preview 0.45 m; Stanley gain 1.8 s^-1; unchanged |
| Historical versus fresh candidate scores | Maximum absolute difference 0.0 m |
| Fresh evaluation | All 360 assigned runs executed: 72 nominal + 288 fractional-factorial stress |
| Final clean-environment evaluation wall-clock time | 419.28 s reported by the archived runner |
| Final clean-environment tuning/evaluation launcher time | 433.23 s; host-specific, not an embedded timing claim |
| Preliminary full execution | 431.9 s evaluation / 445.38 s launcher; its receipts are retained separately |
| Second clean environment | Bootstrap installed dependencies and the pinned simulator from scratch |
| Clean-install issue and correction | Initial Numba import failed because the installed llvmlite shared library was truncated; reinstalling llvmlite 0.46.0 restored the full library |
| Clean-environment smoke | Four runs subsequently completed and were compared with the archive; not counted as additional experiment replicates |
| Fast offline tests | Four tests passed: signs, wrapping, preview indices, design/pairing |

The full evaluation used an isolated virtual environment sharing the workspace's
preinstalled numerical packages at the recorded pinned versions. A separate,
non-system-site-packages environment was then installed with the supplied
bootstrap and exercised with the four-run smoke test and offline verification.
The full campaign was subsequently repeated again in that clean environment;
the delivered results/ directory contains that final clean-environment run.
The preliminary execution and verification receipts are retained under
provenance/, without duplicating its large raw files. Separate executions
reuse the same seed/design and must not be pooled as independent replicates.

## Scientific comparison

| Response | Published | Fresh reproduction |
|---|---:|---:|
| Evaluation runs | 360 | 360 |
| 50 Hz raw rows | 833,662 | 833,662 |
| Nominal PP completions | 36/36 | 36/36 |
| Nominal Stanley completions | 36/36 | 36/36 |
| Stress PP completions | 116/144 | 116/144 |
| Stress Stanley completions | 144/144 | 144/144 |
| Stress PP combined acceptances | 108/144 | 108/144 |
| Stress Stanley combined acceptances | 98/144 | 98/144 |
| Nominal PP-minus-Stanley mean lateral RMSE | -0.04309924354613917 m | -0.04309924354613917 m |

All 33 compared non-timing run-summary fields and 13 non-timing time-series
fields matched. Numeric fields had maximum observed absolute difference
**0.0** under the comparison parser; the acceptance tolerance was absolute
1e-8 with zero relative tolerance. Controller compute times, execution times
and gzip container bytes were explicitly excluded. A successful match is
not an assertion that different hardware or future software must match bitwise.

The original data passed 19 independent checks. Its 100 Hz departure counter
remains independently untestable from the decimated original log, marked
`null` rather than PASS. Fresh data passed 21 checks, including both the
100 Hz projection reconstruction and the consecutive-departure counter.
Five archived-versus-fresh/tuning comparison checks also passed, including
recalculation from all 16 newly recorded tuning trajectories. These checks test
record consistency and computational repeatability, not all aspects of
scientific validity. See verification_report.json for authoritative results.

## Interpretation and remaining author actions

No original experimental observations were replaced, no failed-run RMSE was
imputed and no original/fresh runs were pooled. The fresh reproduction does
not add controller baselines, reverse-track trials, a causal failure mechanism,
physical validation or an independent simulator. It confirms repeatability
of this implementation and its numerical records in the tested environment.

The online GitHub repository has not yet been created. Authors must choose
its actual address, confirm release/licensing and coauthor permissions,
publish an immutable release, and replace the obvious placeholder in the
separate LaTeX manuscript. No URL, DOI, public deposit or human approval has
been fabricated. The native EAP eight-page layout still needs Overleaf checking.

If Numba fails to import with a bus error and a truncated libllvmlite.so is
confirmed, a clean reinstall in the disposable environment was sufficient here:

```bash
.venv/bin/python -m pip install --force-reinstall --no-cache-dir --no-deps llvmlite==0.46.0
```

Do not treat that command as a general fix for every native-library failure;
inspect the actual error and environment first.

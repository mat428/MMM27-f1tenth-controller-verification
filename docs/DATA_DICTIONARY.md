# Data dictionary

One experimental unit is one two-lap simulation, not one logged sample.
`data/published/` is the unchanged original campaign. `results/campaign/`
is the fresh reproduction. Do not pool them as independent random replicates:
they reuse the same deterministic scenarios and seeds.

## Files

| File | Meaning |
|---|---|
| scenario_manifest.csv | Prespecified 360 rows: factors, controller, track, pose seed, order, pair ID |
| run_summary.csv | One row per run, completion, missingness and conditional accuracy |
| timeseries.csv.gz | Gzip-compressed, 50 Hz, true-state observations and control outputs |
| metadata.json | Simulator commit, principal package versions, source hash, frozen tuning and run count |
| boundary_checks.csv.gz | Fresh run only: passive 100 Hz projection and consecutive-exceedance trace, including the terminating check |
| results/execution.json | Fresh host/CPU/software provenance and start/end/status; not an embedded benchmark |
| results/tuning/controller_tuning.csv | Fresh eight-candidate, two-seed-per-candidate selection scores |
| results/tuning/run_summary.csv | Fresh 16 development-track run summaries |
| results/tuning/TUNE_*.csv.gz | Fresh development-track per-run trajectories |
| verification/verification_report.json | Independent audit and archived-versus-fresh comparison outcomes |
| verification/*comparison.csv | Per-field numeric/categorical agreement and maximum absolute discrepancies |
| verification/restricted_and_binary_screens.csv | Paper-aligned restricted-domain PP continuous screens and full-design binary contrasts |
| verification/high_speed_aged_combinations.csv | C/D/E combinations in the 2.0 m/s, aged-state cell, with explicit completion denominators |
| verification/threshold_failures.csv | Overlapping accuracy-threshold failures, separated from incomplete runs |

Historical `factor_effects_descriptive.csv`, `nominal_cells.csv`,
`minitab_worksheet.csv`, `audit_report.json` and `recheck_report.json` are
retained in `data/published/` unchanged for provenance. The historical factor
table uses available complete cases and must not replace the restricted-domain
analysis in `verification/`. `src/audit_analyze.py` is likewise retained
unchanged; use `scripts/verify.py` for the current paper-aligned verification.

## 50 Hz time-series fields

| Field | Unit / meaning |
|---|---|
| run_id | Manifest identifier, DOE001--DOE360 |
| time_s | Simulated time, seconds; not wall-clock time |
| lap_count | Gym's lap-progress counter; second-lap metrics use values >=1 |
| x_m, y_m | True simulator pose-origin position, metres |
| yaw_rad | True simulator yaw, radians |
| speed_mps | True longitudinal speed, m/s |
| path_index | Nearest Euclidean reference waypoint, zero-based |
| lateral_error_m | True signed tangent-frame error; positive to the left |
| heading_error_rad | Wrapped true yaw minus path tangent, [-pi, pi) |
| curvature_1pm | Discrete reference curvature, 1/m |
| steering_command_rad | Output after controller clipping, optional +2-degree bias and re-clipping |
| state_age_s | Age of the state passed to the controller; not total sensor-to-actuator latency |
| compute_ms | Host time spent evaluating the controller function, ms; not embedded timing |

## Summary flags and metrics

`completed_two_laps` reflects Gym's completion return, `boundary_departure`
the sustained geometric proxy, and `collision` Gym's collision flag.
`safe_completion` combines these flags; the word *safe* names the study proxy,
not proven vehicle safety. `valid_accuracy` additionally requires sufficient
second-lap samples. Failed runs have missing RMSE/percentile accuracy, not zero.

`lateral_rmse_m`, `lateral_p95_m`, `lateral_max_m` and `heading_rmse_deg`
are evaluated on the completed second lap. `accepted` is computed by the
verification script: completion plus RMSE <=0.10 m, lateral absolute-error
95th percentile <=0.20 m, and heading RMSE <=5 degrees. These are study limits.

`duration_s` is simulated duration; `logged_samples` counts all 50 Hz rows;
`second_lap_samples` counts rows with lap_count >=1, even if accuracy is
ultimately invalid. `compute_p95_ms` is host-specific. `stop_reason` records
the runner's classification; timeout and other stops share one category.

Factors use -1/+1 coding: A speed 0.6/2.0 m/s; B requested state age
0/75 ms (actual steady-state 0/80 ms); C localization noise off/on;
D steering bias 0/+2 degrees; E nominal/bundled plant perturbation.
The four nominal speeds and three-track blocks are described
in the README. The Phase A `speed_code` is not a continuous-speed encoding.

# Execution details and scope

## Reference geometry and steering

The unmodified `src/reference_campaign.py` defines the four tracks, raster
maps, plant parameters and both control laws. Its Pure Pursuit advance is
`round(0.45 / (track_length/1200))`, using Python rounding and a minimum of
one waypoint. This gives 26, 23 and 24 indices on the oval, stadium/hairpin
and chicane. The denominator uses actual target distance, lower-bounded at
0.05 m; the nominal preview is arc length, not a fixed Euclidean distance.

Both controllers evaluate the simulator pose origin. The nearest waypoint
comes from a KD-tree, with no segment projection or target interpolation.
True positive lateral error is left of the path tangent. Stanley combines
wrapped path-minus-vehicle heading with `atan2(-k*e_y, abs(v)+0.35)`.
PP uses `atan2(2*0.3302*y_target_local, actual_distance**2)`.
Each function clips to +/-0.4189 rad; the runner adds any positive steering
bias and clips again. These are not canonical axle-referenced controllers.

## Initial state, direction and laps

Each reset starts at waypoint zero with independent x/y Gaussian jitter of
SD 0.008 m and yaw jitter of SD 0.35 degrees. Upstream `RaceCar.reset()`
zeros its dynamic state and then sets x, y and yaw; it does not initialize
velocity at the target setpoint. The subsequent command supplies the selected
speed setpoint to the simulator's internal longitudinal controller.
Reset includes one zero-command physics step, so the first logged observation
is at 0.01 simulated seconds with zero speed. In waypoint order the oval and
chicane are counter-clockwise, while the stadium/hairpin is clockwise;
directions were checked by the signed polygon area of the generated paths.

The experiment uses upstream Gym's `lap_counts`, not an independently
implemented waypoint-index wrap. The pinned environment's reset and
near-start crossing logic in `gym/f110_gym/envs/f110_env.py` is authoritative.
In coordinates (u,w) relative to the jittered reset pose and rotated into its
heading frame, its near-start zone is `u**2 + max(abs(w)-2,0)**2 <= 0.1`.
Each change between near and away increments a toggle; lap_count is
`toggle_count//2`, and the terminal completion condition is four toggles.
Study completion requires the returned terminal condition with lap_count >=2,
no collision and no sustained departure. Second-lap accuracy uses logged
lap_count >=1 and at least 20 samples, conditional on completion.

## Noise, delay and command holding

Pose and noise generators are seeded independently, with seeds `seed` and
`seed+100000`. Their streams are matched between controllers. Every 20 ms,
selected pose noise is applied before adding the measurement and speed to
the timestamped buffer. A 75 ms request is rounded upward to four controller
periods. At offsets 0, 20, 40 and 60 ms from the first controller update,
current samples are used; from offset 80 ms onward, samples four updates
earlier are used. The corresponding absolute simulator timestamps are
10, 30, 50, 70 and 90 ms because reset performs one physics step.
State age switches from 0 to 80 ms,
not 0/20/40/60/80. Without requested delay, age stays zero. Commands are held
between controller updates. Gym's internal steering buffer is distinct from
this state-age buffer. None of these are measured ROS/network/physical delays.

## Departure proxy, collision and timeout

Nearest-waypoint projection is evaluated before each 100 Hz physics advance.
The consecutive counter increments when absolute error exceeds 0.36 m and
resets otherwise. At ten consecutive checks, the loop exits before a possible
50 Hz log write. Thus the terminating check need not appear in the old logs.
The threshold is the historical study proxy `1.30/2 - 0.58/2`; it is not exact
body containment for arbitrary yaw and says nothing about brief excursions.

The timeout is `min(100, 2.7*track_length/max(speed_setpoint, 0.35)+8)` simulated
seconds. Scan-based instantaneous TTC termination is disabled by setting each
agent's threshold to -1. Inter-agent collision checking adds no independent
single-vehicle wall protection. No extra footprint/map-wall check is supplied.

## Fresh passive observation

`scripts/reproduce.py` wraps the existing campaign's calls to `nearest_state`
to save the per-physics-step position, projection and consecutive counter.
It returns exactly the original projection value, does not alter simulated
state, does not add controller feedback, and does not replace termination.
This costs host time; compute and total wall-clock times are not reproduction
targets. `scripts/verify.py` independently recalculates that trace from x/y
and checks the departure flags. This additional trace exists only for the
fresh campaign, not retroactively for the original observations.

## What reproduction cannot establish

Matching the same implementation in the same simulator does not validate
its vehicle model against a physical car, establish controller-family
superiority, identify independent aliased disturbance causes, or test
canonical-axle, adaptive-preview, reverse-direction or unbundled baselines.
The tested 3-track/3-seed finite design is not a population sample of roads.

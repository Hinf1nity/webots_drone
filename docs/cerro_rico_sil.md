# Cerro Rico mine and software-in-the-loop validation

## Measured outcome — 2026-10-13

**The mine integration runs, but the current closed-loop system does not pass the
complete traversal acceptance criteria.** Both final trials completed all 130
simulated seconds with no execution error. See the
[saved evidence, plots, images, and per-check results](../validation/cerro_rico/report.md).

| Measurement | Practical lights on | Practical lights off |
| --- | ---: | ---: |
| Altitude RMSE | 0.0091 m | 0.0100 m |
| Roll/pitch estimation RMSE | 2.33° | 2.04° |
| Peak tilt | 10.56° | 9.89° |
| Observed post-takeoff contact samples | 0 | 0 |
| Saturated-motor fraction | 0% | 0% |
| Odometry position RMSE | 0.2518 m | 0.3561 m |
| Drift at last valid odometry estimate | **0.5458 m — FAIL** | **0.6108 m — FAIL** |
| Last valid odometry timestamp (130 s trial) | **48.568 s** | **44.304 s** |
| Return-position error | **2.4680 m — FAIL** | **1.3611 m — FAIL** |
| Forward progress | 3.2821 m | **2.1711 m — FAIL** |
| Map updates | 43 | 40 |
| Evaluated map points (every second published cloud point) | 47,412 | 46,330 |
| Map-to-sampled-surface P95 | 0.3501 m | 0.4146 m |
| Median image intensity / contrast (0–255) | 14.35 / **7.45 — FAIL** | **8.47 / 3.52 — FAIL** |
| Overall acceptance | **FAIL** | **FAIL** |

All specified message-arrival-rate checks passed. **This does not mean localization
remained valid:** the odometry publisher continued emitting invalid/null estimates
after tracking was lost. The original rate check counted these messages; RMSE and
the JSON `end_drift_m` field only cover retained valid poses. The drift values above
are therefore at the last valid estimate, not at 130 s. The original result JSON
is retained, with this interpretation correction.

The ICP logs report registration corrections exceeding the configured 0.2 m
limit, followed by failed correspondence matching. Once the last valid estimate
was more than 0.5 s old, the test fixture stopped horizontal commands. Tracking
was lost before the return leg scheduled at 70 s in both runs, so neither run
actually completed a valid odometry-controlled return attempt.

Ground-truth position differences over 0.48 s windows also show horizontal speeds
up to approximately **1.10 m/s nominal / 1.02 m/s low-light**, despite the fixture's
0.2 m/s desired-velocity limit. That limit bounds a command, not actual speed.
Poor velocity tracking and motion-estimation loss need isolated follow-up tests;
the data do not identify one exclusive cause of the initial ICP failure.

These results demonstrate altitude and
attitude stabilization and functioning local 3D mapping under the configured
simulation assumptions. They do **not** establish reliable autonomous mine
traversal: the ICP-odometry/test-fixture route loop failed to return within 0.5 m,
and drift already exceeded the 0.3 m limit at the last valid pose in both cases. Camera contrast was
below the selected threshold even with practical lights enabled.

The next architectural work indicated by these measurements is better drift
control/state estimation, a tuned navigation/position-control loop with explicit
obstacle clearance, and evaluated onboard illumination/camera exposure. The
route controller here is a test fixture, so its tracking failures should not be
attributed solely to the airframe or low-level attitude controller.
The validation harness also needs explicit **valid-odometry availability,
time-to-tracking-loss, and recovery** criteria so a live publisher cannot mask a
failed estimator.

These are **one final pilot run per lighting case**, not statistical confidence
bounds. Asynchronous ROS/simulator scheduling produces run-to-run variation even
with a fixed world seed; diagnostic runs also exhibited collisions and larger
drift. The two cases followed different realized trajectories, so differences
in odometry are not an isolated causal measurement of illumination effects.
Acceptance thresholds were kept unchanged throughout the trials. The final map
scoring uses a fixed initial map-frame alignment rather than moving the reference
frame with later SLAM corrections.

## Environment

`02_Cerro_Rico_Messy.blend` is the source of `cerro_rico.wbt`. The export retains
Blender XYZ coordinates in metres, with Webots Z up. The source hash, geometry
bounds, Blender version, light inventory, spawn marker, and entrance floor/roof
probes are recorded in
`src/thaiquiri_webots/webots_world/protos/cerro_rico/manifest.json`.

- Full mine: approximately 100 × 94 × 7.7 m, including connected tunnels, timber
  supports, rubble, rails, signs, carts, and packed rock/dirt/timber textures.
- Visual export: 2,127 rendered source objects, converted to meshes and joined
  while preserving material assignments and UVs.
- Physics: the artist-supplied **SIM COLLISION** mesh, 41,661 vertices and 77,774
  faces, used as a static concave collider. Small visual rubble not represented
  in that simplified mesh is not a physical collision obstacle.
- Practical light positions/colors are preserved. Wattage is converted to
  relative Webots intensity with inverse-square attenuation; illumination is
  **not photometrically calibrated**. The Blender inspection torch is omitted.
- A 1.15 × 1.15 m level launch fixture has its top at Z=0.20 m at `(0, 3)`.
  This explicitly added fixture permits repeatable stationary IMU initialization
  above the rough source floor. The drone starts on it facing the +Y corridor.
- The original mine world remains available as `mine_thaiquiri.wbt`.

## Run interactively

From the repository root, after the native uv setup in `readme.md`:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python -m colcon build --symlink-install --packages-select thaiquiri_webots
source install/setup.bash
export WEBOTS_HOME="$HOME/.local/opt/webots"
ros2 launch thaiquiri_webots drone_launch.py world:=cerro_rico.wbt
```

Use `rtabmap:=false` for sensor-only operation, `rtabmap_viz:=false` to suppress
RTAB-Map's GUI, and `gui:=false` to minimize Webots (a display is still needed).
The initial controller target is a **0.5 m altimeter reading**, approximately
the sensor's resting elevation on the fixture; use teleop `t` to climb. The SIL
runner issues its own altitude commands.

Each mapping launch deletes its selected RTAB-Map database on startup. Set
`database_path:=/absolute/path/to/a/new/trial.db` to keep experiments separate.

## Rebuild the assets

Blender 4.2.0 was used for the source's 4.02 file format. Blender is an export-time
tool; it is not required to run the exported Webots assets.

```bash
~/.local/opt/blender-4.2.0-linux-x64/blender \
  -b 02_Cerro_Rico_Messy.blend --python scripts/export_cerro_rico.py
python -m colcon build --symlink-install --packages-select thaiquiri_webots
```

The exporter does not save or modify the source `.blend`. It exports packed
textures, repairs their relative MTL references, exports the supplied collision
mesh, and generates a deterministic area-weighted collision-surface point sample
for map evaluation. Re-export and rebuild after editing the Blender source.

## What is under test

The system under test is the repository's **active Python flight controller**,
four Webots propellers, stereo cameras, rotating LiDAR, Cube IMU processing,
and RTAB-Map with ICP odometry. This is software-in-the-loop simulation of that
architecture, **not ArduPilot SITL**: the alternate ArduPilot driver and an
external autopilot firmware are not exercised.

```text
estimated ICP odometry -> SIL route fixture -> /cmd_pos -> flight controller -> motors
                            (test input)                        ^
                                                         IMU + altimeter
Webots stereo -> stereo_sync RGBDImage --+
Webots LiDAR ---------------------------+-> RTAB-Map -> map / ICP odometry
Webots true pose + contact data ---------------------> metrics only
```

The runner adds a small odometry-feedback route controller as a **test fixture**.
It sends the existing `/cmd_pos` interface; it is not a production mission planner
or obstacle-avoidance system. It uses `/drone/odom`, not simulator truth, for
horizontal feedback. An independent Webots supervisor publishes
`/sil/ground_truth` and `/sil/contact_count` for scoring only, without publishing
TF or feeding the estimator/controller.

### Trial protocol and acceptance criteria

`config/sil_criteria.json` defines the engineering targets before a run:

1. Initialize on the fixture and raise the altimeter target to 1.05 m at 5 s.
2. Command 1.15 m at 15 s and evaluate settled flight from 30 s onward.
3. From 20 s, follow a 3 m forward waypoint using estimated odometry, with
   commanded horizontal speed limited to 0.2 m/s; return after 70 s.
4. Stop at 130 simulated seconds; record startup/flight wall time and simulated
   time separately. A wall-time deadline or early process exit is a failed run.
5. Repeat with practical lights disabled (`low_light`) to expose camera/mapping
   dependence on illumination. The same criteria apply, so stress-case failures
   are reported rather than relaxed away.

The low-light case disables PointLight illumination; emissive lamp-glass
materials remain visible. It is a relative lighting stress case, not calibrated
lux testing or a zero-photon darkness model.

Checks include altitude and attitude-estimation RMSE, peak tilt, motor saturation,
contact samples, route progress/return, sensor and odometry rates, camera exposure
and contrast, map updates/points, odometry drift, and map surface residuals.
Missing measurements fail the corresponding acceptance checks.

Settled altitude/attitude errors are evaluated from 30 s; contact, peak tilt, and
motor-saturation checks include all samples after the initial 10 s takeoff window.
These thresholds are in the versioned JSON rather than chosen from observed results:

| Property | Target |
| --- | --- |
| Altitude RMSE | ≤ 0.10 m |
| IMU roll/pitch error RMSE against true body attitude | ≤ 5° |
| Peak absolute roll/pitch | ≤ 15° |
| Post-takeoff contacts / saturated-motor fraction | 0 / ≤ 1% |
| Odometry position RMSE / final drift | ≤ 0.30 m / ≤ 0.30 m |
| Forward progress / return-position error | ≥ 2.5 m / ≤ 0.50 m |
| IMU / camera / LiDAR / odometry rate | ≥ 20 / 5 / 3 / 3 Hz (simulation time) |
| Map updates / sampled cloud points | ≥ 20 / ≥ 1,000 |
| Sampled map-surface distance P95 | ≤ 0.50 m |
| Median camera grayscale mean / spatial standard deviation | ≥ 10 / ≥ 8 on 0–255 scale |

Odometry is aligned to truth with **one initial SE(3) pose transform**. No scale
or best-fit trajectory adjustment is used. Map points are transformed using this
same alignment and the **first** published map-to-odom transform (kept fixed
throughout the trial, including loop closures), then compared to the
deterministic sampled collision surface. Nearest-sample distances include sampling
and collision-simplification error; they are not a surveyed real-mine accuracy
claim. A dense map or a small surface residual alone does not establish map
completeness or loop-closure correctness.

## Execute the SIL trials

Stop any other Webots launch on port 1234 and prepare the sourced uv terminal
above. Output directories must be new:

```bash
python scripts/run_sil.py --scenario nominal \
  --output experiment_data/sil/nominal
python scripts/run_sil.py --scenario low_light \
  --output experiment_data/sil/low_light
```

Exit code 0 means all specified checks pass; exit code 1 means a failed check or
execution failure. The full mine can take longer to load than the original arena.
The runner uses ROS domain 73 by default (`--domain-id` changes it), checks that
Webots port 1234 is free, and terminates its complete process groups after a run.

Each run saves `result.json`, `report.md`, time-stamped truth/odometry/flight CSVs,
image and LiDAR-quality CSVs, a camera image, map points when available, a unique
RTAB-Map database, and process logs. Metadata includes the source hash, criteria
hash, implementation file hashes, Webots version, Git revision/dirty state,
lighting scenario, and world seed.

The included `validation/cerro_rico/` bundle contains the final JSON results,
CSV evidence, camera images, map point samples, and plots. Full process logs and
RTAB-Map databases remain in the ignored `experiment_data/sil/` run directories.

Create a shareable summary from completed runs:

```bash
python scripts/summarize_sil.py experiment_data/sil/nominal \
  experiment_data/sil/low_light --output validation/cerro_rico
```

### Mapping integration detail

The direct RTAB-Map stereo callback does not include the LiDAR scan in its
synchronized payload, even when `subscribe_scan_cloud` is set. The launch now
uses `stereo_sync` to package the camera pair as `RGBDImage`, then synchronizes
that message with the LiDAR cloud. ICP is selected for registration and 3D grid
generation is enabled. The first trial exposed the former configuration as
`Cannot create local map from scan: scan is empty`; that configuration cannot
substantiate a LiDAR mapping claim merely by publishing `mapData` messages.

Regression checks:

```bash
python -m pytest -q src/thaiquiri_webots/test/test_control.py \
  src/thaiquiri_webots/test/test_sil_metrics.py \
  src/thaiquiri_webots/test/test_cerro_assets.py
```

## Interpretation limits

The trial covers the entrance corridor, not the complete mine or every narrow
branch. It does not validate communications loss, realistic dust/aerosols,
battery/endurance, motor failure, gas sensing, structural strength, or an actual
GNSS-denied flight computer. Sensor noise, lighting, drag, and motor response are
the configured simulation models. Observed failures and untested capabilities
must remain visible when judging readiness for further development.

In particular, the Webots altimeter measures world elevation; these altitude
results do not establish barometric performance in a real mine. The configured
propeller response/damping, idealized camera calibration, and low sensor noise
are simulation assumptions rather than experimentally identified UAV hardware
models. The main reliability question in this trial is whether this configured
software pipeline can stabilize, estimate motion, and map the imported geometry
while following a short commanded route.

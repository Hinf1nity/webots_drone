# Cerro Rico SIL result

Scenario: nominal
Overall: FAIL

| Check | Result | Value | Limit |
| --- | --- | --- | --- |
| completed_duration | PASS | 130.0 | 129.9 |
| clock_monotonic | PASS | 0 | 0 |
| altitude | PASS | 0.009139632872588484 | 0.1 |
| attitude | PASS | 2.330335114862786 | 5.0 |
| tilt | PASS | 10.561133965352942 | 15.0 |
| contacts | PASS | 0 | 0 |
| motor_saturation | PASS | 0.0 | 0.01 |
| odometry | PASS | 0.25177492315387495 | 0.3 |
| odometry_end_drift | FAIL | 0.5457513660576174 | 0.3 |
| route_progress | PASS | 3.2820517142012773 | 2.5 |
| route_return | FAIL | 2.468015590086706 | 0.5 |
| map_updates | PASS | 43 | 20 |
| map_points | PASS | 47412 | 1000 |
| map_surface | PASS | 0.3500856556276521 | 0.5 |
| imu_rate | PASS | 76.44748327029386 | 20.0 |
| camera_rate | PASS | 115.96232179331504 | 5.0 |
| lidar_rate | PASS | 7.435045978689243 | 3.0 |
| odometry_rate | PASS | 7.344375682562796 | 3.0 |
| camera_exposure | PASS | 14.350035807291667 | 10.0 |
| camera_contrast | FAIL | 7.448346211001957 | 8.0 |

Ground truth is scoring-only. The odometry-driven route controller is a test fixture, not a production autonomous navigation stack.
This run validates only the entrance corridor and stated conditions, not the full mine, real-world flight, ArduPilot SITL, or gas sensing.
